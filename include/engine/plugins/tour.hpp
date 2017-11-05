//
// Created by René Brüggemann on 20.04.17.
//

#ifndef TOUR_HPP
#define TOUR_HPP


#include "engine/plugins/plugin_base.hpp"
#include "engine/engine_config.hpp"
#include "engine/api/tour_api.hpp"
#include "engine/datafacade/datafacade_base.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/search_engine_data.hpp"
#include "util/json_container.hpp"
#include "util/typedefs.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/dynamic_graph.hpp"
#include "util/static_graph.hpp"
#include "util/fingerprint.hpp"
#include "util/graph_loader.hpp"
#include "util/coordinate.hpp"
#include "util/timing_util.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/function_output_iterator.hpp>
#include <boost/unordered_map.hpp>

namespace osrm
{
    namespace engine
    {
        namespace plugins
        {

            struct TourEdgeData
            {
                TourEdgeData() : weight(INVALID_EDGE_WEIGHT), distance(INVALID_EDGE_DISTANCE) {}
                TourEdgeData(EdgeWeight _weight, EdgeDistance _distance) :
                        weight(_weight), distance(_distance){}
                EdgeWeight weight;
                EdgeDistance distance;
            };

            struct ReduceResult
            {
                ReduceResult() : nodeID(SPECIAL_NODEID), edgeData(TourEdgeData()) {}
                ReduceResult(NodeID _nodeID, TourEdgeData _edgeData) : nodeID(_nodeID), edgeData(_edgeData) {}
                NodeID nodeID;
                TourEdgeData edgeData;
            };

            using TourGraph = util::StaticGraph<TourEdgeData>;
            using TourEdge = TourGraph::InputEdge;
            using QueryHeap = util::
            BinaryHeap<NodeID, NodeID, EdgeWeight , HeapData, util::UnorderedMapStorage<NodeID, int>>;
            template <class K, class I>
            using Map = std::unordered_map<K,I>;

            class TourPlugin final : public BasePlugin
            {

                private:
                    const storage::StorageConfig storageConfig;

                    std::vector<util::Coordinate> coordinate_list;
                    std::vector<NodeID> nodes_of_interest;

                    TourGraph old_graph;
                    TourGraph graph;
                    mutable EdgeDistance length;
                    mutable float epsilon;
                    mutable NodeID bestMidwayPoint;
                    mutable EdgeWeight bestWeight;
                    mutable EdgeDistance bestDistance;
                    mutable Map<NodeID , std::set<NodeID>> P;


                void dijkstra(NodeID start, const EdgeDistance length,
                              Map<NodeID, NodeID > &parent,
                              Map<NodeID , std::int32_t> &weight,
                              Map<NodeID , EdgeDistance> &distance) const;

                void FindMidwayPoint(NodeID s, Map<NodeID ,NodeID> candidate) const;

                void SearchMidwayPoint(NodeID s, Map<NodeID, NodeID> candidate) const;

                NodeID SearchNearestNodeOfInterest(util::Coordinate coordinate) const;

                std::vector<ReduceResult> ReduceEdges(const NodeID start, const unsigned int degree, TourGraph &graph){
                    std::vector<ReduceResult> result;

                    QueryHeap queryHeap(graph.GetNumberOfNodes());

                    queryHeap.DeleteAll();
                    queryHeap.Clear();

                    Map<NodeID, NodeID> parent;
                    Map<NodeID, EdgeWeight> weight;
                    Map<NodeID, EdgeDistance> distance;

                    TourEdgeData data;
                    NodeID source, target;

                    // insert start node
                    parent[start] = start;
                    weight[start] = 0;
                    distance[start] = 0;

                    queryHeap.Insert(start, distance[start], parent[start]);

                    while(!queryHeap.Empty()){
                        source = queryHeap.DeleteMin();

                        for(const EdgeID edge : graph.GetAdjacentEdgeRange(source)){
                            data = (TourEdgeData &)graph.GetEdgeData(edge);
                            target = graph.GetTarget(edge);

                            if(target == SPECIAL_NODEID) {
                                continue;
                            }

                            EdgeDistance ways_length = distance[source] + data.distance;
                            EdgeWeight ways_weight = weight[source] + data.weight;

                            if(!queryHeap.WasInserted(target)) {
                                weight[target] = ways_weight;
                                distance[target] = ways_length;

                                if(graph.GetOutDegree(target) >= degree){
                                    auto it = std::find(nodes_of_interest.begin(), nodes_of_interest.end(), target);

                                    BOOST_ASSERT_MSG(
                                            it != nodes_of_interest.end(),
                                            "Node not found in node_of_interest");

                                    auto pos = std::distance(nodes_of_interest.begin(), it);
                                    result.emplace_back(pos,TourEdgeData(weight[target], distance[target]));
                                }else{
                                    queryHeap.Insert(target, distance[target], source);
                                }
                            }else if(ways_length < distance[target]){
                                weight[target] = ways_weight;
                                distance[target] = ways_length;
                                queryHeap.GetData(target).parent = source;
                                queryHeap.DecreaseKey(target, distance[target]);
                            }
                        }
                    }

                    return result;
                }

                void loadGraph(const std::string &path, TourGraph &graph)
                {
                    EdgeDistance distance = .0;
                    storage::io::FileReader file_reader(path, storage::io::FileReader::VerifyFingerprint);

                    // load graph data
                    std::vector<TourEdge> graph_edge_list;
                    std::vector<extractor::NodeBasedEdge> edge_list;
                    util::PackedVector<OSMNodeID> osm_node_ids;

                    auto nop = boost::make_function_output_iterator([](auto) {});

                    const auto number_of_nodes =
                            util::loadNodesFromFile(file_reader, nop, nop, coordinate_list, osm_node_ids);

                    util::loadEdgesFromFile(file_reader, edge_list);

                    // Building a node-based graph
                    for (const auto &input_edge : edge_list)
                    {
                        if (input_edge.source == input_edge.target)
                        {
                            continue;
                        }

                        // TODO: CONTRACTION PROCESS
                        distance = util::coordinate_calculation::haversineDistance(coordinate_list[input_edge.source],
                                                                                   coordinate_list[input_edge.target]);

                        if (input_edge.forward)
                        {
                            graph_edge_list.emplace_back(input_edge.source, input_edge.target, input_edge.weight,
                                                         distance);
                        }

                        if (input_edge.backward)
                        {
                            graph_edge_list.emplace_back(input_edge.target, input_edge.source, input_edge.weight,
                                                         distance);
                        }
                    }

                    tbb::parallel_sort(graph_edge_list.begin(), graph_edge_list.end());
                    graph = TourGraph(number_of_nodes, graph_edge_list);
                }


                void loadTourGraph(TourGraph &graph, TourGraph &new_graph, const unsigned int degree)
                {
                    std::vector<TourEdge> graph_edge_list;

                    for(NodeID i = 0; i < graph.GetNumberOfNodes(); i++){
                        if(graph.GetOutDegree(i)>=degree){
                            nodes_of_interest.emplace_back(i);
                        }
                    }

                    util::Log() << "nodes of intresst: " << nodes_of_interest.size();

/*
                    NodeID parent;
                    NodeID target;
                    NodeID inner_target;
                    EdgeDistance distance;
                    EdgeWeight weight;
                    TourEdgeData data;
                    TourEdgeData inner_data;
                    bool deadend;
*/

                    int step = 0;
                    TIMER_START(contract);
                    for(NodeID id = 0; id < nodes_of_interest.size(); id++){
                        // PROGRESS IMPROVE OUT
                        if(id > step*0.1*nodes_of_interest.size()){
                            step++;
                            if(step == 1) std::cout << step * 10 << "%\r";
                            else if(step == 10) std::cout << " - " << step * 10 << "%\r\n";
                            else std::cout << " - " << step * 10 << "%\r";
                            std::cout.flush();
                        }

                        for(ReduceResult result : ReduceEdges(nodes_of_interest[id], degree, graph)){
                                /**
                                 *  Um einen Static_Graph zu erstellen, müssen die Knoten in einer Listen ohne Lücken
                                 *  vorhanden sein! Der Konstruktor von Static_Graph nutzt ranges und differenzen von
                                 *  Abbständen.
                                 *
                                 *  Beispiele:
                                 *
                                 *       #1 for (auto node : util::irange(0u, nodes))
                                 *
                                 *       #2 offset = std::distance(begin, iter);
                                 *
                                 *  Eine Übersetzung der "lückenhaften" Knotenliste zu einer "lückenlosen" Knotenliste
                                 *  ist unbedingt notwendig!
                                 */

                                graph_edge_list.emplace_back(id, result.nodeID, result.edgeData.weight, result.edgeData.distance);
                        }
                    }
                    TIMER_STOP(contract);
                    util::Log() << "Grahp_edge_list size: " << graph_edge_list.size();
                    util::Log() << "Contraction took " << TIMER_MIN(contract) << " min and " << (int)TIMER_SEC(contract) % 60 << " sec";

                    tbb::parallel_sort(graph_edge_list.begin(), graph_edge_list.end());
                    new_graph = TourGraph(nodes_of_interest.size(), graph_edge_list);
                }

                public:
                    explicit TourPlugin(const storage::StorageConfig storageConfig_): storageConfig(storageConfig_){
                        loadGraph(storageConfig.base_path.string(), old_graph);
                        util::Log() << "Start to reduce the graph";
                        TIMER_START(load);
                        //loadGraph(storageConfig.base_path.string(), graph);
                        loadTourGraph(old_graph, graph, 3);
                        TIMER_STOP(load);
                        util::Log() << "Load the Graph took: " << TIMER_MIN(load) << "min and " << (int)TIMER_SEC(load) % 60 << " sec";
                        util::Log() << "Number of Nodes: " << graph.GetNumberOfNodes();
                        util::Log() << "Number of Edges: " << graph.GetNumberOfEdges();
                    }

                    Status HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                                         const RoutingAlgorithmsInterface &algorithms,
                                         const api::TourParameters &tour_parameters,
                                         util::json::Object &json_result) const;
            };
        }
    }
}


#endif //OSRM_TOUR_HPP
