//
// Created by René Brüggemann on 20.04.17.
//

#ifndef TOUR_HPP
#define TOUR_HPP


#include "engine/plugins/plugin_base.hpp"
#include "engine/api/route_parameters.hpp"
#include "engine/plugins/viaroute.hpp"
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


            using RouteParameters = engine::api::RouteParameters;
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

                    TourGraph forward_graph;
                    TourGraph backward_graph;
                    TourGraph original_graph;
                    const ViaRoutePlugin route_plugin;

                void dijkstra(NodeID start, const EdgeDistance length,
                              Map<NodeID, NodeID > &parent,
                              Map<NodeID , std::int32_t> &weight,
                              Map<NodeID , EdgeDistance> &distance) const;

                void FindMidwayPoint(NodeID start, EdgeWeight w_h, EdgeDistance c_h ,
                                     Map<NodeID, NodeID> candidate, TourGraph graph,
                                     EdgeDistance &bestDistance, NodeID &bestMidwayPoint,
                                     EdgeWeight &bestWeight, Map<NodeID , Map<NodeID ,EdgeWeight>> &weights,
                                     EdgeDistance length, float epsilon, Map<NodeID , std::set<NodeID>> &P,
                                     bool original) const;

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

                void loadTourGraph(TourGraph &new_graph, TourGraph &graph){
                    for(NodeID i = 0; i < graph.GetNumberOfNodes(); i++){
                            nodes_of_interest.emplace_back(i);
                    }
                    new_graph = graph;
                }

                void loadTourGraph(TourGraph &new_graph, TourGraph &graph, const unsigned int degree)
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
                            if(step == 1) std::cout << (step-1) * 10 << "%\r";
                            else if(step == 10) std::cout << " - " << (step-1) * 10 << "%\r\n";
                            else std::cout << " - " << (step-1) * 10 << "%\r";
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

                void TourGraphReverse(TourGraph &new_graph, TourGraph &graph)
                {
                    std::vector<TourEdge> graph_edge_list;

                    for(auto node : nodes_of_interest){
                        for(auto edge : graph.GetAdjacentEdgeRange(node)) {
                            auto data = (TourEdgeData &)forward_graph.GetEdgeData(edge);
                            graph_edge_list.emplace_back(graph.GetTarget(edge), node, data.weight,
                                                         data.distance);
                        }
                    }
                    tbb::parallel_sort(graph_edge_list.begin(), graph_edge_list.end());
                    new_graph = TourGraph(graph.GetNumberOfNodes(), graph_edge_list);
                }

                public:
                    explicit TourPlugin(const storage::StorageConfig storageConfig_, int max_locations_viaroute_):
                            storageConfig(storageConfig_),route_plugin(max_locations_viaroute_){

                        std::vector<TourEdge> edge_list;
                        loadGraph(storageConfig.base_path.string(), original_graph);
                        util::Log() << "In OSRM";
                        util::Log() << "(original) Number of Nodes: " << original_graph.GetNumberOfNodes();
                        util::Log() << "(original) Number of Edges: " << original_graph.GetNumberOfEdges();

                        util::Log() << "Start to reduce the graph";

                        TIMER_START(core);
                        loadTourGraph(forward_graph, original_graph);
                        TIMER_STOP(core);

                        util::Log() << "Reduce took: " << TIMER_MIN(core) << "min and " << (int)TIMER_SEC(core) % 60 << " sec";
                        util::Log() << "(forward) Number of Nodes: " << forward_graph.GetNumberOfNodes();
                        util::Log() << "(forward) Number of Edges: " << forward_graph.GetNumberOfEdges();

                       // TourGraphReverse(backward_graph, forward_graph);

                        util::Log() << "Reverse done";

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
