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

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/function_output_iterator.hpp>

namespace osrm
{
    namespace engine
    {
        namespace plugins
        {

            struct TourEdgeData
            {
                TourEdgeData() : weight(INVALID_EDGE_WEIGHT), distance(INVALID_EDGE_DISTANCE), real(false) {}
                TourEdgeData(EdgeWeight weight_, bool real_) :
                        weight(weight_), distance(INVALID_EDGE_DISTANCE), real(real_) {}
                TourEdgeData(EdgeWeight weight_, EdgeDistance distance_, bool real_) :
                        weight(weight_), distance(distance_), real(real_) {}
                EdgeWeight weight;
                EdgeDistance distance;
                bool real;
            };

            using TourGraph = util::StaticGraph<TourEdgeData>;
            using TourEdge = TourGraph::InputEdge;
            using QueryHeap = util::
            BinaryHeap<NodeID, NodeID, EdgeWeight , HeapData, util::UnorderedMapStorage<NodeID, int>>;

            class TourPlugin final : public BasePlugin
            {

                private:
                    const storage::StorageConfig storageConfig;
                    std::vector<TourEdge> graph_edge_list;
                    std::vector<util::Coordinate> coordinate_list;
                    TourGraph graph;

                    mutable EdgeDistance length;
                    mutable float epsilon;
                    mutable NodeID bestMidwayPoint;
                    mutable EdgeWeight bestWeight;
                    mutable std::map<NodeID , std::set<NodeID>> P;


                void dijkstra(NodeID start, const EdgeDistance length,
                              std::map<NodeID, NodeID > &parent,
                              std::map<NodeID , std::int32_t> &weight,
                              std::map<NodeID , EdgeDistance> &distance) const;

                void FindMidwayPoint(NodeID s, EdgeWeight weight, EdgeDistance distance,
                                     std::map<NodeID ,NodeID> candidate) const;

                std::size_t loadGraph(const std::string &path)
                {
                    EdgeDistance distance = .0;
                    storage::io::FileReader file_reader(path, storage::io::FileReader::VerifyFingerprint);

                    // load graph data
                    //TODO: SHOULD BE PRIVATE
                    //std::vector<util::Coordinate> coordinate_list;
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
                                                         distance, true);
                        }

                        if (input_edge.backward)
                        {
                            graph_edge_list.emplace_back(input_edge.target, input_edge.source, input_edge.weight,
                                                         distance, true);
                        }
                    }

                    return number_of_nodes;
                }

                public:
                    explicit TourPlugin(const storage::StorageConfig storageConfig_): storageConfig(storageConfig_){
                        auto number_of_nodes = loadGraph(storageConfig.base_path.string());

                        tbb::parallel_sort(graph_edge_list.begin(), graph_edge_list.end());

                        graph = TourGraph(number_of_nodes, graph_edge_list);
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
