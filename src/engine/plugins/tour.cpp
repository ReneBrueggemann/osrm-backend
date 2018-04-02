//
// Created by René Brüggemann on 20.04.17.
//

#include "engine/plugins/tour.hpp"


#include "util/kml_writer.hpp"

#include <algorithm>    // std::random_shuffle
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include <thread>
#include <stdlib.h>     /* srand, rand */
#include <time.h>

namespace osrm
{
    namespace engine
    {
        namespace plugins {
            void TourPlugin::dijkstra(NodeID start, const EdgeDistance length,
                                           Map<NodeID, NodeID> &parent, Map<NodeID, EdgeWeight> &weight,
                                           Map<NodeID, EdgeDistance> &distance) const {
                TourEdgeData data;
                NodeID source, target;
                QueryHeap queryHeap(forward_graph.GetNumberOfNodes());

                // insert start node
                parent[start] = start;
                weight[start] = 0;
                distance[start] = 0;

                queryHeap.Insert(start, weight[start], parent[start]);

                while (!queryHeap.Empty()) {

                    source = queryHeap.DeleteMin();

                    for (const EdgeID edge : forward_graph.GetAdjacentEdgeRange(source)) {
                        data = (TourEdgeData &) forward_graph.GetEdgeData(edge);
                        target = forward_graph.GetTarget(edge);

                        if (target == SPECIAL_NODEID) {
                            continue;
                        }

                        EdgeDistance ways_length = distance[source] + data.distance;
                        EdgeWeight ways_weight = weight[source] + data.weight;

                        if (ways_length > length) {
                            continue;
                        }

                        if (!queryHeap.WasInserted(target)) {
                            weight[target] = ways_weight;
                            distance[target] = ways_length;
                            parent[target] = source;
                            if (distance[target] < length) {
                                queryHeap.Insert(target, weight[target], source);
                            }
                        } else if (ways_weight < weight[target]) {
                            weight[target] = ways_weight;
                            distance[target] = ways_length;
                            parent[target] = source;
                            queryHeap.GetData(target).parent = source;
                            queryHeap.DecreaseKey(target, weight[target]);
                        }
                    }
                }
            }


            NodeID TourPlugin::SearchNearestNodeOfInterest(util::Coordinate coordinate) const {
                EdgeDistance min_distance = INVALID_EDGE_DISTANCE;
                NodeID nearest_node_id = SPECIAL_NODEID;
                for (NodeID id = 0; id < nodes_of_interest.size(); id++) {
                    NodeID osrm_node_id = nodes_of_interest[id];
                    EdgeDistance distance = util::coordinate_calculation::haversineDistance(coordinate,
                                                                                            coordinate_list[osrm_node_id]);
                    if (distance < min_distance) {
                        nearest_node_id = id;
                        min_distance = distance;
                    }
                }
                return nearest_node_id;
            }


            Status
            TourPlugin::HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                                      const RoutingAlgorithmsInterface &algorithms,
                                      const api::TourParameters &tour_parameters,
                                      util::json::Object &json_result) const {
                BOOST_ASSERT(tour_parameters.IsValid());

                // ShortestPathSearch support
                if (!algorithms.HasShortestPathSearch()) {
                    return Error("NotImplemented",
                                 "Shortest path search is not implemented for the chosen search algorithm. "
                                         "Only two coordinates supported.",
                                 json_result);
                }

                // DirectShortestPathSearch support
                if (!algorithms.HasDirectShortestPathSearch()) {
                    return Error(
                            "NotImplemented",
                            "Direct shortest path search is not implemented for the chosen search algorithm.",
                            json_result);
                }

                if (!CheckAllCoordinates(tour_parameters.coordinates)) {
                    return Error("InvalidValue", "Invalid coordinate value.", json_result);
                }

                NodeID bestMidwayPoint;
                EdgeWeight bestWeight;
                EdgeDistance bestDistance;

                Map<NodeID, std::set<NodeID>> P;
                Map<NodeID, Map<NodeID, EdgeWeight>> weights;

                // Ermittel die naheliegendste NodeID
                //auto start = facade.GetNearestNodeIDs(tour_parameters.coordinates.back()).first;

                NodeID start = SearchNearestNodeOfInterest(tour_parameters.coordinates[0]);

                auto length = tour_parameters.length;
                auto epsilon = tour_parameters.epsilon;

                /**
                 *  DESCRIPTION
                 *
                 *  (w',c',parent') ←       dijkstra(start,(1+ε) * length/4);
                 *
                 *  RENAMES
                 *
                 *      w'          →       weights
                 *      c'          →       distances
                 *  parents'        →       parents
                 */
                Map<NodeID, NodeID> parents_dij;
                Map<NodeID, EdgeWeight> weights_dij;
                Map<NodeID, EdgeDistance> distances_dij;
                EdgeDistance max_len = (EdgeDistance) ((1 + epsilon) * length / 4);
                EdgeDistance min_len = (EdgeDistance) ((1 - epsilon) * length / 4);


                TIMER_START(dijkstra);

                dijkstra(start, max_len, parents_dij, weights_dij, distances_dij);

                TIMER_STOP(dijkstra);

                util::Log() << "Dijkstra : " << TIMER_MSEC(dijkstra) << " ms";

                Map<NodeID, NodeID> p;
                Map<NodeID, EdgeWeight> w;
                Map<NodeID, EdgeDistance> d;

                /**
                 *  DESCRIPTION
                 *
                 *  K   ←   {v ∈ V |c'[v] ∈ I(l/4, ε)};
                 */
                std::set<NodeID> K;
                EdgeDistance distance;
                NodeID node;

                for (auto instance : distances_dij) {
                    node = instance.first;
                    distance = instance.second;
                    if (distance > min_len && distance < max_len) {
                        K.insert(node);
                    }
                }

                /**
                 *  DESCRIPTION
                 *
                 *  foreach p ∈ K do
                 *      hp ← p;
                 *      while ⎟ c'[parent'[hp]] - c'[p] / 2 ⎟ < ⎟ c'[hp] - c'[p] / 2 ⎟ do
                 *          hp ← parent'[hp];
                 *      H ← H ∪ {hp};
                 *      candidate[hp][p] ← p;
                 */

                std::set<NodeID> H;
                Map<NodeID, Map<NodeID, NodeID>> candidate;
                NodeID hp;

                std::set<NodeID> Kn;

                for (auto p : K) {
                    hp = p;
                    while (distances_dij[parents_dij[hp]] > distances_dij[p] * 0.5) {
                        hp = parents_dij[hp];
                    }
                    H.insert(hp);
                    candidate[hp][p] = p;
                }

                util::Log() << "HalfWayPoints in " << max_len / 2000 << " km range: " << H.size() << " of " << K.size();

                bestMidwayPoint = SPECIAL_NODEID;
                bestWeight = INVALID_EDGE_WEIGHT;
                bestDistance = 0;

                P.clear();
                weights.clear();

                TIMER_START(Midway);
                /**
                 *  DESCRIPTION
                 *
                 *  foreach h ∈ H do
                 *      FindMidwayPoint(h, w'[h], c'[h], candidate[h]) -- Gewicht
                 *
                 *      SearchMidwayPoint(h, w'[h], c'[h], candidate[h]) -- Entfernung
                 */
                for (auto h : H) {
                    FindMidwayPoint(h, weights_dij[h], distances_dij[h], candidate[h], forward_graph,
                                      bestDistance, bestMidwayPoint, bestWeight, weights, length, epsilon,
                                      P, tour_parameters.alternatives);

                    /*
                    if(bestDistance > length / 10){
                        break;
                    }
                     */
                }

                TIMER_STOP(Midway);

                util::Log() << "Midway : " << TIMER_MSEC(Midway) << " ms";

                if (bestMidwayPoint == SPECIAL_NODEID || bestWeight == INVALID_EDGE_WEIGHT) {
                    return Error("NoRoute", "No route found between points", json_result);
                }

                RouteParameters routeParameters = RouteParameters(tour_parameters.steps,
                                                                  tour_parameters.alternatives,
                                                                  tour_parameters.annotations,
                                                                  tour_parameters.geometries,
                                                                  tour_parameters.overview,
                                                                  tour_parameters.continue_straight);


                routeParameters.coordinates.push_back(*tour_parameters.coordinates.begin());

                auto it = P[bestMidwayPoint].begin();
                auto osrm_p1 = nodes_of_interest[*it];
                auto osrm_m = nodes_of_interest[bestMidwayPoint];
                auto osrm_p2 = nodes_of_interest[*(++it)];

                routeParameters.coordinates.push_back(coordinate_list[osrm_p1]);
                routeParameters.coordinates.push_back(coordinate_list[osrm_m]);
                routeParameters.coordinates.push_back(coordinate_list[osrm_p2]);
                routeParameters.coordinates.push_back(*routeParameters.coordinates.begin());

                for (size_t i = 0; i < routeParameters.coordinates.size(); i++) {
                    routeParameters.hints.push_back(Hint());
                }

                return route_plugin.HandleRequest(facade, algorithms, routeParameters, json_result);
            }


            void TourPlugin::FindMidwayPoint(NodeID start, EdgeWeight w_h, EdgeDistance c_h,
                                               Map<NodeID, NodeID> candidate, TourGraph graph,
                                               EdgeDistance &bestDistance, NodeID &bestMidwayPoint,
                                               EdgeWeight &bestWeight, Map<NodeID, Map<NodeID, EdgeWeight>> &weights,
                                               EdgeDistance length, float epsilon,
                                               Map<NodeID, std::set<NodeID>> &P, bool original) const {

                Map<NodeID, NodeID> parent;
                Map<NodeID, EdgeWeight> w;
                Map<NodeID, EdgeDistance> c;

                TourEdgeData data;
                NodeID source, target;
                QueryHeap queryHeap(graph.GetNumberOfNodes());

                EdgeDistance haversineDistance_P1_P2;
                EdgeDistance ways_length = INVALID_EDGE_DISTANCE;
                EdgeWeight ways_weight = INVALID_EDGE_WEIGHT;

                EdgeDistance max_len = (EdgeDistance) ((1 + epsilon) * length / 2);
                EdgeDistance min_len = (EdgeDistance) ((1 - epsilon) * length / 2);


                /**
                 *  DESCRIPTION
                 *
                 *  (w'[s], c'[s]) ← (w_s,c_s)
                 */
                w[start] = w_h;
                c[start] = c_h;
                candidate[start] = SPECIAL_NODEID;
                /**
                 *  DESCRIPTION
                 *
                 *  Q ← {s}
                 */
                queryHeap.Insert(start, w[start], start);

                while (!queryHeap.Empty()) {
                    /**
                     *  DESCRIPTION
                     *
                     *  v ← argmin(w'[q]) mit q ∈ Q
                     *  Q ← Q\{v}
                     */
                    source = queryHeap.DeleteMin();

                    /**
                     *  DESCRIPTION
                     *
                     *  foreach e = {v,u} ∈ E do
                     *
                     *
                     */
                    for (const EdgeID edge : graph.GetAdjacentEdgeRange(source)) {
                        data = (TourEdgeData &) graph.GetEdgeData(edge);
                        target = graph.GetTarget(edge);

                        if (target == SPECIAL_NODEID) {
                            continue;
                        }

                        /**
                         *  DESCRIPTION
                         *
                         *  c'[u] = c[v] + c(e)
                         *  w'[u] = w[v] + w(e)
                         *
                         *
                         */
                        ways_length = c[source] + data.distance;
                        ways_weight = w[source] + data.weight;

                        if (ways_length > max_len) {
                            continue;
                        }

                        /**
                         *  DESCRIPTION
                         *
                         *  if c'[u] < (1+ε)l/2 then
                         *      Q ← Q ∪ {v}
                         */

                        if (!queryHeap.WasInserted(target)) {
                            w[target] = ways_weight;
                            c[target] = ways_length;
                            parent[target] = source;
                            if (c[target] < max_len) {
                                queryHeap.Insert(target, w[target], source);
                            }
                        } else if (ways_weight < w[target]) {
                            w[target] = ways_weight;
                            c[target] = ways_length;
                            parent[target] = source;
                            queryHeap.GetData(target).parent = source;
                            queryHeap.DecreaseKey(target, w[target]);
                        }
                    }

                    if (candidate.find(source) == candidate.end()) {
                        candidate[source] = candidate[parent[source]];
                    }

                    if (c[source] > min_len && c[source] < max_len && candidate[source] != SPECIAL_NODEID) {
                        P[source].insert(candidate[source]);
                        weights[source][candidate[source]] = w[source];

                        if (P[source].size() > 2) {
                            if(!original){
                                EdgeDistance best = 0;
                                NodeID b1, b2;
                                for(NodeID node1 : P[source]) {
                                    for (NodeID node2 : P[source]) {
                                        auto hav = util::coordinate_calculation::haversineDistance(
                                                coordinate_list[nodes_of_interest[node1]],
                                                coordinate_list[nodes_of_interest[node2]]);
                                        if(best < hav){
                                            best = hav;
                                            b1 = node1;
                                            b2 = node2;
                                        }
                                    }
                                }
                                P[source].clear();
                                P[source].insert(b1);
                                P[source].insert(b2);
                            }else{
                                EdgeWeight max_weight = 0;
                                NodeID to_delete = SPECIAL_NODEID;
                                for (NodeID node : P[source]) {
                                    if (max_weight < weights[source][node]) {
                                        max_weight = weights[source][node];
                                        to_delete = node;
                                    }
                                }
                                P[source].erase(to_delete);
                            }
                        }

                        if (P[source].size() == 2) {
                            NodeID p1 = SPECIAL_NODEID, p2 = SPECIAL_NODEID;

                            for (NodeID node : P[source]) {
                                if (p1 == SPECIAL_NODEID) {
                                    p1 = node;
                                } else {
                                    p2 = node;
                                }
                            }


                            NodeID osrm_p1 = nodes_of_interest[p1];
                            NodeID osrm_p2 = nodes_of_interest[p2];

                            haversineDistance_P1_P2 = (EdgeDistance) util::coordinate_calculation::haversineDistance(
                                    coordinate_list[osrm_p1], coordinate_list[osrm_p2]);
                            if (!original && haversineDistance_P1_P2 > bestDistance) {
                                bestMidwayPoint = source;
                                bestDistance = haversineDistance_P1_P2;
                                bestWeight = weights[source][p1] + weights[source][p2];
                            }else if (original && bestWeight > weights[source][p1] + weights[source][p2]) {
                                bestMidwayPoint = source;
                                bestDistance = haversineDistance_P1_P2;
                                bestWeight = weights[source][p1] + weights[source][p2];
                            }

                        }
                    }
                }
            }
        }
    }
}

