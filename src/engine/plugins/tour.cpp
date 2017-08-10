//
// Created by René Brüggemann on 20.04.17.
//

#include "engine/plugins/tour.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/status.hpp"
#include "engine/algorithm.hpp"
#include "engine/engine_config.hpp"

#include "util/for_each_pair.hpp"
#include "util/integer_range.hpp"
#include "util/json_container.hpp"
#include "util/log.hpp"
#include "util/coordinate.hpp"
#include "util/kml_writer.hpp"
#include "util/timing_util.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <ctime>

namespace osrm
{
    namespace engine
    {
        namespace plugins
        {

            void TourPlugin::dijkstra(NodeID start, const EdgeDistance length,
                                      std::map<NodeID, NodeID> &parent, std::map<NodeID, std::int32_t> &weight,
                                      std::map<NodeID, EdgeDistance> &distance) const {
                TourEdgeData data;
                NodeID source, target;
                QueryHeap queryHeap(graph.GetNumberOfNodes());

                // insert start node
                parent[start] = start;
                weight[start] = 0;
                distance[start] = 0;

                queryHeap.Insert(start, distance[start], parent[start]);

                while (!queryHeap.Empty()){

                    source = queryHeap.DeleteMin();

                    for (const EdgeID edge : graph.GetAdjacentEdgeRange(source)) {
                        data = (TourEdgeData &)graph.GetEdgeData(edge);
                        target = graph.GetTarget(edge);

                        if(target == SPECIAL_NODEID) {
                            continue;
                        }

                        EdgeDistance ways_lenght = distance[source] + data.distance;
                        EdgeWeight ways_weight = weight[source] + data.weight;

                        if(ways_lenght > length){
                            continue;
                        }

                        if(!queryHeap.WasInserted(target)){
                            weight[target] = ways_weight;
                            distance[target] = ways_lenght;
                            parent[target] = source;
                            queryHeap.Insert(target, distance[target], source);
                        }else if(ways_lenght < distance[target]){
                            weight[target] = ways_weight;
                            distance[target] = ways_lenght;
                            parent[target] = source;
                            queryHeap.GetData(target).parent = source;
                            queryHeap.DecreaseKey(target, distance[target]);
                        }
                    }

                }
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

                if (!CheckAllCoordinates(tour_parameters.coordinates))
                {
                    return Error("InvalidValue", "Invalid coordinate value.", json_result);
                }

                // Ermittel die naheliegendste NodeID
                auto start = facade.GetNearestNodeIDs(tour_parameters.coordinates.back());

                length = tour_parameters.length;
                epsilon = tour_parameters.epsilon;

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
                std::map<NodeID , NodeID> parents;
                std::map<NodeID , EdgeWeight> weights;
                std::map<NodeID , EdgeDistance> distances;

                EdgeDistance max_len = (EdgeDistance)((1 + epsilon) * length / 4);
                EdgeDistance min_len = (EdgeDistance)((1 - epsilon) * length / 4);

                TIMER_START(dijkstra);

                dijkstra(start.first, max_len, parents, weights, distances);

                TIMER_STOP(dijkstra);

                util::Log() << "Dijkstra : " << TIMER_MSEC(dijkstra) << " ms";

                /**
                 *  DESCRIPTION
                 *
                 *  K   ←   {v ∈ V |c'[v] ∈ I(l/4, ε)};
                 */
                std::set<NodeID> K;
                EdgeDistance distance;
                NodeID node;

                for(auto instance : distances){
                    node = instance.first;
                    distance = instance.second;
                    if(distance > min_len && distance < max_len && graph.GetOutDegree(node) > 3 ){
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
                std::map<NodeID , std::map<NodeID ,NodeID>>candidate;
                NodeID hp;

                std::set<NodeID> Kn;

                int i = 0;
                for(auto p : K){
                    hp = p;
                    while(distances[parents[hp]] > distances[p]*0.75){
                        hp = parents[hp];
                    }
                    H.insert(hp);
                    candidate[hp][p] = p;
                    i++;
                }

                util::Log() << "K.size() : " << i;

                bestMidwayPoint = SPECIAL_NODEID;
                bestWeight = INVALID_EDGE_WEIGHT;

                int j = 0;
                TIMER_START(findmidwaypoint);
                /**
                 *  DESCRIPTION
                 *
                 *  foreach h ∈ H do
                 *      FindMidwayPoint(h, w'[h], c'[h], candidate[h])
                 */
                for(auto h : H){
                    FindMidwayPoint(h, weights[h], distances[h], candidate[h]);
                    j++;
                }
                TIMER_STOP(findmidwaypoint);


                util::Log() << "H.size() : " << j;

                util::Log() << "FindMidwayPoint : " << TIMER_SEC(findmidwaypoint) << " seconds";

                if(bestMidwayPoint == SPECIAL_NODEID || bestWeight == INVALID_EDGE_WEIGHT){
                    return Error("NoRoute", "No route found between points", json_result);
                }

                util::Log(logDEBUG) << "start: " << coordinate_list[start.first];

                util::Log(logDEBUG) << "mid: " << coordinate_list[bestMidwayPoint];

                for(NodeID node : P[bestMidwayPoint]){
                    util::Log(logDEBUG) << "p: " << coordinate_list[node];
                }


                /**
                 *
                 */

                api::TourParameters working_parameters = tour_parameters;
                for(NodeID node : P[bestMidwayPoint]){
                    if(node == *P[bestMidwayPoint].begin()){
                        working_parameters.coordinates.push_back(coordinate_list[node]);
                        working_parameters.coordinates.push_back(coordinate_list[bestMidwayPoint]);
                        working_parameters.hints.push_back(Hint());
                        working_parameters.hints.push_back(Hint());
                    }else{
                        working_parameters.coordinates.push_back(coordinate_list[node]);
                        working_parameters.coordinates.push_back(*working_parameters.coordinates.begin());
                        working_parameters.hints.push_back(Hint());
                        working_parameters.hints.push_back(Hint());
                    }
                }



                auto phantom_node_pairs = GetPhantomNodes(facade, working_parameters);
                if (phantom_node_pairs.size() != working_parameters.coordinates.size())
                {
                    return Error("NoSegment",
                                 std::string("Could not find a matching segment for coordinate ") +
                                 std::to_string(phantom_node_pairs.size()),
                                 json_result);
                }

                BOOST_ASSERT(phantom_node_pairs.size() == working_parameters.coordinates.size());

                const bool continue_straight_at_waypoint =  working_parameters.continue_straight
                                                            ? *working_parameters.continue_straight
                                                            : facade.GetContinueStraightDefault();


                auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);
                if (snapped_phantoms.size() != working_parameters.coordinates.size())
                {
                    return Error("NoSegment",
                                 std::string("Could not find a matching segment for coordinate ") +
                                 std::to_string(phantom_node_pairs.size()),
                                 json_result);
                }

                BOOST_ASSERT(snapped_phantoms.size() == working_parameters.coordinates.size());


                std::vector<PhantomNodes> start_end_nodes;
                auto build_phantom_pairs = [&start_end_nodes, continue_straight_at_waypoint](
                        const PhantomNode &first_node, const PhantomNode &second_node) {
                    start_end_nodes.push_back(PhantomNodes{first_node, second_node});
                    auto &last_inserted = start_end_nodes.back();
                    // enable forward direction if possible
                    if (last_inserted.source_phantom.forward_segment_id.id != SPECIAL_SEGMENTID)
                    {
                        last_inserted.source_phantom.forward_segment_id.enabled |=
                                !continue_straight_at_waypoint;
                    }
                    // enable reverse direction if possible
                    if (last_inserted.source_phantom.reverse_segment_id.id != SPECIAL_SEGMENTID)
                    {
                        last_inserted.source_phantom.reverse_segment_id.enabled |=
                                !continue_straight_at_waypoint;
                    }
                };
                util::for_each_pair(snapped_phantoms, build_phantom_pairs);


                InternalRouteResult raw_route;
                raw_route = algorithms.ShortestPathSearch(start_end_nodes, continue_straight_at_waypoint);


                if (raw_route.is_valid())
                {
                    api::TourAPI tour_api{facade, working_parameters};
                    tour_api.MakeResponse(raw_route, json_result);
                }else{
                    return Error("NoRoute", "No route found between points", json_result);
                }

                return Status::Ok;
            }

            void TourPlugin::FindMidwayPoint(NodeID s, EdgeWeight weight, EdgeDistance distance,
                                             std::map<NodeID, NodeID> candidate) const {

                std::map<NodeID , NodeID> parent;
                std::map<NodeID , EdgeWeight> w;
                std::map<NodeID , EdgeDistance> c;

                std::map<NodeID , EdgeWeight> weights;

                TourEdgeData data;
                NodeID source, target;
                QueryHeap queryHeap(graph.GetNumberOfNodes());

                EdgeDistance max_len = (EdgeDistance)((1 + epsilon) * length / 4);
                EdgeDistance min_len = (EdgeDistance)((1 - epsilon) * length / 4);

                w[s] = weight;
                c[s] = distance;

                queryHeap.Insert(s, c[s], s);

                while (!queryHeap.Empty()) {
                    source = queryHeap.DeleteMin();

                    for (const EdgeID edge : graph.GetAdjacentEdgeRange(source)) {
                        data = (TourEdgeData &)graph.GetEdgeData(edge);
                        target = graph.GetTarget(edge);

                        if(target == SPECIAL_NODEID) {
                            continue;
                        }

                        EdgeDistance ways_lenght = c[source] + data.distance;
                        EdgeWeight ways_weight = w[source] + data.weight;

                        if(ways_lenght > max_len){
                            continue;
                        }

                        if(!queryHeap.WasInserted(target)){
                            w[target] = ways_weight;
                            c[target] = ways_lenght;
                            parent[target] = source;
                            queryHeap.Insert(target, c[target], source);
                        }else if(ways_lenght < c[target]){
                            w[target] = ways_weight;
                            c[target] = ways_lenght;
                            parent[target] = source;
                            queryHeap.GetData(target).parent = source;
                            queryHeap.DecreaseKey(target, c[target]);
                        }
                    }


                    if(candidate.find(source) == candidate.end()){
                        candidate[source] = candidate[parent[source]];
                    }


                    if(c[source] > min_len && c[source] < max_len && candidate.find(source) != candidate.end()){
                        P[source].insert(candidate[source]);

                        weights[candidate[source]] = w[source];

                        if(P[source].size() > 2){
                            EdgeWeight max_weight = 0;
                            NodeID to_delete = SPECIAL_NODEID;
                            for(NodeID node : P[source]){
                                if(max_weight < weights[node]){
                                    max_weight = weights[node];
                                    to_delete = node;
                                }
                            }
                            P[source].erase(to_delete);
                        }

                        if(P[source].size() == 2){
                            NodeID p1 = SPECIAL_NODEID, p2 = SPECIAL_NODEID;

                            for(NodeID node : P[source]){
                                if(p1 == SPECIAL_NODEID){
                                    p1 = node;
                                }else{
                                    p2 = node;
                                }
                            }

                            if(weights[p1] + weights[p2] < bestWeight){
                                bestMidwayPoint = source;
                                bestWeight = weights[p1] + weights[p2];
                            }
                        }
                    }

                }
            }
        }
    }
}

