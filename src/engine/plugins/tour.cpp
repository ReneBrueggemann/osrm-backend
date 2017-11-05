//
// Created by René Brüggemann on 20.04.17.
//

#include "engine/plugins/tour.hpp"

#include "util/kml_writer.hpp"



#include <thread>
#include <stdlib.h>     /* srand, rand */
#include <time.h>

namespace osrm
{
    namespace engine
    {
        namespace plugins
        {

            void TourPlugin::dijkstra(NodeID start, const EdgeDistance length,
                                      Map<NodeID, NodeID> &parent, Map<NodeID, EdgeWeight> &weight,
                                      Map<NodeID, EdgeDistance> &distance) const {
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

                        EdgeDistance ways_length = distance[source] + data.distance;
                        EdgeWeight ways_weight = weight[source] + data.weight;

                        if(ways_length > length){
                            continue;
                        }

                        if(!queryHeap.WasInserted(target)) {
                            weight[target] = ways_weight;
                            distance[target] = ways_length;
                            parent[target] = source;
                            if (distance[target] < length) {
                                queryHeap.Insert(target, distance[target], source);
                            }
                        }else if(ways_length < distance[target]){
                            weight[target] = ways_weight;
                            distance[target] = ways_length;
                            parent[target] = source;
                            queryHeap.GetData(target).parent = source;
                            queryHeap.DecreaseKey(target, distance[target]);
                        }
                    }

                }
            }


            NodeID TourPlugin::SearchNearestNodeOfInterest(util::Coordinate coordinate) const{
                EdgeDistance min_distance = INVALID_EDGE_DISTANCE;
                NodeID nearest_node_id = SPECIAL_NODEID;
                for(NodeID id = 0; id < nodes_of_interest.size(); id++){
                    NodeID osrm_node_id = nodes_of_interest[id];
                    EdgeDistance distance = util::coordinate_calculation::haversineDistance(coordinate, coordinate_list[osrm_node_id]);
                    if(distance < min_distance){
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

                if (!CheckAllCoordinates(tour_parameters.coordinates))
                {
                    return Error("InvalidValue", "Invalid coordinate value.", json_result);
                }

                // Ermittel die naheliegendste NodeID
                //auto start = facade.GetNearestNodeIDs(tour_parameters.coordinates.back()).first;


                NodeID start = SearchNearestNodeOfInterest(tour_parameters.coordinates[0]);

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
                //Map<NodeID , NodeID> parents;
                //Map<NodeID , EdgeWeight> weights;
                //Map<NodeID , EdgeDistance> distances;
                Map<NodeID , NodeID> parents;
                Map<NodeID , EdgeWeight> weights;
                Map<NodeID, EdgeDistance> distances;
                EdgeDistance max_len = (EdgeDistance)((1 + epsilon) * length / 4);
                EdgeDistance min_len = (EdgeDistance)((1 - epsilon) * length / 4);

                TIMER_START(dijkstra);

                dijkstra(start, max_len, parents, weights, distances);

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
                    /*
                     * if(distance > min_len && distance < max_len && graph.GetOutDegree(node) > 3 ){
                     *  K.insert(node);
                     * }
                     */
                    if(distance > min_len && distance < max_len){
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
                Map<NodeID , Map<NodeID ,NodeID>>candidate;
                NodeID hp;

                std::set<NodeID> Kn;

                for(auto p : K){
                    hp = p;
                    while(distances[parents[hp]] > distances[p]*0.5){
                        hp = parents[hp];
                    }
                    H.insert(hp);
                    candidate[hp][p] = p;
                }


                bestMidwayPoint = SPECIAL_NODEID;
                bestWeight = INVALID_EDGE_WEIGHT;
                bestDistance = 0;

                TIMER_START(findmidwaypoints);
                /**
                 *  DESCRIPTION
                 *
                 *  foreach h ∈ H do
                 *      FindMidwayPoint(h, w'[h], c'[h], candidate[h])
                 */
                for(auto h : H){
                    //TIMER_START(findmidwaypoint);
                    SearchMidwayPoint(h, candidate[h]);
                    //TIMER_STOP(findmidwaypoint);

                }
                TIMER_STOP(findmidwaypoints);

                util::Log() << "FindMidwayPoint : " << TIMER_SEC(findmidwaypoints) << " seconds";

                if(bestMidwayPoint == SPECIAL_NODEID || bestWeight == INVALID_EDGE_WEIGHT){
                    return Error("NoRoute", "No route found between points", json_result);
                }

                api::TourParameters working_parameters = tour_parameters;


                for(NodeID node : P[bestMidwayPoint]){
                    if(node == *P[bestMidwayPoint].begin()){

                        auto osrm_p1 = nodes_of_interest[node];
                        auto osrm_m = nodes_of_interest[bestMidwayPoint];

                        working_parameters.coordinates.push_back(coordinate_list[osrm_p1]);
                        working_parameters.coordinates.push_back(coordinate_list[osrm_m]);
                        working_parameters.hints.push_back(Hint());
                        working_parameters.hints.push_back(Hint());
                    }else{

                        auto osrm_p2 = nodes_of_interest[node];


                        working_parameters.coordinates.push_back(coordinate_list[osrm_p2]);
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


            void TourPlugin::SearchMidwayPoint(NodeID start, Map<NodeID, NodeID> candidate) const{

                Map<NodeID , NodeID> parent;
                Map<NodeID , EdgeWeight> w;
                Map<NodeID , EdgeDistance> c;

                Map<NodeID , EdgeWeight> weights;

                TourEdgeData data;
                NodeID source, target;
                QueryHeap queryHeap(graph.GetNumberOfNodes());

                EdgeDistance haversineDistance_P1_P2;
                EdgeDistance ways_length = INVALID_EDGE_DISTANCE;
                EdgeWeight ways_weight = INVALID_EDGE_WEIGHT;

                EdgeDistance max_len = (EdgeDistance)((1 + epsilon) * length / 2);
                EdgeDistance min_len = (EdgeDistance)((1 - epsilon) * length / 2);


                w[start] = 0;
                c[start] = 0;

                queryHeap.Insert(start, w[start], start);

                while (!queryHeap.Empty()) {
                    source = queryHeap.DeleteMin();

                    for(const EdgeID edge : graph.GetAdjacentEdgeRange(source)){
                        data = (TourEdgeData &)graph.GetEdgeData(edge);
                        target = graph.GetTarget(edge);

                        if(target == SPECIAL_NODEID){
                            continue;
                        }

                        ways_length = c[source] + data.distance;
                        ways_weight = w[source] + data.weight;

                        if(!queryHeap.WasInserted(target)){
                            w[target] = ways_weight;
                            c[target] = ways_length;
                            parent[target] = source;
                            if(c[target] < max_len) {
                                queryHeap.Insert(target, w[target], source);
                            }
                        }else if(ways_weight < w[target]){
                            w[target] = ways_weight;
                            c[target] = ways_length;
                            parent[target] = source;
                            queryHeap.GetData(target).parent = source;
                            queryHeap.DecreaseKey(target, w[target]);
                        }
                    }

                    if(candidate.find(source) == candidate.end()) {
                        candidate[source] = candidate.find(parent[source]) == candidate.end() ? SPECIAL_NODEID
                                                                                              : candidate[parent[source]];
                    }


                    /*
                     * if(c[source] > min_len && c[source] < max_len
                     * && graph.GetOutDegree(source) > 3 && candidate[source] != SPECIAL_NODEID){
                     */

                    if(c[source] > min_len && c[source] < max_len && candidate[source] != SPECIAL_NODEID){
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


                            NodeID osrm_p1 = nodes_of_interest[p1];
                            NodeID osrm_p2 = nodes_of_interest[p2];

                            haversineDistance_P1_P2 = (EdgeDistance)util::coordinate_calculation::haversineDistance(coordinate_list[osrm_p1], coordinate_list[osrm_p2]);
                            if(haversineDistance_P1_P2 > bestDistance){
                                bestMidwayPoint = source;
                                bestDistance = haversineDistance_P1_P2;
                                bestWeight = weights[p1] + weights[p2];
                            }

                        }
                    }
                }
            }

            void TourPlugin::FindMidwayPoint(NodeID s, Map<NodeID, NodeID> candidate) const {

                Map<NodeID , NodeID> parent;
                Map<NodeID , EdgeWeight> w;
                Map<NodeID , EdgeDistance> c;

                Map<NodeID , EdgeWeight> weights;

                TourEdgeData data;
                NodeID source, target;
                QueryHeap queryHeap(graph.GetNumberOfNodes());

                EdgeDistance max_len = (EdgeDistance)((1 + epsilon) * length / 2);
                EdgeDistance min_len = (EdgeDistance)((1 - epsilon) * length / 2);

                EdgeDistance ways_length = INVALID_EDGE_DISTANCE;
                EdgeWeight ways_weight = INVALID_EDGE_WEIGHT;

                EdgeDistance haversineDistance_P1_P2;

                w[s] = 0;
                c[s] = 0;

                queryHeap.Insert(s, w[s], s);

                while (!queryHeap.Empty()) {
                    source = queryHeap.DeleteMin();

                    for (const EdgeID edge : graph.GetAdjacentEdgeRange(source)) {
                        data = (TourEdgeData &)graph.GetEdgeData(edge);
                        target = graph.GetTarget(edge);

                        if(target == SPECIAL_NODEID) {
                            continue;
                        }

                        ways_length = c[source] + data.distance;
                        ways_weight = w[source] + data.weight;

                        if(!queryHeap.WasInserted(target)){
                            w[target] = ways_weight;
                            c[target] = ways_length;
                            parent[target] = source;
                            if(c[target] < max_len) {
                                queryHeap.Insert(target, w[target], source);
                            }
                        }else if(ways_weight < w[target]){
                            w[target] = ways_weight;
                            c[target] = ways_length;
                            parent[target] = source;
                            queryHeap.GetData(target).parent = source;
                            queryHeap.DecreaseKey(target, w[target]);
                        }
                    }

                    if(candidate.find(source) == candidate.end()) {
                        candidate[source] = candidate.find(parent[source]) == candidate.end() ? SPECIAL_NODEID
                                                                                              : candidate[parent[source]];
                    }

                    /*
                     * if(c[source] > min_len && c[source] < max_len
                       && graph.GetOutDegree(source) > 3 && candidate[source] != SPECIAL_NODEID)
                    */
                    if(c[source] > min_len && c[source] < max_len && candidate[source] != SPECIAL_NODEID){
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

                            haversineDistance_P1_P2 = (EdgeDistance)util::coordinate_calculation::haversineDistance(coordinate_list[p1], coordinate_list[p2]);
                            if(haversineDistance_P1_P2 > bestDistance){
                                bestMidwayPoint = source;
                                bestDistance = haversineDistance_P1_P2;
                                bestWeight = weights[p1] + weights[p2];
                            }
                        }
                    }
                }
            }
        }
    }
}

