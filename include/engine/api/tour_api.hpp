//
// Created by René Brüggemann on 20.04.17.
//

#ifndef ENGINE_API_TOUR_API_HPP
#define ENGINE_API_TOUR_API_HPP


#include "engine/api/base_api.hpp"
#include "engine/api/json_factory.hpp"
#include "engine/api/tour_parameters.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/guidance/assemble_geometry.hpp"
#include "engine/guidance/assemble_leg.hpp"
#include "engine/guidance/assemble_overview.hpp"
#include "engine/guidance/assemble_route.hpp"
#include "engine/guidance/assemble_steps.hpp"
#include "engine/guidance/collapse_turns.hpp"
#include "engine/guidance/lane_processing.hpp"
#include "engine/guidance/post_processing.hpp"
#include "engine/guidance/verbosity_reduction.hpp"

#include "engine/internal_route_result.hpp"

#include "util/coordinate.hpp"
#include "util/integer_range.hpp"
#include "util/json_util.hpp"

#include <iterator>
#include <vector>

namespace osrm
{
    namespace engine
    {
        namespace api
        {

            class TourAPI : public BaseAPI
            {
            public:
                TourAPI(const datafacade::BaseDataFacade &facade_, const TourParameters &parameters_)
                        : BaseAPI(facade_, parameters_), parameters(parameters_)
                {
                }

                void MakeResponse(const InternalRouteResult &raw_tour, util::json::Object &response) const
                {
                    auto number_of_tour = raw_tour.has_alternative() ? 2UL : 1UL;
                    util::json::Array tours;
                    tours.values.resize(number_of_tour);
                    tours.values[0] = MakeRoute(raw_tour.segment_end_coordinates,
                                                 raw_tour.unpacked_path_segments,
                                                 raw_tour.source_traversed_in_reverse,
                                                 raw_tour.target_traversed_in_reverse);
                    if (raw_tour.has_alternative())
                    {
                        std::vector<std::vector<PathData>> wrapped_leg(1);
                        wrapped_leg.front() = std::move(raw_tour.unpacked_alternative);
                        tours.values[1] = MakeRoute(raw_tour.segment_end_coordinates,
                                                     wrapped_leg,
                                                     raw_tour.alt_source_traversed_in_reverse,
                                                     raw_tour.alt_target_traversed_in_reverse);
                    }
                    response.values["waypoints"] = BaseAPI::MakeWaypoints(raw_tour.segment_end_coordinates);
                    response.values["tours"] = std::move(tours);
                    response.values["code"] = "Ok";
                }

            protected:
                template <typename ForwardIter>
                util::json::Value MakeGeometry(ForwardIter begin, ForwardIter end) const
                {
                    if (parameters.geometries == TourParameters::GeometriesType::Polyline)
                    {
                        return json::makePolyline<100000>(begin, end);
                    }

                    if (parameters.geometries == TourParameters::GeometriesType::Polyline6)
                    {
                        return json::makePolyline<1000000>(begin, end);
                    }

                    BOOST_ASSERT(parameters.geometries == TourParameters::GeometriesType::GeoJSON);
                    return json::makeGeoJSONGeometry(begin, end);
                }

                template <typename GetFn>
                util::json::Array GetAnnotations(const guidance::LegGeometry &leg, GetFn Get) const
                {
                    util::json::Array annotations_store;
                    annotations_store.values.reserve(leg.annotations.size());
                    std::for_each(leg.annotations.begin(),
                                  leg.annotations.end(),
                                  [Get, &annotations_store](const auto &step) {
                                      annotations_store.values.push_back(Get(step));
                                  });
                    return annotations_store;
                }

                util::json::Object MakeRoute(const std::vector<PhantomNodes> &segment_end_coordinates,
                                             const std::vector<std::vector<PathData>> &unpacked_path_segments,
                                             const std::vector<bool> &source_traversed_in_reverse,
                                             const std::vector<bool> &target_traversed_in_reverse) const
                {
                    std::vector<guidance::RouteLeg> legs;
                    std::vector<guidance::LegGeometry> leg_geometries;
                    auto number_of_legs = segment_end_coordinates.size();
                    legs.reserve(number_of_legs);
                    leg_geometries.reserve(number_of_legs);

                    for (auto idx : util::irange<std::size_t>(0UL, number_of_legs))
                    {
                        const auto &phantoms = segment_end_coordinates[idx];
                        const auto &path_data = unpacked_path_segments[idx];

                        const bool reversed_source = source_traversed_in_reverse[idx];
                        const bool reversed_target = target_traversed_in_reverse[idx];

                        auto leg_geometry = guidance::assembleGeometry(BaseAPI::facade,
                                                                       path_data,
                                                                       phantoms.source_phantom,
                                                                       phantoms.target_phantom,
                                                                       reversed_source,
                                                                       reversed_target);
                        auto leg = guidance::assembleLeg(facade,
                                                         path_data,
                                                         leg_geometry,
                                                         phantoms.source_phantom,
                                                         phantoms.target_phantom,
                                                         reversed_target,
                                                         parameters.steps);

                        if (parameters.steps)
                        {
                            auto steps = guidance::assembleSteps(BaseAPI::facade,
                                                                 path_data,
                                                                 leg_geometry,
                                                                 phantoms.source_phantom,
                                                                 phantoms.target_phantom,
                                                                 reversed_source,
                                                                 reversed_target);

                            /* Perform step-based post-processing.
                             *
                             * Using post-processing on basis of route-steps for a single leg at a time
                             * comes at the cost that we cannot count the correct exit for roundabouts.
                             * We can only emit the exit nr/intersections up to/starting at a part of the leg.
                             * If a roundabout is not terminated in a leg, we will end up with a
                             *enter-roundabout
                             * and exit-roundabout-nr where the exit nr is out of sync with the previous enter.
                             *
                             *         | S |
                             *         *   *
                             *  ----*        * ----
                             *                  T
                             *  ----*        * ----
                             *       V *   *
                             *         |   |
                             *         |   |
                             *
                             * Coming from S via V to T, we end up with the legs S->V and V->T. V-T will say to
                             *take
                             * the second exit, even though counting from S it would be the third.
                             * For S, we only emit `roundabout` without an exit number, showing that we enter a
                             *roundabout
                             * to find a via point.
                             * The same exit will be emitted, though, if we should start routing at S, making
                             * the overall response consistent.
                             */

                            guidance::trimShortSegments(steps, leg_geometry);
                            leg.steps = guidance::collapseTurnInstructions(std::move(steps));
                            leg.steps = guidance::postProcess(std::move(leg.steps));
                            leg.steps = guidance::buildIntersections(std::move(leg.steps));
                            leg.steps = guidance::suppressShortNameSegments(std::move(leg.steps));
                            leg.steps = guidance::assignRelativeLocations(std::move(leg.steps),
                                                                          leg_geometry,
                                                                          phantoms.source_phantom,
                                                                          phantoms.target_phantom);
                            leg.steps = guidance::anticipateLaneChange(std::move(leg.steps));
                            leg.steps = guidance::collapseUseLane(std::move(leg.steps));
                            leg_geometry = guidance::resyncGeometry(std::move(leg_geometry), leg.steps);
                        }

                        leg_geometries.push_back(std::move(leg_geometry));
                        legs.push_back(std::move(leg));
                    }

                    auto tour = guidance::assembleRoute(legs);
                    boost::optional<util::json::Value> json_overview;
                    if (parameters.overview != TourParameters::OverviewType::False)
                    {
                        const auto use_simplification =
                                parameters.overview == TourParameters::OverviewType::Simplified;
                        BOOST_ASSERT(use_simplification ||
                                     parameters.overview == TourParameters::OverviewType::Full);

                        auto overview = guidance::assembleOverview(leg_geometries, use_simplification);
                        json_overview = MakeGeometry(overview.begin(), overview.end());
                    }

                    std::vector<util::json::Value> step_geometries;
                    for (const auto idx : util::irange<std::size_t>(0UL, legs.size()))
                    {
                        auto &leg_geometry = leg_geometries[idx];

                        step_geometries.reserve(step_geometries.size() + legs[idx].steps.size());

                        std::transform(
                                legs[idx].steps.begin(),
                                legs[idx].steps.end(),
                                std::back_inserter(step_geometries),
                                [this, &leg_geometry](const guidance::RouteStep &step) {
                                    if (parameters.geometries == TourParameters::GeometriesType::Polyline)
                                    {
                                        return static_cast<util::json::Value>(json::makePolyline<100000>(
                                                leg_geometry.locations.begin() + step.geometry_begin,
                                                leg_geometry.locations.begin() + step.geometry_end));
                                    }

                                    if (parameters.geometries == TourParameters::GeometriesType::Polyline6)
                                    {
                                        return static_cast<util::json::Value>(json::makePolyline<1000000>(
                                                leg_geometry.locations.begin() + step.geometry_begin,
                                                leg_geometry.locations.begin() + step.geometry_end));
                                    }

                                    BOOST_ASSERT(parameters.geometries == TourParameters::GeometriesType::GeoJSON);
                                    return static_cast<util::json::Value>(json::makeGeoJSONGeometry(
                                            leg_geometry.locations.begin() + step.geometry_begin,
                                            leg_geometry.locations.begin() + step.geometry_end));
                                });
                    }

                    std::vector<util::json::Object> annotations;

                    // To maintain support for uses of the old default constructors, we check
                    // if annotations property was set manually after default construction
                    auto requested_annotations = parameters.annotations_type;
                    if ((parameters.annotations == true) &&
                        (parameters.annotations_type == TourParameters::AnnotationsType::None))
                    {
                        requested_annotations = TourParameters::AnnotationsType::All;
                    }

                    if (requested_annotations != TourParameters::AnnotationsType::None)
                    {
                        for (const auto idx : util::irange<std::size_t>(0UL, leg_geometries.size()))
                        {
                            auto &leg_geometry = leg_geometries[idx];
                            util::json::Object annotation;

                            // AnnotationsType uses bit flags, & operator checks if a property is set
                            if (parameters.annotations_type & TourParameters::AnnotationsType::Speed)
                            {
                                annotation.values["speed"] = GetAnnotations(
                                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                                            auto val = std::round(anno.distance / anno.duration * 10.) / 10.;
                                            return util::json::clamp_float(val);
                                        });
                            }

                            if (requested_annotations & TourParameters::AnnotationsType::Duration)
                            {
                                annotation.values["duration"] = GetAnnotations(
                                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                                            return anno.duration;
                                        });
                            }
                            if (requested_annotations & TourParameters::AnnotationsType::Distance)
                            {
                                annotation.values["distance"] = GetAnnotations(
                                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                                            return anno.distance;
                                        });
                            }
                            if (requested_annotations & TourParameters::AnnotationsType::Weight)
                            {
                                annotation.values["weight"] = GetAnnotations(
                                        leg_geometry,
                                        [](const guidance::LegGeometry::Annotation &anno) { return anno.weight; });
                            }
                            if (requested_annotations & TourParameters::AnnotationsType::Datasources)
                            {
                                annotation.values["datasources"] = GetAnnotations(
                                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                                            return anno.datasource;
                                        });
                            }
                            if (requested_annotations & TourParameters::AnnotationsType::Nodes)
                            {
                                util::json::Array nodes;
                                nodes.values.reserve(leg_geometry.osm_node_ids.size());
                                std::for_each(leg_geometry.osm_node_ids.begin(),
                                              leg_geometry.osm_node_ids.end(),
                                              [this, &nodes](const OSMNodeID &node_id) {
                                                  nodes.values.push_back(static_cast<std::uint64_t>(node_id));
                                              });
                                annotation.values["nodes"] = std::move(nodes);
                            }

                            annotations.push_back(std::move(annotation));
                        }
                    }

                    auto result = json::makeRoute(tour,
                                                  json::makeRouteLegs(std::move(legs),
                                                                      std::move(step_geometries),
                                                                      std::move(annotations)),
                                                  std::move(json_overview),
                                                  facade.GetWeightName());

                    return result;
                }

                const TourParameters &parameters;
            };

        } // ns api
    } // ns engine
} // ns osrm

#endif //OSRM_TOUR_API_HPP
