//
// Created by René Brüggemann on 20.04.17.
//

#ifndef ENGINE_API_TOUR_PARAMETERS_HPP
#define ENGINE_API_TOUR_PARAMETERS_HPP

#include "engine/api/base_parameters.hpp"

#include <vector>


namespace osrm {
    namespace engine {
        namespace api {
            struct TourParameters : public BaseParameters {
                enum class GeometriesType {
                    Polyline,
                    Polyline6,
                    GeoJSON
                };
                enum class OverviewType {
                    Simplified,
                    Full,
                    False
                };
                enum class AnnotationsType {
                    None = 0,
                    Duration = 0x01,
                    Nodes = 0x02,
                    Distance = 0x04,
                    Weight = 0x08,
                    Datasources = 0x10,
                    Speed = 0x20,
                    All = Duration | Nodes | Distance | Weight | Datasources | Speed
                };

                TourParameters() = default;

                template<typename... Args>
                TourParameters(const bool steps_,
                               const bool alternatives_,
                               const GeometriesType geometries_,
                               const OverviewType overview_,
                               const boost::optional<bool> continue_straight_,
                               Args... args_)
                        : BaseParameters{std::forward<Args>(args_)...}, steps{steps_}, alternatives{alternatives_},
                          annotations{false}, annotations_type{AnnotationsType::None}, geometries{geometries_},
                          overview{overview_}, continue_straight{continue_straight_}
                // Once we perfectly-forward `args` (see #2990) this constructor can delegate to the one below.
                {
                }

                // TourParameters constructor adding the `annotations` setting in a API-compatible way.
                template<typename... Args>
                TourParameters(const bool steps_,
                               const bool alternatives_,
                               const bool annotations_,
                               const GeometriesType geometries_,
                               const OverviewType overview_,
                               const boost::optional<bool> continue_straight_,
                               Args... args_)
                        : BaseParameters{std::forward<Args>(args_)...}, steps{steps_}, alternatives{alternatives_},
                          annotations{annotations_},
                          annotations_type{annotations_ ? AnnotationsType::All : AnnotationsType::None},
                          geometries{geometries_}, overview{overview_}, continue_straight{continue_straight_} {
                }

                // enum based implementation of annotations parameter
                template<typename... Args>
                TourParameters(const bool steps_,
                               const bool alternatives_,
                               const AnnotationsType annotations_,
                               const GeometriesType geometries_,
                               const OverviewType overview_,
                               const boost::optional<bool> continue_straight_,
                               Args... args_)
                        : BaseParameters{std::forward<Args>(args_)...}, steps{steps_}, alternatives{alternatives_},
                          annotations{annotations_ == AnnotationsType::None ? false : true},
                          annotations_type{annotations_}, geometries{geometries_}, overview{overview_},
                          continue_straight{continue_straight_} {
                }

                bool steps = false;
                bool alternatives = false;
                bool annotations = false;
                AnnotationsType annotations_type = AnnotationsType::None;
                GeometriesType geometries = GeometriesType::Polyline;
                OverviewType overview = OverviewType::Simplified;
                boost::optional<bool> continue_straight;

                bool IsValid() const { return coordinates.size() >= 2 && BaseParameters::IsValid(); }
            };

            inline bool operator&(TourParameters::AnnotationsType lhs, TourParameters::AnnotationsType rhs) {
                return static_cast<bool>(
                        static_cast<std::underlying_type_t<TourParameters::AnnotationsType>>(lhs) &
                        static_cast<std::underlying_type_t<TourParameters::AnnotationsType>>(rhs));
            }

            inline TourParameters::AnnotationsType operator|(TourParameters::AnnotationsType lhs,
                                                             TourParameters::AnnotationsType rhs) {
                return (TourParameters::AnnotationsType) (
                        static_cast<std::underlying_type_t<TourParameters::AnnotationsType>>(lhs) |
                        static_cast<std::underlying_type_t<TourParameters::AnnotationsType>>(rhs));
            }

            inline TourParameters::AnnotationsType operator|=(TourParameters::AnnotationsType lhs,
                                                              TourParameters::AnnotationsType rhs) {
                return lhs = lhs | rhs;
            }
        }
    }
}


#endif //OSRM_TOUR_PARAMETERS_HPP
