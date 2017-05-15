//
// Created by René Brüggemann on 20.04.17.
//

#ifndef ENGINE_API_TOUR_PARAMETERS_HPP
#define ENGINE_API_TOUR_PARAMETERS_HPP

//#include "engine/api/base_parameters.hpp"
#include "engine/api/route_parameters.hpp"

#include <vector>


namespace osrm {
    namespace engine {
        namespace api {
            struct TourParameters : public RouteParameters {

                TourParameters() = default;

                template<typename... Args>
                TourParameters(const float length_,
                               const bool steps_,
                               const bool alternatives_,
                               const GeometriesType geometries_,
                               const OverviewType overview_,
                               const boost::optional<bool> continue_straight_,
                               Args... args_)
                        : RouteParameters{steps_, alternatives_, geometries_,
                                          overview_, continue_straight_,std::forward<Args>(args_)...}, length{length_}
                // Once we perfectly-forward `args` (see #2990) this constructor can delegate to the one below.
                {
                }

                // TourParameters constructor adding the `annotations` setting in a API-compatible way.
                template<typename... Args>
                TourParameters(const float length_,
                               const bool steps_,
                               const bool alternatives_,
                               const bool annotations_,
                               const GeometriesType geometries_,
                               const OverviewType overview_,
                               const boost::optional<bool> continue_straight_,
                               Args... args_)
                        :RouteParameters{steps_, alternatives_, annotations_, geometries_,
                                         overview_, continue_straight_,std::forward<Args>(args_)...}, length{length_}
                {
                }

                // enum based implementation of annotations parameter
                template<typename... Args>
                TourParameters(const float length_,
                               const bool steps_,
                               const bool alternatives_,
                               const AnnotationsType annotations_,
                               const GeometriesType geometries_,
                               const OverviewType overview_,
                               const boost::optional<bool> continue_straight_,
                               Args... args_)
                        :RouteParameters{steps_, alternatives_, annotations_, geometries_,
                                        overview_, continue_straight_, std::forward<Args>(args_)...}, length{length_}
                {
                }

            public:
                float length = 0.0f;

                bool IsValid() const { return coordinates.size() >= 2 && BaseParameters::IsValid(); }
            };
        }
    }
}


#endif //OSRM_TOUR_PARAMETERS_HPP
