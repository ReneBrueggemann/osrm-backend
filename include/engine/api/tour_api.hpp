//
// Created by René Brüggemann on 20.04.17.
//

#ifndef ENGINE_API_TOUR_API_HPP
#define ENGINE_API_TOUR_API_HPP


#include "engine/api/route_api.hpp"
#include "engine/api/tour_parameters.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/internal_route_result.hpp"

#include "util/integer_range.hpp"

namespace osrm
{
    namespace engine
    {
        namespace api
        {

            class TourAPI : public RouteAPI
            {
            public:
                TourAPI(const datafacade::BaseDataFacade &facade_, const TourParameters &parameters_)
                        : RouteAPI(facade_, parameters_), parameters(parameters_)
                {
                }

                const TourParameters &parameters;
            };

        } // ns api
    } // ns engine
} // ns osrm

#endif //OSRM_TOUR_API_HPP
