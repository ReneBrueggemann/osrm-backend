//
// Created by René Brüggemann on 20.04.17.
//

#ifndef TOUR_HPP
#define TOUR_HPP


#include "engine/plugins/plugin_base.hpp"

#include "engine/api/tour_api.hpp"
#include "engine/datafacade/datafacade_base.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/search_engine_data.hpp"
#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
    namespace engine
    {
        namespace plugins
        {
            class TourPlugin final : public BasePlugin
            {
            private:
                const int max_locations_tour;

            public:
                explicit TourPlugin(int max_locations_tour_) : max_locations_tour(max_locations_tour_) {}

                Status HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                                     const RoutingAlgorithmsInterface &algorithms,
                                     const api::TourParameters &tour_parameters,
                                     util::json::Object &json_result) const;
            };
        }
    }
}


#endif //OSRM_TOUR_HPP
