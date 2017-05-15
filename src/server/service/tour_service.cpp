//
// Created by René Brüggemann on 20.04.17.
//
#include "server/service/tour_service.hpp"
#include "server/service/utils.hpp"

#include "server/api/parameters_parser.hpp"
#include "engine/api/tour_parameters.hpp"

#include "util/json_container.hpp"

namespace osrm
{
    namespace server
    {
        namespace service
        {
            namespace
            {
                std::string getWrongOptionHelp(const engine::api::TourParameters &parameters)
                {
                    std::string help;

                    const auto coord_size = parameters.coordinates.size();

                    const bool param_size_mismatch =
                            constrainParamSize(
                                    PARAMETER_SIZE_MISMATCH_MSG, "hints", parameters.hints, coord_size, help) ||
                            constrainParamSize(
                                    PARAMETER_SIZE_MISMATCH_MSG, "bearings", parameters.bearings, coord_size, help) ||
                            constrainParamSize(
                                    PARAMETER_SIZE_MISMATCH_MSG, "radiuses", parameters.radiuses, coord_size, help);

                    if (!param_size_mismatch && parameters.coordinates.size() < 2)
                    {
                        help = "Number of coordinates needs to be at least two.";
                    }

                    return help;
                }
            } // anon. ns

            engine::Status
            TourService::RunQuery(std::size_t prefix_length, std::string &query, ResultT &result)
            {
                result = util::json::Object();
                auto &json_result = result.get<util::json::Object>();

                auto query_iterator = query.begin();
                auto parameters = api::parseParameters<engine::api::TourParameters>(query_iterator, query.end());
                if (!parameters || query_iterator != query.end())
                {
                    const auto position = std::distance(query.begin(), query_iterator);
                    json_result.values["code"] = "InvalidQuery";
                    json_result.values["message"] =
                            "Query string malformed close to position " + std::to_string(prefix_length + position);
                    return engine::Status::Error;
                }
                BOOST_ASSERT(parameters);

                if (!parameters->IsValid())
                {
                    json_result.values["code"] = "InvalidOptions";
                    json_result.values["message"] = getWrongOptionHelp(*parameters);
                    return engine::Status::Error;
                }
                BOOST_ASSERT(parameters->IsValid());

                return BaseService::routing_machine.Tour(*parameters, json_result);
            }
        }
    }
}


