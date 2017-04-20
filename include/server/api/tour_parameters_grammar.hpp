//
// Created by René Brüggemann on 20.04.17.
//

#ifndef TOUR_PARAMETERS_GRAMMAR_HPP
#define TOUR_PARAMETERS_GRAMMAR_HPP


#include "server/api/base_parameters_grammar.hpp"
#include "engine/api/tour_parameters.hpp"

#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

namespace osrm
{
    namespace server
    {
        namespace api
        {

            namespace
            {
                namespace ph = boost::phoenix;
                namespace qi = boost::spirit::qi;
            }

            template <typename Iterator = std::string::iterator,
                    typename Signature = void(engine::api::TourParameters &)>
            struct TourParametersGrammar : public BaseParametersGrammar<Iterator, Signature>
            {
                using BaseGrammar = BaseParametersGrammar<Iterator, Signature>;

                TourParametersGrammar() : TourParametersGrammar(root_rule)
                {
                    tour_rule =
                            (qi::lit("alternatives=") >
                             qi::bool_[ph::bind(&engine::api::TourParameters::alternatives, qi::_r1) = qi::_1]) |
                            (qi::lit("continue_straight=") >
                             (qi::lit("default") |
                              qi::bool_[ph::bind(&engine::api::TourParameters::continue_straight, qi::_r1) =
                                                qi::_1]));

                    root_rule = query_rule(qi::_r1) > -qi::lit(".json") >
                                -('?' > (tour_rule(qi::_r1) | base_rule(qi::_r1)) % '&');
                }

                TourParametersGrammar(qi::rule<Iterator, Signature> &root_rule_) : BaseGrammar(root_rule_)
                {
                    using AnnotationsType = engine::api::TourParameters::AnnotationsType;

                    const auto add_annotation = [](engine::api::TourParameters &tour_parameters,
                                                   AnnotationsType tour_param) {
                        tour_parameters.annotations_type = tour_parameters.annotations_type | tour_param;
                        tour_parameters.annotations =
                                tour_parameters.annotations_type != AnnotationsType::None;
                    };

                    geometries_type.add("geojson", engine::api::TourParameters::GeometriesType::GeoJSON)(
                            "polyline", engine::api::TourParameters::GeometriesType::Polyline)(
                            "polyline6", engine::api::TourParameters::GeometriesType::Polyline6);

                    overview_type.add("simplified", engine::api::TourParameters::OverviewType::Simplified)(
                            "full", engine::api::TourParameters::OverviewType::Full)(
                            "false", engine::api::TourParameters::OverviewType::False);

                    annotations_type.add("duration", AnnotationsType::Duration)("nodes",
                                                                                AnnotationsType::Nodes)(
                            "distance", AnnotationsType::Distance)("weight", AnnotationsType::Weight)(
                            "datasources", AnnotationsType::Datasources)("speed", AnnotationsType::Speed);

                    base_rule =
                            BaseGrammar::base_rule(qi::_r1) |
                            (qi::lit("steps=") >
                             qi::bool_[ph::bind(&engine::api::TourParameters::steps, qi::_r1) = qi::_1]) |
                            (qi::lit("geometries=") >
                             geometries_type[ph::bind(&engine::api::TourParameters::geometries, qi::_r1) =
                                                     qi::_1]) |
                            (qi::lit("overview=") >
                             overview_type[ph::bind(&engine::api::TourParameters::overview, qi::_r1) = qi::_1]) |
                            (qi::lit("annotations=") >
                             (qi::lit("true")[ph::bind(add_annotation, qi::_r1, AnnotationsType::All)] |
                              qi::lit("false")[ph::bind(add_annotation, qi::_r1, AnnotationsType::None)] |
                              (annotations_type[ph::bind(add_annotation, qi::_r1, qi::_1)] % ',')));

                    query_rule = BaseGrammar::query_rule(qi::_r1);
                }

            protected:
                qi::rule<Iterator, Signature> base_rule;
                qi::rule<Iterator, Signature> query_rule;

            private:
                qi::rule<Iterator, Signature> root_rule;
                qi::rule<Iterator, Signature> tour_rule;

                qi::symbols<char, engine::api::TourParameters::GeometriesType> geometries_type;
                qi::symbols<char, engine::api::TourParameters::OverviewType> overview_type;
                qi::symbols<char, engine::api::TourParameters::AnnotationsType> annotations_type;
            };
        }
    }
}

#endif //TOUR_PARAMETERS_GRAMMAR_HPP
