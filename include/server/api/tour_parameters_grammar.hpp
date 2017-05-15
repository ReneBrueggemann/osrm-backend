//
// Created by René Brüggemann on 20.04.17.
//

#ifndef TOUR_PARAMETERS_GRAMMAR_HPP
#define TOUR_PARAMETERS_GRAMMAR_HPP


#include "server/api/route_parameters_grammar.hpp"
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
            struct TourParametersGrammar : public RouteParametersGrammar<Iterator, Signature>
            {
                using BaseGrammar = RouteParametersGrammar<Iterator, Signature>;

                TourParametersGrammar() : BaseGrammar(root_rule)
                {
                    tour_rule = (qi::lit("length=") >
                             qi::float_[ph::bind(&engine::api::TourParameters::length, qi::_r1) = qi::_1]
                    | (qi::lit("alternatives=") >
                       qi::bool_[ph::bind(&engine::api::RouteParameters::alternatives, qi::_r1) = qi::_1]));

                    root_rule = BaseGrammar::query_rule(qi::_r1) > -qi::lit(".json") >
                    -('?' > (tour_rule(qi::_r1) | BaseGrammar::base_rule(qi::_r1) ) %
                            '&');
                }

            private:
                qi::rule<Iterator, Signature> root_rule;
                qi::rule<Iterator, Signature> tour_rule;
            };
        }
    }
}

#endif //TOUR_PARAMETERS_GRAMMAR_HPP
