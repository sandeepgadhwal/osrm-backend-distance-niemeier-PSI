#ifndef ENGINE_API_TABLE_HPP
#define ENGINE_API_TABLE_HPP

#include "engine/api/base_api.hpp"
#include "engine/api/json_factory.hpp"
#include "engine/api/table_parameters.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/guidance/assemble_geometry.hpp"
#include "engine/guidance/assemble_leg.hpp"
#include "engine/guidance/assemble_overview.hpp"
#include "engine/guidance/assemble_route.hpp"
#include "engine/guidance/assemble_steps.hpp"

#include "engine/internal_route_result.hpp"

#include "util/integer_range.hpp"

#include <boost/range/algorithm/transform.hpp>

#include <iterator>

namespace osrm
{
namespace engine
{
namespace api
{
    
inline util::json::Value duration_encoder(const std::pair<EdgeWeight,DistanceData> & entry) 
{
    if (entry.first == INVALID_EDGE_WEIGHT) return util::json::Value(util::json::Null());
    return util::json::Value(util::json::Number(entry.first / 10.));
}

inline util::json::Value distance_encoder(const std::pair<EdgeWeight,DistanceData> & entry) 
{
    if (entry.second == INVALID_DISTANCE_DATA) return util::json::Value(util::json::Null());
    return util::json::Value(util::json::Number(entry.second));
}

class TableAPI final : public BaseAPI
{
  public:
    TableAPI(const datafacade::BaseDataFacade &facade_, const TableParameters &parameters_)
        : BaseAPI(facade_, parameters_), parameters(parameters_)
    {
    }

    virtual void MakeResponse(const std::vector<std::pair<EdgeWeight,DistanceData>> &entries,
                              const std::vector<PhantomNode> &phantoms,
                              util::json::Object &response,
                              const std::vector<TableOutputComponent> & output_components
                              ) const
    {
        auto number_of_sources = parameters.sources.size();
        auto number_of_destinations = parameters.destinations.size();
        ;

        // symmetric case
        if (parameters.sources.empty())
        {
            response.values["sources"] = MakeWaypoints(phantoms);
            number_of_sources = phantoms.size();
        }
        else
        {
            response.values["sources"] = MakeWaypoints(phantoms, parameters.sources);
        }

        if (parameters.destinations.empty())
        {
            response.values["destinations"] = MakeWaypoints(phantoms);
            number_of_destinations = phantoms.size();
        }
        else
        {
            response.values["destinations"] = MakeWaypoints(phantoms, parameters.destinations);
        }
        
        bool durationsRequested = output_components.size() == 0 || std::find(output_components.begin(), output_components.end(), DURATION) != output_components.end();
        bool distancesRequested = output_components.size() > 0 && std::find(output_components.begin(), output_components.end(), DISTANCE) != output_components.end();

        if(durationsRequested)
        {
            response.values["durations"] =
                MakeTable(entries, number_of_sources, number_of_destinations, duration_encoder);
        }
        if(distancesRequested)
        {
            response.values["distances"] =
                MakeTable(entries, number_of_sources, number_of_destinations, distance_encoder);
        }
        response.values["code"] = "Ok";
    }

    // FIXME gcc 4.8 doesn't support for lambdas to call protected member functions
    //  protected:
    virtual util::json::Array MakeWaypoints(const std::vector<PhantomNode> &phantoms) const
    {
        util::json::Array json_waypoints;
        json_waypoints.values.reserve(phantoms.size());
        BOOST_ASSERT(phantoms.size() == parameters.coordinates.size());

        boost::range::transform(
            phantoms,
            std::back_inserter(json_waypoints.values),
            [this](const PhantomNode &phantom) { return BaseAPI::MakeWaypoint(phantom); });
        return json_waypoints;
    }

    virtual util::json::Array MakeWaypoints(const std::vector<PhantomNode> &phantoms,
                                            const std::vector<std::size_t> &indices) const
    {
        util::json::Array json_waypoints;
        json_waypoints.values.reserve(indices.size());
        boost::range::transform(indices,
                                std::back_inserter(json_waypoints.values),
                                [this, phantoms](const std::size_t idx) {
                                    BOOST_ASSERT(idx < phantoms.size());
                                    return BaseAPI::MakeWaypoint(phantoms[idx]);
                                });
        return json_waypoints;
    }

    template<typename JSON_VALUE_ENCODER> 
    util::json::Array MakeTable(const std::vector<std::pair<EdgeWeight,DistanceData>> &values,
                                        std::size_t number_of_rows,
                                        std::size_t number_of_columns,
                                        JSON_VALUE_ENCODER encoder
                                      ) const
    {
        util::json::Array json_table;
        for (const auto row : util::irange<std::size_t>(0UL, number_of_rows))
        {
            util::json::Array json_row;
            auto row_begin_iterator = values.begin() + (row * number_of_columns);
            auto row_end_iterator = values.begin() + ((row + 1) * number_of_columns);
            json_row.values.resize(number_of_columns);
            std::transform(row_begin_iterator,
                           row_end_iterator,
                           json_row.values.begin(),
                           encoder);
            json_table.values.push_back(std::move(json_row));
        }
        return json_table;
    }

    const TableParameters &parameters;
};

} // ns api
} // ns engine
} // ns osrm

#endif
