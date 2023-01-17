#pragma once

#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <vector>

#include "ppr/common/routing_graph.h"
#include "ppr/routing/route.h"
#include "ppr/routing/search_profile.h"
#include "ppr/routing/statistics.h"
#include "ppr/routing/input_pt.h"

namespace ppr::routing {

struct search_result {
  std::vector<std::vector<route>> routes_;
  routing_statistics stats_;

  int total_route_count() const {
    return std::accumulate(begin(routes_), end(routes_), 0,
                           [](auto const& sum, auto const& routes) {
                             return sum + static_cast<int>(routes.size());
                           });
  }

  int destinations_reached() const {
    return static_cast<int>(
        std::count_if(begin(routes_), end(routes_),
                      [](auto const& routes) { return !routes.empty(); }));
  }
};

enum class search_direction { FWD, BWD };

enum class osm_type {
  NODE, AREA, EDGE, UNKNOWN
};

search_result find_routes(routing_graph const& g, location const& start,
                          std::vector<location> const& destinations,
                          search_profile const& profile,
                          search_direction dir = search_direction::FWD,
                          bool allow_expansion = true);

search_result find_routes(routing_graph const& g, std::int64_t const& start_id,
                          osm_type const& type, location const& start_loc,
                          std::vector<std::int64_t> const& destination_ids,
                          std::vector<osm_type> const& destination_types,
                          std::vector<location> const& destination_locs,
                          search_profile const& profile,
                          search_direction dir = search_direction::FWD,
                          bool allow_expansion = true);

std::vector<input_pt> find_destinations(routing_graph const& g,
                                        std::vector<osm_type> const& end_type,
                                        std::vector<std::int64_t> const& destination_ids);

input_pt find_start(routing_graph const& g, std::int64_t const& start_id,
                    osm_type const& start_type);

}  // namespace ppr::routing
