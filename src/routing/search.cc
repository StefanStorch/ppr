#include <algorithm>
#include <queue>

#include "ppr/common/timing.h"
#include "ppr/routing/label.h"
#include "ppr/routing/labels_to_route.h"
#include "ppr/routing/pareto_dijkstra.h"
#include "ppr/routing/postprocessing.h"
#include "ppr/routing/search.h"

namespace ppr::routing {

using mapped_pt = std::pair<location, std::vector<input_pt>>;

search_result find_routes(search_result& result, mapped_pt const& start,
                          std::vector<mapped_pt> const& destinations,
                          search_profile const& profile, search_direction dir) {
  auto const t_start = timing_now();
  pareto_dijkstra<label> pd(profile, dir == search_direction::BWD);

  pd.add_start(start.first, start.second);

  for (auto const& goal : destinations) {
    pd.add_goal(goal.first, goal.second);
  }

  pd.search();

  auto const t_after_search = timing_now();
  auto results = pd.get_results();
  auto& routes = result.routes_;
  routes.resize(destinations.size());
  assert(results.size() == destinations.size());
  for (auto i = 0UL; i < results.size(); i++) {
    assert(i < routes.size());
    if (!routes[i].empty()) {
      continue;
    }
    auto const& goal_results = results[i];
    std::transform(begin(goal_results), end(goal_results),
                   std::back_inserter(routes[i]),
                   [&](auto& label) { return labels_to_route(label); });
  }

  result.stats_.attempts_++;
  result.stats_.dijkstra_statistics_.push_back(pd.get_statistics());
  auto& stats = result.stats_.dijkstra_statistics_.back();
  stats.d_labels_to_route_ = ms_since(t_after_search);
  stats.d_total_ = ms_since(t_start);

  return result;
}

bool all_goals_reached(search_result const& result) {
  return std::all_of(
      begin(result.routes_), end(result.routes_),
      [](std::vector<route> const& routes) { return !routes.empty(); });
}

constexpr unsigned initial_max_pt_query = 10;
constexpr unsigned initial_max_pt_count = 1;
constexpr double initial_max_pt_dist = 200;

constexpr unsigned expanded_max_pt_query = 40;
constexpr unsigned expanded_max_pt_count = 20;
constexpr double expanded_max_pt_dist = 300;

search_result find_routes(routing_graph const& g, location const& start,
                          std::vector<location> const& destinations,
                          search_profile const& profile, search_direction dir,
                          bool allow_expansion) {
  return find_routes(g, start, destinations, profile, dir, allow_expansion, nullptr);
}

search_result find_routes(routing_graph const& g, location const& start,
                          std::vector<location> const& destinations,
                          search_profile const& profile, search_direction dir,
                          bool allow_expansion,
                          std::chrono::time_point<std::chrono::steady_clock>* start_t) {
  search_result result;
  std::chrono::time_point<std::chrono::steady_clock> t_start;
  if(start_t == nullptr) {
    t_start = timing_now();
  } else {
    t_start = *start_t;
  }
  mapped_pt mapped_start = {
      start, nearest_points(g, start, initial_max_pt_query,
                            initial_max_pt_count, initial_max_pt_dist)};
  auto const t_after_start = timing_now();
  result.stats_.d_start_pts_ = ms_between(t_start, t_after_start);

  std::vector<mapped_pt> mapped_goals;
  mapped_goals.reserve(destinations.size());
  std::transform(
      begin(destinations), end(destinations), std::back_inserter(mapped_goals),
      [&](location const& loc) {
        return mapped_pt{
            loc, nearest_points(g, loc, initial_max_pt_query,
                                initial_max_pt_count, initial_max_pt_dist)};
      });
  auto const t_after_dest = timing_now();
  result.stats_.d_destination_pts_ = ms_between(t_after_start, t_after_dest);

  // 1st attempt: only nearest start + goal points
  find_routes(result, mapped_start, mapped_goals, profile, dir);

  if (allow_expansion && !all_goals_reached(result)) {
    auto const orig_start = mapped_start.second;
    // 2nd attempt: expand start point
    auto const t_before_expand_start = timing_now();
    mapped_start.second =
        nearest_points(g, start, expanded_max_pt_query, expanded_max_pt_count,
                       expanded_max_pt_dist);
    result.stats_.start_pts_extended_++;
    result.stats_.d_start_pts_extended_ = ms_since(t_before_expand_start);
    find_routes(result, mapped_start, mapped_goals, profile, dir);
    if (!all_goals_reached(result)) {
      // 3rd attempt: expand goal points
      auto const expanded_start = mapped_start.second;
      mapped_start.second = orig_start;
      auto const t_before_expand_dest = timing_now();
      for (std::size_t i = 0; i < destinations.size(); i++) {
        if (result.routes_[i].empty()) {
          mapped_goals[i].second =
              nearest_points(g, destinations[i], expanded_max_pt_query,
                             expanded_max_pt_count, expanded_max_pt_dist);
          result.stats_.destination_pts_extended_++;
        }
      }
      result.stats_.d_destination_pts_extended_ =
          ms_since(t_before_expand_dest);
      find_routes(result, mapped_start, mapped_goals, profile, dir);
      if (!all_goals_reached(result)) {
        // 4th attempt: expand start and goal points
        mapped_start.second = expanded_start;
        find_routes(result, mapped_start, mapped_goals, profile, dir);
      }
    }
  }

  postprocess_result(result, profile);
  result.stats_.d_total_ = ms_since(t_start);
  return result;
}

// find routes with locations as destination and an osm-id of a given type as start
search_result find_routes(routing_graph const& g, std::int64_t const& start_id,
                          osm_type const& type,
                          std::vector<location> const& destinations,
                          search_profile const& profile, search_direction dir,
                          bool allow_expansion) {
  std::chrono::time_point<std::chrono::steady_clock> t_start = timing_now();
  std::optional<location> start;
  switch (type) {
    case osm_type::NODE:
      start = g.find_osm_node(start_id);
      break;
    case osm_type::AREA:
      start = g.find_osm_area(start_id);
      break;
    case osm_type::EDGE:
      start = g.find_osm_edge(start_id);
      break;
    default:
      start = g.find_osm_id(start_id);
  }

  if(start.has_value()) {
    return find_routes(g, start.value(), destinations, profile, dir, allow_expansion,
                       &t_start);
  }
  return find_routes(g, location(), destinations, profile, dir, allow_expansion,
                     &t_start);
}

// find routes with a location as start and osm-ids as destinations
search_result find_routes(routing_graph const&g, location const& start,
                          std::vector<osm_type> const& end_type,
                          std::vector<std::int64_t> const& destination_ids,
                          search_profile const& profile, search_direction dir,
                          bool allow_expansion) {
  std::chrono::time_point<std::chrono::steady_clock> t_start = timing_now();
  std::vector<location> destinations;
  auto it = destinations.begin();
  auto type_it = end_type.begin();
  for (std::int64_t destination_id : destination_ids) {
    std::optional<location> id;
    switch (*(type_it++)._Ptr) {
      case osm_type::NODE:
        id = g.find_osm_node(destination_id);
        break;
      case osm_type::AREA:
        id = g.find_osm_area(destination_id);
        break;
      case osm_type::EDGE:
        id = g.find_osm_edge(destination_id);
        break;
      default:
        id = g.find_osm_id(destination_id);
    }
    if(id.has_value()) {
      it = destinations.insert(it, id.value());
    }
  }
  return find_routes(g, start, destinations, profile, dir, allow_expansion, &t_start);
}

// find routes with osm-ids, as start and destinations
search_result find_routes(routing_graph const&g, std::int64_t const& start_id,
                          osm_type const& start_type,
                          std::vector<osm_type> const& end_type,
                          std::vector<std::int64_t> destination_ids,
                          search_profile const& profile, search_direction dir,
                          bool allow_expansion) {
  std::chrono::time_point<std::chrono::steady_clock> t_start = timing_now();
  std::vector<location> destinations;
  auto it = destinations.begin();
  auto type_it = end_type.begin();
  for (std::int64_t const& destination_id : destination_ids) {
    std::optional<location> id;
    switch (*(type_it++)._Ptr) {
      case osm_type::NODE:
        id = g.find_osm_node(destination_id);
        break;
      case osm_type::AREA:
        id = g.find_osm_area(destination_id);
        break;
      case osm_type::EDGE:
        id = g.find_osm_edge(destination_id);
        break;
      default:
        id = g.find_osm_id(destination_id);
    }
    if(id.has_value()) {
      it = destinations.insert(it, id.value());
    }
  }
  std::optional<location> start;
  switch (start_type) {
    case osm_type::NODE:
      start = g.find_osm_node(start_id);
      break;
    case osm_type::AREA:
      start = g.find_osm_area(start_id);
      break;
    case osm_type::EDGE:
      start = g.find_osm_edge(start_id);
      break;
    default:
      start = g.find_osm_id(start_id);
  }
  if(start.has_value()) {
    return find_routes(g, start.value(), destinations, profile, dir, allow_expansion,
                       &t_start);
  }
  return find_routes(g, location(), destinations, profile, dir, allow_expansion, &t_start);
}

}  // namespace ppr::routing
