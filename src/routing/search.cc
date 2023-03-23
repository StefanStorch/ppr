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
  search_result result;
  auto t_start = timing_now();
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

// find routes with osm-ids, as start and/or destinations
// start_loc and destination_locs are only used if the start/none of the destinations
// are found using the osm_ids
search_result find_routes(routing_graph const&g, std::int64_t const& start_id,
                          osm_type const& start_type, location const& start_loc,
                          std::vector<std::int64_t> const& destination_ids,
                          std::vector<osm_type> const& destination_types,
                          std::vector<location> const& destination_locs,
                          search_profile const& profile, search_direction dir,
                          bool /*allow_expansion*/) {
  assert(destination_ids.size() == destination_types.size());
  assert(destination_ids.size() == destination_locs.size());
  auto t_start = timing_now();
  search_result result;
  // search whether the osm object defined by id+type exists
  auto start = find_start(g, start_id, start_type);
  mapped_pt mapped_start;
  // if the osm object is not found use the start location as start
  if(!start) {
    mapped_start = {start_loc,
                    nearest_points(g, start_loc, initial_max_pt_query,
                                   initial_max_pt_count, initial_max_pt_dist)};
    result.stats_.start_is_location = true;
  } else {
    mapped_start = {start.input_, {start}};
  }
  auto const t_after_start = timing_now();
  result.stats_.d_start_pts_ = ms_between(t_start, t_after_start);
  // search whether the osm objects defined by id+type exist
  auto goals = find_destinations(g, destination_types, destination_ids);
  std::vector<mapped_pt> mapped_goals;
  mapped_goals.reserve(destination_ids.size());
  //check whether the osm objects were found and if any weren't found use the location for that destination
  for(int i = 0; i < goals.size(); ++i) {
    if (goals.at(i)) {
      mapped_goals.emplace_back(mapped_pt{goals.at(i).input_, {goals.at(i)}});
      result.stats_.destination_is_location.emplace_back(false);
    } else {
      mapped_goals.emplace_back(
          destination_locs.at(i),
          nearest_points(g, destination_locs.at(i), initial_max_pt_query,
                              initial_max_pt_count, initial_max_pt_dist));
      result.stats_.destination_is_location.emplace_back(true);
    }
  }
  auto const t_after_dest = timing_now();
  result.stats_.d_destination_pts_ = ms_between(t_after_start, t_after_dest);
  // 1st attempt: only nearest start + goal points
  find_routes(result, mapped_start, mapped_goals, profile, dir);
  if (!all_goals_reached(result)) {
    auto const orig_start1 = mapped_start.second;
    // 2nd attempt: use start location
    auto const t_before_expand_start1 = timing_now();
    mapped_start = {start_loc, nearest_points(g, start_loc, initial_max_pt_query,
                                   initial_max_pt_count, initial_max_pt_dist)};
    result.stats_.start_pts_extended_++;
    result.stats_.d_start_pts_extended_ = ms_since(t_before_expand_start1);
    find_routes(result, mapped_start, mapped_goals, profile, dir);
    if (!all_goals_reached(result)) {
      // 3rd attempt: use destination location
      auto const expanded_start1 = mapped_start.second;
      mapped_start.second = orig_start1;
      auto const t_before_expand_dest1 = timing_now();
      for (std::size_t i = 0; i < destination_locs.size(); i++) {
        if (result.routes_[i].empty()) {
          mapped_goals[i] = {destination_locs.at(i),
                             nearest_points(g, destination_locs.at(i), initial_max_pt_query,
                             initial_max_pt_count, initial_max_pt_dist)};
          result.stats_.destination_pts_extended_++;
        }
      }
      result.stats_.d_destination_pts_extended_ =
          ms_since(t_before_expand_dest1);
      find_routes(result, mapped_start, mapped_goals, profile, dir);
      if (!all_goals_reached(result)) {
        // 4th attempt: use start and destination location
        mapped_start.second = expanded_start1;
        find_routes(result, mapped_start, mapped_goals, profile, dir);
        if (!all_goals_reached(result)) {
          // 5th attempt: expand start point
          auto const t_before_expand_start = timing_now();
          mapped_start.second =
              nearest_points(g, start_loc, expanded_max_pt_query, expanded_max_pt_count,
                             expanded_max_pt_dist);
          result.stats_.start_pts_extended_++;
          result.stats_.d_start_pts_extended_ = ms_since(t_before_expand_start);
          find_routes(result, mapped_start, mapped_goals, profile, dir);
        }
      }
    }
  }
  result.stats_.d_total_ = ms_since(t_start);
  return result;
}

input_pt find_start(routing_graph const& g, std::int64_t const& start_id,
                    osm_type const& start_type) {
  if (start_id == -1) {
    return {};
  }
  area* area;
  edge* edge;
  node* node;
  switch (start_type) {
    case osm_type::NODE:
      node = g.find_osm_node(start_id);
      if (node != nullptr) {
        return input_pt{node};
      }
      break;
    case osm_type::AREA:
      area = g.find_osm_area(start_id);
      if (area != nullptr) {
        return input_pt{area->get_middle(), area};
      }
      break;
    case osm_type::EDGE:
      edge = g.find_osm_edge(start_id);
      if (edge != nullptr) {
        return input_pt{edge->from_};
      }
      break;
    default:
      area = g.find_osm_area(start_id);
      if (area != nullptr) {
        return input_pt{area->get_middle(), area};
      }
      edge = g.find_osm_edge(start_id);
      if (edge != nullptr) {
        return input_pt{edge->from_};
      }
      node = g.find_osm_node(start_id);
      if (node != nullptr) {
        return input_pt{node};
      }
  }
  return {};
}

std::vector<input_pt> find_destinations(routing_graph const& g,
                                        std::vector<osm_type> const& end_type,
                                        std::vector<std::int64_t> const& destination_ids) {
  std::vector<input_pt> destinations{};
  area* area;
  edge* edge;
  node* node;
  auto type_it = end_type.begin();
  for (std::int64_t destination_id : destination_ids) {
    if (destination_id == -1) {
      type_it++;
      destinations.emplace_back();
      continue;
    }
    switch (*(type_it++)._Ptr) {
      case osm_type::NODE:
        node = g.find_osm_node(destination_id);
        if (node != nullptr) {
          destinations.emplace_back(node);
        } else {
          destinations.emplace_back();
        }
        continue;
      case osm_type::AREA:
        area = g.find_osm_area(destination_id);
        if (area != nullptr) {
          destinations.emplace_back(area->get_middle(), area);
        } else {
          destinations.emplace_back();
        }
        continue;
      case osm_type::EDGE:
        edge = g.find_osm_edge(destination_id);
        if (edge != nullptr) {
          destinations.emplace_back(edge->from_);
        } else {
          destinations.emplace_back();
        }
        continue;
      default:
        area = g.find_osm_area(destination_id);
        if (area != nullptr) {
          destinations.emplace_back(area->get_middle(), area);
          continue;
        }
        edge = g.find_osm_edge(destination_id);
        if (edge != nullptr) {
          destinations.emplace_back(edge->from_);
          continue;
        }
        node = g.find_osm_node(destination_id);
        if (node != nullptr) {
          destinations.emplace_back(node);
          continue;
        }
        destinations.emplace_back();
    }
  }
  return destinations;
}

}  // namespace ppr::routing
