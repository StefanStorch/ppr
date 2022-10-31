#include <iostream>
#include <vector>

#include "boost/filesystem.hpp"

#include "ppr/cmd/benchmark/prog_options.h"
#include "ppr/serialization/reader.h"
#include "ppr/routing/search.h"

using namespace ppr;
using namespace ppr::benchmark;
using namespace ppr::serialization;
using namespace ppr::routing;

void find_locations_by_ids(std::vector<int64_t> const& ids, routing_graph const& rg) {
  for(auto id: ids) {
    auto myLocation = rg.find_osm_id(id);

    if (myLocation.has_value()) {
      std::cout << myLocation.value() << std::endl;
    } else {
      std::cout << "No Location with the ID " << id << " found!" << std::endl;
    }
  }
}

void print_route(search_result const& result) {
  if(result.destinations_reached() < 1) {
    std::cout << "No route found" << std::endl;
    return;
  }
  for (auto const& route : result.routes_) {
    for (auto const& route_part : route) {
      std::cout << "distance: " << route_part.distance_ << " duration:  " << route_part.duration_ << std::endl;
    }
  }
}

int main(int argc, char const* argv[]) {
  std::cout.precision(12);
  std::cerr.precision(12);
  prog_options opt;

  if (!boost::filesystem::exists(opt.graph_file_)) {
    std::cerr << "File not found: " << opt.graph_file_ << std::endl;
    return 1;
  }
  if (!opt.station_file_.empty() &&
      !boost::filesystem::exists(opt.station_file_)) {
    std::cerr << "File not found: " << opt.station_file_ << std::endl;
    return 1;
  }

  boost::filesystem::path graph_path{opt.graph_file_};
  auto const map_name = graph_path.stem().string();

  routing_graph rg;
  std::cout << "Loading routing graph..." << std::endl;
  read_routing_graph(rg, opt.graph_file_);

  std::cout << "Routing graph: " << rg.data_->nodes_.size() << " nodes, "
            << rg.data_->areas_.size() << " areas" << std::endl;

  std::cout << "Creating r-trees..." << std::endl;
  rg.prepare_for_routing();

  std::vector<int64_t> ids{1096061370, 4743094, 1091110991, 1101668808, 967515792, 463507836, 182243822};
  auto start_type = ppr::routing::osm_type::UNKNOWN;
  auto end_type = std::vector{start_type, start_type, start_type};
  auto end_type2 = std::vector{start_type};
  auto destination = std::vector{std::int64_t{1091110991}, std::int64_t{967515792}, std::int64_t{4743094}};
  auto destination2 = std::vector{std::int64_t{182243822}};
  auto profile = search_profile();
  auto direction = search_direction::FWD;
  auto start_location = make_location(9.782356, 50.321303);
  auto end_location = std::vector{make_location(9.768958, 50.326320)};
  auto response = find_routes(rg, 1101668808,
                              start_type,
                              end_type,
                              destination,
                              profile,
                              direction,
                              true);
  auto response2 = find_routes(rg, 463507836,
                              start_type,
                              end_type2,
                              destination2,
                              profile,
                              direction,
                              true);
  auto response3 = find_routes(rg, start_location,
                               end_location, profile, direction,
                               true);
  print_route(response);
  std::cout << "2." << std::endl;
  print_route(response2);
  std::cout << "3." << std::endl;
  print_route(response3);

  find_locations_by_ids(ids, rg);

  return 0;
}
