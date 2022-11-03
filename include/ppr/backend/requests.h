#pragma once

#include <vector>

#include "ppr/common/location.h"
#include "ppr/routing/search.h"
#include "ppr/routing/search_profile.h"

namespace ppr::backend {

struct route_request {
  location start_;
  location destination_;
  int64_t osm_id_start_{-1};
  int64_t osm_id_destination_{-1};
  ppr::routing::osm_type start_type_{ppr::routing::osm_type::UNKNOWN};
  ppr::routing::osm_type destination_type_{ppr::routing::osm_type::UNKNOWN};
  ppr::routing::search_profile profile_;
  bool include_infos_{};
  bool include_full_path_{};
  bool include_steps_{};
  bool include_steps_path_{};
  bool include_edges_{};
  bool include_statistics_{};
};

struct graph_request {
  std::vector<location> waypoints_;
  bool include_areas_{};
  bool include_visibility_graphs_{};
};

}  // namespace ppr::backend
