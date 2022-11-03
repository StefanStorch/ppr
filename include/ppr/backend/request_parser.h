#include "rapidjson/document.h"

#include "ppr/common/location.h"

#include "ppr/profiles/parse_search_profile.h"

namespace ppr::backend {

inline void get_waypoints(std::vector<location>& waypoints,
                          rapidjson::Value const& doc, char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsArray()) {
      for (auto i = 0U; i < val.Size() - 1; i += 2) {
        waypoints.emplace_back(
            make_location(val[i].GetDouble(), val[i + 1].GetDouble()));
      }
    }
  }
}

inline void get_location(location& loc, rapidjson::Value const& doc,
                         char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsObject()) {
      auto const& lng = val.FindMember("lng");
      auto const& lat = val.FindMember("lat");
      if (lng != val.MemberEnd() && lat != val.MemberEnd() &&
          lng->value.IsNumber() && lat->value.IsNumber()) {
        loc = make_location(lng->value.GetDouble(), lat->value.GetDouble());
      }
    }
  }
}

inline void get_profile(ppr::routing::search_profile& profile,
                        rapidjson::Value const& doc, char const* key) {
  if (doc.HasMember(key)) {
    profiles::parse_search_profile(profile, doc[key]);
  }
}

inline void get_int64(int64_t& value, rapidjson::Value const& doc, char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsInt64()) {
      value = static_cast<int64_t>(val.GetInt64());
    }
  }
}

inline void get_type(ppr::routing::osm_type& type,
                     rapidjson::Value const& doc, char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsString()) {
      auto const& str_val = (std::string)val.GetString();
      if (str_val == "NODE") {
        type = ppr::routing::osm_type::NODE;
        return;
      }
      if (str_val == "AREA") {
        type = ppr::routing::osm_type::AREA;
        return;
      }
      if (str_val == "EDGE") {
        type = ppr::routing::osm_type::EDGE;
        return;
      }
    }
  }
  type = ppr::routing::osm_type::UNKNOWN;
}

}  // namespace ppr::backend
