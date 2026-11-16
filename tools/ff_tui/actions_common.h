#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "app.h"

// shorthand for building action tables
inline Action MakeAction(std::string category, std::string name, Target target, std::string help, std::function<bool(Ctx&)> run) {
  Action a;
  a.category = std::move(category);
  a.name     = std::move(name);
  a.target   = target;
  a.help     = std::move(help);
  a.run      = std::move(run);
  return a;
}

template <typename P>
Action& Bind(Action& a, std::shared_ptr<P> params, std::vector<FieldRow> rows) {
  a.params = params;
  a.rows   = std::move(rows);
  PullAll(a.rows);
  return a;
}

inline std::vector<std::pair<std::string, int>> SortedById(const std::map<std::string, int>& m) {
  std::vector<std::pair<std::string, int>> v(m.begin(), m.end());
  std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
  return v;
}

inline std::vector<std::pair<std::string, int>> SensorTypeOptions() {
  return {{"rgb camera", ueds_connector::SENSOR_RGB_CAMERA},
          {"lidar", ueds_connector::SENSOR_LIDAR},
          {"rangefinder", ueds_connector::SENSOR_RANGEFINDER},
          {"lidar livox", ueds_connector::SENSOR_LIDAR_LIVOX},
          {"rgb seg camera", ueds_connector::SENSOR_RGB_SEG_CAMERA},
          {"stereo camera", ueds_connector::SENSOR_STEREO_CAMERA},
          {"event camera", ueds_connector::SENSOR_EVENT_CAMERA},
          {"fisheye camera", ueds_connector::SENSOR_FISHEYE_CAMERA},
          {"instance seg camera", ueds_connector::SENSOR_INSTANCE_SEG_CAMERA},
          {"depth camera", ueds_connector::SENSOR_DEPTH_CAMERA}};
}

inline std::string SensorTypeName(int type) {
  for (auto& [name, id] : SensorTypeOptions()) {
    if (id == type) {
      return name;
    }
  }
  return "type " + std::to_string(type);
}

// meta/compat calls exist on both sockets
template <typename Client>
bool DescribeMeta(Ctx& c, Client* client) {
  auto [ok, meta] = client->GetMeta();
  if (!ok) {
    c.line("GetMeta failed");
    return false;
  }
  c.line("api v" + std::to_string(meta.api_version_major_) + "." + std::to_string(meta.api_version_minor_) + ", " + std::to_string(meta.ops_.size()) +
         " ops");
  for (const auto& op : meta.ops_) {
    c.line("  " + std::to_string(op.id_) + " " + op.name_ + "  req " + op.request_);
  }
  return true;
}

template <typename Client>
bool DescribeCompat(Ctx& c, Client* client) {
  auto [ok, missing] = client->CheckCompatibility();
  if (!ok) {
    c.line("CheckCompatibility failed");
    return false;
  }
  if (missing.empty()) {
    c.line("compatible: every op matches");
  } else {
    c.line(std::to_string(missing.size()) + " op(s) missing or different on the server:");
    for (auto& name : missing) {
      c.line("  " + name);
    }
  }
  return true;
}
