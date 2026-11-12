#include <flight_forge_connector/simulator_client.h>

#include <flight_forge_connector/wire_convert.h>

#include "reply_helpers.h"

namespace flightforge
{

using FBB = flatbuffers::FlatBufferBuilder;

SimulatorClient::SimulatorClient(const std::string& address, const uint16_t port) : connection_(std::make_shared<Connection>(address, port)) {
}

SimulatorClient::SimulatorClient(std::shared_ptr<Connection> connection) : connection_(std::move(connection)) {
}

/* connection //{ */

Status SimulatorClient::Ping() {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreatePingRequest(b)));
}

Expected<ApiInfo> SimulatorClient::GetInfo() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetInfoRequest(b));
  Status     status;
  const auto body = Body<wire::GetInfoResponse>(reply, status);
  if (!body) {
    return status;
  }

  ApiInfo info;
  info.major = body->api_major();
  info.minor = body->api_minor();
  if (body->schema()) {
    CopyScalars(info.schema, body->schema());
  }
  return info;
}

/* //} */

/* time //{ */

Expected<TimeInfo> SimulatorClient::GetTime() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetTimeRequest(b));
  Status     status;
  const auto body = Body<wire::GetTimeResponse>(reply, status);
  if (!body) {
    return status;
  }
  return TimeInfo{body->wall_time(), body->sim_time(), body->time_dilation()};
}

Expected<float> SimulatorClient::GetFps() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetFpsRequest(b));
  Status     status;
  const auto body = Body<wire::GetFpsResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->fps();
}

Expected<double> SimulatorClient::SetTimeDilation(const double dilation) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateSetTimeDilationRequest(b, dilation));
  Status     status;
  const auto body = Body<wire::SetTimeDilationResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->applied();
}

Expected<double> SimulatorClient::SetFixedTimestep(const double dt, const double adaptive_min_dt) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateSetFixedTimestepRequest(b, dt, adaptive_min_dt));
  Status     status;
  const auto body = Body<wire::SetFixedTimestepResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->applied();
}

Status SimulatorClient::SetLockstep(const bool enabled) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetLockstepRequest(b, enabled)));
}

Expected<StepResult> SimulatorClient::Step(const uint32_t ticks) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateStepRequest(b, ticks), std::nullopt, std::nullopt, step_timeout_);
  Status     status;
  const auto body = Body<wire::StepResponse>(reply, status);
  if (!body) {
    return status;
  }
  return StepResult{body->sim_time(), body->tick_count()};
}

/* //} */

/* world //{ */

Status SimulatorClient::SwitchWorld(const std::string& level) {
  FBB        b;
  const auto name = b.CreateString(level);
  return AsStatus(connection_->Call(b, wire::CreateSwitchWorldRequest(b, name)));
}

Status SimulatorClient::SetWeather(const wire::Weather weather) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetWeatherRequest(b, weather)));
}

Status SimulatorClient::SetDaytime(const int hour, const int minute) {
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59) {
    return Status::Fail("not a time of day");
  }
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetDaytimeRequest(b, static_cast<uint8_t>(hour), static_cast<uint8_t>(minute))));
}

Status SimulatorClient::SetGraphicsSettings(const wire::GraphicsQuality quality) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetGraphicsSettingsRequest(b, quality)));
}

Status SimulatorClient::SetForestDensity(const int level) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetForestDensityRequest(b, level)));
}

Status SimulatorClient::SetForestHillyLevel(const int level) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetForestHillyLevelRequest(b, level)));
}

Status SimulatorClient::SetMutualVisibility(const bool enabled) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetMutualVisibilityRequest(b, enabled)));
}

Expected<wire::CaptureMode> SimulatorClient::GetCaptureMode() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetCaptureModeRequest(b));
  Status     status;
  const auto body = Body<wire::GetCaptureModeResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->mode();
}

Status SimulatorClient::SetCaptureMode(const wire::CaptureMode mode) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetCaptureModeRequest(b, mode)));
}

Expected<Vec3> SimulatorClient::GetWorldOrigin() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetWorldOriginRequest(b));
  Status     status;
  const auto body = Body<wire::GetWorldOriginResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->origin() ? FromWire(*body->origin()) : Vec3{};
}

Expected<wire::GetClassMapResponseT> SimulatorClient::GetClassMap() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetClassMapRequest(b));
  Status     status;
  const auto body = Body<wire::GetClassMapResponse>(reply, status);
  if (!body) {
    return status;
  }
  wire::GetClassMapResponseT out;
  body->UnPackTo(&out);
  return out;
}

/* //} */

/* scene //{ */

Expected<std::vector<int>> SimulatorClient::SpawnObjectsFromYaml(const std::string& yaml) {
  FBB        b;
  const auto document = b.CreateString(yaml);
  const auto reply    = connection_->Call(b, wire::CreateSpawnObjectsFromYamlRequest(b, document));
  Status     status;
  const auto body = Body<wire::SpawnObjectsFromYamlResponse>(reply, status);
  if (!body) {
    return status;
  }
  std::vector<int> ids;
  if (body->object_ids()) {
    CopyScalars(ids, body->object_ids());
  }
  return ids;
}

Expected<int> SimulatorClient::RemoveSpawnedObject(const int object_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateRemoveSpawnedObjectRequest(b, object_id));
  Status     status;
  const auto body = Body<wire::RemoveSpawnedObjectResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->removed_count();
}

Expected<int> SimulatorClient::RemoveAllSpawnedObjects() {
  return RemoveSpawnedObject(-1);
}

Status SimulatorClient::MoveSpawnedObject(const int object_id, const Pose& pose, const Vec3& scale) {
  FBB              b;
  const wire::Pose p = ToWire(pose);
  const wire::Vec3 s = ToWire(scale);
  return AsStatus(connection_->Call(b, wire::CreateMoveSpawnedObjectRequest(b, object_id, &p, &s)));
}

Expected<std::vector<SpawnedObject>> SimulatorClient::ListSpawnedObjects() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateListSpawnedObjectsRequest(b));
  Status     status;
  const auto body = Body<wire::ListSpawnedObjectsResponse>(reply, status);
  if (!body) {
    return status;
  }

  std::vector<SpawnedObject> objects;
  if (body->objects()) {
    for (const wire::SpawnedObject* object : *body->objects()) {
      SpawnedObject out;
      out.id      = object->id();
      out.asset   = object->asset() ? object->asset()->str() : std::string();
      out.stencil = object->stencil();
      if (object->pose()) {
        out.pose = FromWire(*object->pose());
      }
      if (object->scale()) {
        out.scale = FromWire(*object->scale());
      }
      objects.push_back(std::move(out));
    }
  }
  return objects;
}

Expected<std::string> SimulatorClient::ExportScene() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateExportSceneRequest(b));
  Status     status;
  const auto body = Body<wire::ExportSceneResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->yaml() ? body->yaml()->str() : std::string();
}

Expected<AssetList> SimulatorClient::ListAssets(const std::string& path_prefix, const std::string& name_filter) {
  FBB        b;
  const auto prefix = b.CreateString(path_prefix);
  const auto filter = b.CreateString(name_filter);
  const auto reply  = connection_->Call(b, wire::CreateListAssetsRequest(b, prefix, filter));
  Status     status;
  const auto body = Body<wire::ListAssetsResponse>(reply, status);
  if (!body) {
    return status;
  }

  AssetList list;
  list.truncated = body->truncated();
  if (body->assets()) {
    for (const flatbuffers::String* name : *body->assets()) {
      list.assets.push_back(name->str());
    }
  }
  return list;
}

Expected<int> SimulatorClient::LoadLibrary(const std::string& path) {
  FBB        b;
  const auto where = b.CreateString(path);
  const auto reply = connection_->Call(b, wire::CreateLoadLibraryRequest(b, where));
  Status     status;
  const auto body = Body<wire::LoadLibraryResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->count();
}

Expected<LibraryPage> SimulatorClient::ListLibrary(const std::string& library, const std::string& object_class, const std::string& filter,
                                                   const uint32_t offset, const uint32_t limit) {
  FBB        b;
  const auto lib   = b.CreateString(library);
  const auto cls   = b.CreateString(object_class);
  const auto words = b.CreateString(filter);
  const auto reply = connection_->Call(b, wire::CreateListLibraryRequest(b, lib, cls, words, offset, limit));
  Status     status;
  const auto body = Body<wire::ListLibraryResponse>(reply, status);
  if (!body) {
    return status;
  }

  LibraryPage page;
  page.total = body->total();
  if (body->objects()) {
    for (const wire::LibraryObject* object : *body->objects()) {
      page.objects.emplace_back();
      object->UnPackTo(&page.objects.back());
    }
  }
  return page;
}

Expected<WorldMesh> SimulatorClient::GetWorldMesh(const Vec3& center, const Vec3& extent, const uint32_t max_triangles) {
  FBB              b;
  const wire::Vec3 c = ToWire(center);
  const wire::Vec3 e = ToWire(extent);
  const auto       reply = connection_->Call(b, wire::CreateGetWorldMeshRequest(b, &c, &e, max_triangles));
  Status           status;
  const auto       body = Body<wire::GetWorldMeshResponse>(reply, status);
  if (!body) {
    return status;
  }

  WorldMesh mesh;
  mesh.truncated    = body->truncated();
  mesh.source_count = body->source_count();
  if (body->vertices()) {
    mesh.vertices.resize(body->vertices()->size());
    std::memcpy(mesh.vertices.data(), body->vertices()->Data(), mesh.vertices.size() * sizeof(wire::Vec3f));
  }
  if (body->indices()) {
    CopyScalars(mesh.indices, body->indices());
  }
  return mesh;
}

Expected<std::vector<float>> SimulatorClient::GetDistanceBatch(const std::vector<std::array<float, 3>>& points, const double max_radius, const double tolerance,
                                                               const bool terrain_only) {
  static_assert(sizeof(std::array<float, 3>) == sizeof(wire::Vec3f));

  FBB         b;
  wire::Vec3f* out    = nullptr;
  const auto   vector = b.CreateUninitializedVectorOfStructs(points.size(), &out);
  std::memcpy(out, points.data(), points.size() * sizeof(wire::Vec3f));

  const auto reply = connection_->Call(b, wire::CreateGetDistanceBatchRequest(b, vector, max_radius, tolerance, terrain_only));
  Status     status;
  const auto body = Body<wire::GetDistanceBatchResponse>(reply, status);
  if (!body) {
    return status;
  }
  std::vector<float> distances;
  if (body->distances()) {
    CopyScalars(distances, body->distances());
  }
  return distances;
}

Expected<std::vector<double>> SimulatorClient::GetSurfaceHeight(const std::vector<Vec3>& points, const bool ignore_robots) {
  static_assert(sizeof(Vec3) == sizeof(wire::Vec3));

  FBB         b;
  wire::Vec3* out    = nullptr;
  const auto  vector = b.CreateUninitializedVectorOfStructs(points.size(), &out);
  std::memcpy(out, points.data(), points.size() * sizeof(wire::Vec3));

  const auto reply = connection_->Call(b, wire::CreateGetSurfaceHeightRequest(b, vector, ignore_robots));
  Status     status;
  const auto body = Body<wire::GetSurfaceHeightResponse>(reply, status);
  if (!body) {
    return status;
  }
  std::vector<double> heights;
  CopyScalars(heights, body->heights());
  if (heights.size() != points.size()) {
    return Status::Fail("get_surface_height answered " + std::to_string(heights.size()) + " heights for " + std::to_string(points.size()) + " points");
  }
  return heights;
}

/* //} */

/* robots //{ */

Expected<int> SimulatorClient::SpawnRobot(const std::string& name, const Pose& pose, const std::string& label) {
  FBB              b;
  const auto       frame = b.CreateString(name);
  const auto       tag   = b.CreateString(label);
  const wire::Pose p     = ToWire(pose);
  const auto       reply = connection_->Call(b, wire::CreateSpawnRobotRequest(b, frame, 0, &p, tag));
  Status           status;
  const auto       body = Body<wire::SpawnRobotResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->robot_id();
}

Expected<std::vector<RobotInfo>> SimulatorClient::ListRobots() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateListRobotsRequest(b));
  Status     status;
  const auto body = Body<wire::ListRobotsResponse>(reply, status);
  if (!body) {
    return status;
  }

  std::vector<RobotInfo> robots;
  if (body->robots()) {
    for (const wire::RobotInfo* robot : *body->robots()) {
      robots.push_back(RobotInfo{robot->id(), robot->name() ? robot->name()->str() : std::string(), robot->kind(),
                                 robot->label() ? robot->label()->str() : std::string()});
    }
  }
  return robots;
}

Status SimulatorClient::RemoveRobot(const int robot_id) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateRemoveRobotRequest(b), robot_id));
}

Expected<std::vector<SetStateResult>> SimulatorClient::SetStates(const std::vector<std::pair<int, RobotState>>& states, const bool collide,
                                                                 const bool wait_applied) {
  FBB                                                b;
  std::vector<flatbuffers::Offset<wire::RobotStateEntry>> entries;
  for (const auto& [id, state] : states) {
    const auto wire_state = BuildState(b, state);
    entries.push_back(wire::CreateRobotStateEntry(b, id, wire_state));
  }
  const auto list  = b.CreateVector(entries);
  const auto reply = connection_->Call(b, wire::CreateSetStatesRequest(b, list, collide, wait_applied));
  if (!reply) {
    return AsStatus(reply);
  }

  std::vector<SetStateResult> results;
  if (const auto body = reply.body<wire::SetStatesResponse>(); body && body->results()) {
    for (const wire::SetStateResponse* result : *body->results()) {
      results.push_back(ReadStateResult(result));
    }
  }
  return results;
}

/* //} */

}  // namespace flightforge
