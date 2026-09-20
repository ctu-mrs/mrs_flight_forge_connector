#include <flight_forge_connector/game_mode_controller.h>

#include <algorithm>
#include <sstream>

using kissnet::socket_status;
using ueds_connector::CameraCaptureModeEnum;
using ueds_connector::GameModeController;

/* getDrones() //{ */

std::pair<bool, std::vector<int>> GameModeController::GetDrones() {

  Serializable::GameMode::GetDrones::Request request{};

  Serializable::GameMode::GetDrones::Response response{};
  const auto                                  status  = Request(request, response);
  const auto                                  success = status && response.status;

  return std::make_pair(success, success ? response.ports : std::vector<int>());
}

//}

/* GetWorldOrigin() //{ */

std::pair<bool, ueds_connector::Coordinates> GameModeController::GetWorldOrigin() {

  Serializable::GameMode::GetWorldOrigin::Request request{};

  Serializable::GameMode::GetWorldOrigin::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  ueds_connector::Coordinates coordinates{};

  if (success) {
    coordinates.x = response.x;
    coordinates.y = response.y;
    coordinates.z = response.z;
  }

  return std::make_pair(success, coordinates);
}

//}

/* spawnDrone() //{ */

std::pair<bool, int> GameModeController::SpawnDrone() {

  Serializable::GameMode::SpawnDrone::Request request{};

  Serializable::GameMode::SpawnDrone::Response response{};
  const auto                                   status  = Request(request, response);
  const auto                                   success = status && response.status;

  return std::make_pair(success, success ? response.port : 0);
}

//}

/* spawnDrone() //{ */

std::pair<bool, int> GameModeController::SpawnDroneAtLocation(ueds_connector::Coordinates &Location, std::string &TypeUav) {

  Serializable::GameMode::SpawnDroneAtLocation::Request request{};
  request.x = Location.x;
  request.y = Location.y;
  request.z = Location.z;
  request.MeshName = TypeUav;

  Serializable::GameMode::SpawnDroneAtLocation::Response response{};
  const auto                                   status  = Request(request, response);
  const auto                                   success = status && response.status;

  return std::make_pair(success, success ? response.port : 0);
}

//}

/* removeDrone() //{ */

bool GameModeController::RemoveDrone(const int port) {

  Serializable::GameMode::RemoveDrone::Request request{};
  request.port = port;

  Serializable::GameMode::RemoveDrone::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success;
}

//}

/* SetForestDensity() //{ */

bool GameModeController::SetForestDensity(const int DensityLevel) {

  Serializable::GameMode::SetForestDensity::Request request{};
  request.Density_Level = DensityLevel;

  Serializable::GameMode::SetForestDensity::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success;
}

//}

/* SetForestHillyLevel() //{ */

bool GameModeController::SetForestHillyLevel(const int HillyLevel) {

  Serializable::GameMode::SetForestHillyLevel::Request request{};
  request.Hilly_Level = HillyLevel;

  Serializable::GameMode::SetForestHillyLevel::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success;
}

//}

/* SetWeather() //{ */

bool GameModeController::SetWeather(const int& type_id){

  Serializable::GameMode::SetWeather::Request request{};
  request.weather_id = type_id;

  Serializable::GameMode::SetWeather::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success; 
}

//}

/* SetDatetime() //{ */

bool GameModeController::SetDatetime(const int& hour, const int& minute){

  Serializable::GameMode::SetDaytime::Request request{};
  request.hour = hour;
  request.minute = minute;

  Serializable::GameMode::SetDaytime::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success; 
}

//}

/* SetMutualDroneVisibility() //{ */

bool ueds_connector::GameModeController::SetMutualDroneVisibility(const bool &enabled)
{
  Serializable::GameMode::SetMutualVisibility::Request request{};
  request.mutual_visibiliti_enabled = enabled;

  Serializable::GameMode::SetMutualVisibility::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success;
}

//}

/* getCameraCaptureMode() //{ */

std::pair<bool, CameraCaptureModeEnum> GameModeController::GetCameraCaptureMode() {

  Serializable::GameMode::GetCameraCaptureMode::Request request{};

  Serializable::GameMode::GetCameraCaptureMode::Response response{};
  const auto                                             status  = Request(request, response);
  const auto                                             success = status && response.status;

  CameraCaptureModeEnum captureMode = CameraCaptureModeEnum::CAPTURE_ALL_FRAMES;
  if (success) {
    captureMode = static_cast<CameraCaptureModeEnum>(response.cameraCaptureMode);
  }

  return std::make_pair(success, captureMode);
}

//}

/* setCameraCaptureMode() //{ */

bool GameModeController::SetCameraCaptureMode(const CameraCaptureModeEnum& cameraCaptureMode) {

  Serializable::GameMode::SetCameraCaptureMode::Request request{};
  request.cameraCaptureMode = static_cast<Serializable::GameMode::CameraCaptureModeEnum>(cameraCaptureMode);

  Serializable::GameMode::SetCameraCaptureMode::Response response{};
  const auto                                             status  = Request(request, response);
  const auto                                             success = status && response.status;

  return success;
}

//}

/* SetGraphicsSettings() //{ */

bool GameModeController::SetGraphicsSettings(const int& graphicsSettings) {

  Serializable::GameMode::SetGraphicsSettings::Request request{};
  request.graphicsSettings = graphicsSettings;

  Serializable::GameMode::SetGraphicsSettings::Response response{};
  const auto                                             status  = Request(request, response);
  const auto                                             success = status && response.status;

  return success;
}

//}

/* SwitchWorldLevel() //{ */

bool GameModeController::SwitchWorldLevel(const std::string& worldLevelName){

  Serializable::GameMode::SwitchWorldLevel::Request request{};
  request.worldLevelName = worldLevelName;

  Serializable::GameMode::SwitchWorldLevel::Response response{};
  const auto                                             status  = Request(request, response);
  const auto                                             success = status && response.status;

  return success;
}

//}

/* getFps() //{ */

std::pair<bool, float> GameModeController::GetFps() {

  Serializable::GameMode::GetFps::Request request{};

  Serializable::GameMode::GetFps::Response response{};
  const auto                               status  = Request(request, response);
  const auto                               success = status && response.status;

  return std::make_pair(success, success ? response.fps : 0);
}

//}

/* getApiVersion() //{ */

std::pair<bool, std::pair<int,int>> GameModeController::GetApiVersion() {

  Serializable::GameMode::GetApiVersion::Request request{};

  Serializable::GameMode::GetApiVersion::Response response{};
  const auto                                      status  = Request(request, response);
  const auto                                      success = status && response.status;

  return std::make_pair(success, success ? std::make_pair(response.api_version_major, response.api_version_minor) : std::make_pair(-1, -1));
}

//}

/* getClassMap() //{ */

std::pair<bool, Serializable::GameMode::GetClassMap::Response> GameModeController::GetClassMap() {

  Serializable::GameMode::GetClassMap::Request request{};

  Serializable::GameMode::GetClassMap::Response response{};
  const auto                                    status = Request(request, response);

  return std::make_pair(status && response.status, response);
}

//}

/* getTime() //{ */

std::pair<bool, double> GameModeController::GetTime() {

  Serializable::GameMode::GetTime::Request request{};

  Serializable::GameMode::GetTime::Response response{};
  const auto                                status  = Request(request, response);
  const auto                                success = status && response.status;

  return std::make_pair(success, success ? response.time : 0);
}

//}

/* getSimTime() //{ */

bool GameModeController::GetSimTime(double& out_sim_seconds, double& out_dilation) {

  Serializable::GameMode::GetTime::Request request{};

  Serializable::GameMode::GetTime::Response response{};
  const auto                                status  = Request(request, response);
  const auto                                success = status && response.status;

  if (success) {
    out_sim_seconds = response.sim_time;
    out_dilation    = response.time_dilation;
  }

  return success;
}

//}

/* setFixedTimestep() //{ */

bool GameModeController::SetFixedTimestep(double dt, double adaptive_min_dt, double& out_applied) {

  Serializable::GameMode::SetFixedTimestep::Request request{};
  request.dt              = dt;
  request.adaptive_min_dt = adaptive_min_dt;

  Serializable::GameMode::SetFixedTimestep::Response response{};
  const auto                                         status  = Request(request, response);
  const auto                                         success = status && response.status;

  if (success) {
    out_applied = response.applied;
  }

  return success;
}

//}

/* setLockstep() //{ */

bool GameModeController::SetLockstep(bool enabled) {

  Serializable::GameMode::SetLockstep::Request request{};
  request.enabled = enabled;

  Serializable::GameMode::SetLockstep::Response response{};
  const auto                                    status = Request(request, response);

  return status && response.status;
}

//}

/* step() //{ */

bool GameModeController::Step(int ticks, double& out_sim_time, uint64_t& out_tick_count) {

  Serializable::GameMode::Step::Request request{};
  request.ticks = ticks;

  // the reply comes only after every tick ran and rendered; the server itself gives up after a minute
  const int saved_timeout = GetReceiveTimeoutMs();
  SetReceiveTimeoutMs(std::max(saved_timeout, 90000));

  Serializable::GameMode::Step::Response response{};
  const auto                             status  = Request(request, response);
  const auto                             success = status && response.status;

  SetReceiveTimeoutMs(saved_timeout);

  if (success) {
    out_sim_time   = response.sim_time;
    out_tick_count = response.tick_count;
  }

  return success;
}

//}

/* setTimeDilation() //{ */

bool GameModeController::SetTimeDilation(double dilation, double& out_applied) {

  Serializable::GameMode::SetTimeDilation::Request request{};
  request.dilation = dilation;

  Serializable::GameMode::SetTimeDilation::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  if (success) {
    out_applied = response.applied;
  }

  return success;
}

//}

/* spawnObjectsFromYaml() //{ */

std::pair<bool, std::vector<int>> GameModeController::SpawnObjectsFromYaml(const std::string& yaml, std::string& OutError) {

  Serializable::GameMode::SpawnObjectsFromYaml::Request request{};
  request.yaml = yaml;

  Serializable::GameMode::SpawnObjectsFromYaml::Response response{};
  const auto                                            status  = Request(request, response);
  const auto                                            success = status && response.status;

  OutError = status ? response.error : "no response from the simulator";

  return std::make_pair(success, success ? response.object_ids : std::vector<int>());
}

//}

/* removeSpawnedObject() //{ */

std::pair<bool, int> GameModeController::RemoveSpawnedObject(const int object_id) {

  Serializable::GameMode::RemoveSpawnedObject::Request request{};
  request.object_id = object_id;

  Serializable::GameMode::RemoveSpawnedObject::Response response{};
  const auto                                           status  = Request(request, response);
  const auto                                           success = status && response.status;

  return std::make_pair(success, success ? response.removed_count : 0);
}

//}

/* removeAllSpawnedObjects() //{ */

std::pair<bool, int> GameModeController::RemoveAllSpawnedObjects() {

  // A negative id is the clear-everything sentinel.
  return RemoveSpawnedObject(-1);
}

//}

/* moveSpawnedObject() //{ */

bool GameModeController::MoveSpawnedObject(const int object_id, const std::array<double, 3>& position, const std::array<double, 3>& orientation,
                                           const std::array<double, 3>& scale) {

  Serializable::GameMode::MoveSpawnedObject::Request request{};
  request.object_id   = object_id;
  request.position    = position;
  request.orientation = orientation;
  request.scale       = scale;

  Serializable::GameMode::MoveSpawnedObject::Response response{};
  const auto                                          status = Request(request, response);

  return status && response.status;
}

//}

/* listSpawnedObjects() //{ */

std::pair<bool, std::vector<Serializable::GameMode::SpawnedObjectInfo>> GameModeController::ListSpawnedObjects() {

  Serializable::GameMode::ListSpawnedObjects::Request request{};

  Serializable::GameMode::ListSpawnedObjects::Response response{};
  const auto                                           status  = Request(request, response);
  const auto                                           success = status && response.status;

  return std::make_pair(success, success ? response.objects : std::vector<Serializable::GameMode::SpawnedObjectInfo>());
}

//}

/* exportScene() //{ */

std::pair<bool, std::string> GameModeController::ExportScene() {

  Serializable::GameMode::ExportScene::Request request{};

  Serializable::GameMode::ExportScene::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return std::make_pair(success, success ? response.yaml : std::string());
}

//}

/* getWorldMesh() //{ */

bool GameModeController::GetWorldMesh(const std::array<double, 3>& center, const std::array<double, 3>& extent, uint32_t max_triangles,
                                      std::vector<float>& out_vertices, std::vector<uint32_t>& out_indices, bool& out_truncated, int32_t& out_sources) {

  Serializable::GameMode::GetWorldMesh::Request request{};
  request.center_x_      = center[0];
  request.center_y_      = center[1];
  request.center_z_      = center[2];
  request.extent_x_      = extent[0];
  request.extent_y_      = extent[1];
  request.extent_z_      = extent[2];
  request.max_triangles_ = max_triangles;

  Serializable::GameMode::GetWorldMesh::Response response{};
  const auto                                     status  = Request(request, response);
  const auto                                     success = status && response.status;

  if (success) {
    out_vertices  = std::move(response.vertices_);
    out_indices   = std::move(response.indices_);
    out_truncated = response.truncated_;
    out_sources   = response.source_count_;
  }

  return success;
}

//}

/* getDistanceBatch() //{ */

bool GameModeController::GetDistanceBatch(const std::vector<float>& query_points, double max_radius, double tolerance, bool terrain_only,
                                          std::vector<float>& out_distances) {

  Serializable::GameMode::GetDistanceBatch::Request request{};
  request.query_points_ = query_points;
  request.max_radius_   = max_radius;
  request.tolerance_    = tolerance;
  request.terrain_only_ = terrain_only;

  Serializable::GameMode::GetDistanceBatch::Response response{};
  const auto                                         status  = Request(request, response);
  const auto                                         success = status && response.status;

  if (success) {
    out_distances = std::move(response.distances_);
  }

  return success;
}

//}

/* listAssets() //{ */

std::pair<bool, std::vector<std::string>> GameModeController::ListAssets(const std::string& path_prefix, const std::string& name_filter,
                                                                         bool& OutTruncated) {

  Serializable::GameMode::ListAssets::Request request{};
  request.path_prefix = path_prefix;
  request.name_filter = name_filter;

  Serializable::GameMode::ListAssets::Response response{};
  const auto                                   status  = Request(request, response);
  const auto                                   success = status && response.status;

  OutTruncated = success && response.truncated;

  return std::make_pair(success, success ? response.assets : std::vector<std::string>());
}

//}
