// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#include <flight_forge_connector/game_mode_controller.h>

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

std::pair<bool, int> GameModeController::SpawnDroneAtLocation(ueds_connector::Coordinates &Location, int &TypeUavID) {

  Serializable::GameMode::SpawnDroneAtLocation::Request request{};
  request.x = Location.x;
  request.y = Location.y;
  request.z = Location.z;
  request.idMesh = TypeUavID;

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

bool GameModeController::SetWeather(const int& type_id){

  Serializable::GameMode::SetWeather::Request request{};
  request.weather_id = type_id;

  Serializable::GameMode::SetWeather::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success; 
}

bool GameModeController::SetDatetime(const int& hour, const int& minute){

  Serializable::GameMode::SetDaytime::Request request{};
  request.hour = hour;
  request.minute = minute;

  Serializable::GameMode::SetDaytime::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success; 
}

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

/* setGraphicsSettings() {*/
bool GameModeController::SetGraphicsSettings(const int& graphicsSettings) {

  Serializable::GameMode::SetGraphicsSettings::Request request{};
  request.graphicsSettings = graphicsSettings;

  Serializable::GameMode::SetGraphicsSettings::Response response{};
  const auto                                             status  = Request(request, response);
  const auto                                             success = status && response.status;

  return success;
}
//}

/* SwitchWorldLevel() {*/
bool GameModeController::SwitchWorldLevel(const short& worldLevelEnum){

  Serializable::GameMode::SwitchWorldLevel::Request request{};
  request.worldLevelEnum = worldLevelEnum;

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

/* getTime() //{ */

std::pair<bool, double> GameModeController::GetTime() {

  Serializable::GameMode::GetTime::Request request{};

  Serializable::GameMode::GetTime::Response response{};
  const auto                                status  = Request(request, response);
  const auto                                success = status && response.status;

  return std::make_pair(success, success ? response.time : 0);
}

//}
