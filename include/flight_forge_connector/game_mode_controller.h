// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <vector>

#include <flight_forge_connector/data_types.h>
#include <flight_forge_connector/socket_client.h>
#include <flight_forge_connector/serialization/serializable_shared.h>

#define API_VERSION_MAJOR 0
#define API_VERSION_MINOR 11

namespace ueds_connector
{

class GameModeController : public SocketClient {
public:
  GameModeController() : SocketClient() {
  }
  GameModeController(const std::string& address, uint16_t port) : SocketClient(address, port) {
  }

  std::pair<bool, std::vector<int>> GetDrones();

  std::pair<bool, int> SpawnDrone();

  std::pair<bool, int> SpawnDroneAtLocation(ueds_connector::Coordinates &Location, int &TypeUavID);

  bool RemoveDrone(const int port);

  std::pair<bool, CameraCaptureModeEnum> GetCameraCaptureMode();

  bool SetCameraCaptureMode(const CameraCaptureModeEnum& cameraCaptureMode);

  std::pair<bool, float> GetFps();

  std::pair<bool, std::pair<int,int>> GetApiVersion();

  std::pair<bool, double> GetTime();
  
  bool SetGraphicsSettings(const int& graphicsSettings);

  bool SwitchWorldLevel(const short& worldLevelEnum);

  bool SetForestDensity(const int DensityLevel);

  bool SetForestHillyLevel(const int HillyLevel);

  std::pair<bool, ueds_connector::Coordinates> GetWorldOrigin();

  bool SetWeather(const int& type_id);

  bool SetDatetime(const int& hour, const int& minute);

  bool SetMutualDroneVisibility(const bool& enabled);
};

}  // namespace ueds_connector
