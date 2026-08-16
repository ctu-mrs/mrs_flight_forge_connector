#pragma once

#include <string>
#include <vector>

#include <flight_forge_connector/data_types.h>
#include <flight_forge_connector/socket_client.h>
#include <flight_forge_connector/serialization/serializable_shared.h>

#define API_VERSION_MAJOR 0
#define API_VERSION_MINOR 15

namespace ueds_connector
{

class GameModeController : public SocketClient {
public:
  GameModeController() : SocketClient() {
  }
  GameModeController(const std::string& address, uint16_t port) : SocketClient(address, port) {
  }

  /* drones //{ */

  std::pair<bool, std::vector<int>> GetDrones();

  std::pair<bool, int> SpawnDrone();

  std::pair<bool, int> SpawnDroneAtLocation(ueds_connector::Coordinates &Location, std::string &TypeUav);

  bool RemoveDrone(const int port);

  //}

  /* session + rendering //{ */

  std::pair<bool, CameraCaptureModeEnum> GetCameraCaptureMode();

  bool SetCameraCaptureMode(const CameraCaptureModeEnum& cameraCaptureMode);

  std::pair<bool, float> GetFps();

  std::pair<bool, std::pair<int,int>> GetApiVersion();

  std::pair<bool, double> GetTime();
  
  bool SetGraphicsSettings(const int& graphicsSettings);

  //}

  /* world //{ */

  bool SwitchWorldLevel(const std::string &worldLevelName);

  bool SetForestDensity(const int DensityLevel);

  bool SetForestHillyLevel(const int HillyLevel);

  std::pair<bool, ueds_connector::Coordinates> GetWorldOrigin();

  bool SetWeather(const int& type_id);

  bool SetDatetime(const int& hour, const int& minute);

  bool SetMutualDroneVisibility(const bool& enabled);

  //}

  /* scene editor //{ */

  /**
   * Places objects described by a spawn configuration document. Assets already cooked into
   * the simulator are used as-is; those that are not are loaded from disk on the machine
   * running the simulator, which is what allows a packaged binary to be extended.
   *
   * Returns the handles of the objects that were placed, for RemoveSpawnedObject. On
   * failure the second element of the pair is empty and the error is written to OutError.
   */
  std::pair<bool, std::vector<int>> SpawnObjectsFromYaml(const std::string& yaml, std::string& OutError);

  /** Removes one previously spawned object. */
  std::pair<bool, int> RemoveSpawnedObject(const int object_id);

  /** Removes every object placed by the spawn tool, returning how many were removed. */
  std::pair<bool, int> RemoveAllSpawnedObjects();

  /**
   * Repositions a spawned object. The pose is in the spawn config's frame: right-handed
   * (ROS) metres and degrees (roll, pitch, yaw) relative to the world origin.
   */
  bool MoveSpawnedObject(const int object_id, const std::array<double, 3>& position, const std::array<double, 3>& orientation,
                         const std::array<double, 3>& scale = {1.0, 1.0, 1.0});

  /** Lists every spawned object with its CURRENT pose, in the same frame MoveSpawnedObject takes. */
  std::pair<bool, std::vector<Serializable::GameMode::SpawnedObjectInfo>> ListSpawnedObjects();

  /**
   * Serializes the currently spawned objects back into a spawn configuration document
   * (stencils included), so feeding it to SpawnObjectsFromYaml recreates the scene.
   */
  std::pair<bool, std::string> ExportScene();

  /* Triangle soup of the world's static geometry in the scene frame (metres, right-handed). */
  bool GetWorldMesh(const std::array<double, 3>& center, const std::array<double, 3>& extent, uint32_t max_triangles, std::vector<float>& out_vertices,
                    std::vector<uint32_t>& out_indices, bool& out_truncated, int32_t& out_sources);

  /* Exact distances [m] to the nearest collision surface for packed xyz scene-frame points. Blocks the sim's game thread. */
  bool GetDistanceBatch(const std::vector<float>& query_points, double max_radius, double tolerance, bool terrain_only, std::vector<float>& out_distances);

  /**
   * Enumerates what the spawn tool could place: cooked assets under path_prefix (empty
   * means /Game) plus on-disk glTF models, optionally filtered by a case-insensitive
   * substring of the name. OutTruncated is set when the server-side cap was hit.
   */
  std::pair<bool, std::vector<std::string>> ListAssets(const std::string& path_prefix, const std::string& name_filter, bool& OutTruncated);
  //}
};

}  // namespace ueds_connector
