#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <flight_forge_connector/connection.h>
#include <flight_forge_connector/data_types.h>

namespace flightforge
{

// simulator-wide ops; poses in the ground frame
class SimulatorClient {
public:
  static constexpr uint16_t kApiMajor = 1;
  static constexpr uint16_t kApiMinor = 1;

  SimulatorClient(const std::string& address = "127.0.0.1", uint16_t port = Connection::kDefaultPort);
  explicit SimulatorClient(std::shared_ptr<Connection> connection);

  bool Connect(int attempts = 1, std::chrono::milliseconds retry_delay = std::chrono::milliseconds(500)) {
    return connection_->Connect(attempts, retry_delay);
  }
  void Disconnect() {
    connection_->Disconnect();
  }
  const std::shared_ptr<Connection>& SharedConnection() const {
    return connection_;
  }

  // how long Step may run before the call gives up
  void SetStepTimeout(std::chrono::milliseconds timeout) {
    step_timeout_ = timeout;
  }

  /* connection //{ */

  Status           Ping();
  Expected<ApiInfo> GetInfo();

  /* //} */

  /* time //{ */

  Expected<TimeInfo> GetTime();
  Expected<float>    GetFps();

  // returns what the engine applied after its clamp
  Expected<double> SetTimeDilation(double dilation);

  // dt 0 follows the wall clock; adaptive_min_dt > 0 lets an event camera shrink the step
  Expected<double> SetFixedTimestep(double dt, double adaptive_min_dt = 0.0);

  // lockstep parks the world paused; Step runs ticks and returns once every camera has its frames
  Status               SetLockstep(bool enabled);
  Expected<StepResult> Step(uint32_t ticks = 1);

  /* //} */

  /* world //{ */

  Status SwitchWorld(const std::string& level);
  // a world without a sky or weather system fails with an error containing "has no sky system"
  Status SetWeather(wire::Weather weather);
  Status SetDaytime(int hour, int minute);
  Status SetGraphicsSettings(wire::GraphicsQuality quality);
  Status SetForestDensity(int level);
  Status SetForestHillyLevel(int level);
  Status SetMutualVisibility(bool enabled);

  Expected<wire::CaptureMode> GetCaptureMode();
  Status                      SetCaptureMode(wire::CaptureMode mode);

  // the ground frame's origin in Unreal cm, for aligning with editor coordinates
  Expected<Vec3> GetWorldOrigin();

  Expected<wire::GetClassMapResponseT> GetClassMap();

  /* //} */

  /* scene //{ */

  Expected<std::vector<int>> SpawnObjectsFromYaml(const std::string& yaml);
  Expected<int>              RemoveSpawnedObject(int object_id);
  Expected<int>              RemoveAllSpawnedObjects();
  Status                     MoveSpawnedObject(int object_id, const Pose& pose, const Vec3& scale = Vec3{1.0, 1.0, 1.0});

  Expected<std::vector<SpawnedObject>> ListSpawnedObjects();
  Expected<std::string>                ExportScene();

  Expected<AssetList>   ListAssets(const std::string& path_prefix = "", const std::string& name_filter = "");
  Expected<int>         LoadLibrary(const std::string& path);
  Expected<LibraryPage> ListLibrary(const std::string& library = "", const std::string& object_class = "", const std::string& filter = "",
                                    uint32_t offset = 0, uint32_t limit = 200);

  // any extent <= 0 takes the whole world
  Expected<WorldMesh> GetWorldMesh(const Vec3& center = {}, const Vec3& extent = {}, uint32_t max_triangles = 0);

  Expected<std::vector<float>> GetDistanceBatch(const std::vector<std::array<float, 3>>& points, double max_radius = 5.0, double tolerance = 0.001,
                                                bool terrain_only = false);

  // m, of the first surface below each point, NaN where there is none; a NaN z traces from the top of the world
  Expected<std::vector<double>> GetSurfaceHeight(const std::vector<Vec3>& points, bool ignore_robots = true);

  /* //} */

  /* robots //{ */

  Expected<int>                    SpawnRobot(const std::string& name, const Pose& pose, const std::string& label = "");
  Expected<std::vector<RobotInfo>> ListRobots();
  Status                           RemoveRobot(int robot_id);

  // many robots in one message; results are empty unless wait_applied
  Expected<std::vector<SetStateResult>> SetStates(const std::vector<std::pair<int, RobotState>>& states, bool collide = false, bool wait_applied = true);

  /* //} */

private:
  std::shared_ptr<Connection> connection_;
  std::chrono::milliseconds   step_timeout_{120000};
};

}  // namespace flightforge
