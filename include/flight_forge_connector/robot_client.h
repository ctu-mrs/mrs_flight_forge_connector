#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <flight_forge_connector/connection.h>
#include <flight_forge_connector/data_types.h>

namespace flightforge
{

// a sensor_id of -1 addresses the robot's default sensor of the op's type, created on first use
class RobotClient {
public:
  RobotClient(std::shared_ptr<Connection> connection, int robot_id);
  RobotClient(const std::string& address, uint16_t port, int robot_id);

  bool Connect(int attempts = 1, std::chrono::milliseconds retry_delay = std::chrono::milliseconds(500)) {
    return connection_->Connect(attempts, retry_delay);
  }
  void Disconnect() {
    connection_->Disconnect();
  }
  const std::shared_ptr<Connection>& SharedConnection() const {
    return connection_;
  }
  int Id() const {
    return robot_id_;
  }

  /* state //{ */

  Expected<StampedState> GetState();

  // returns once the state is applied, with where the base ended up
  Expected<SetStateResult> SetState(const RobotState& state, bool collide = false);

  // returns before the state is applied, for pose streams
  Status SetStateAsync(const RobotState& state, bool collide = false);

  Expected<SetStateResult> SetPose(const Pose& pose, bool collide = false);
  Status                   SetPoseAsync(const Pose& pose, bool collide = false);

  // rad/s per joint, GetDescription's joint order; empty restores the default spin
  Status SetJointVelocities(const std::vector<double>& velocities);

  // the base follows the samples, interpolated each tick, and holds the last; empty releases it
  Status SetTrajectory(const std::vector<TrajectorySample>& samples);

  Expected<wire::GetDescriptionResponseT> GetDescription();
  Expected<CollisionState>                GetCollision();

  Expected<bool> GetMoveLineVisible();
  Status         SetMoveLineVisible(bool visible);

  /* //} */

  /* sensors and devices //{ */

  // pose is relative to the link (empty is the base link) or the named mount
  Expected<int> AddSensor(wire::SensorType type, const std::optional<Pose>& pose = std::nullopt, const std::string& link = "", const std::string& mount = "");
  Status        RemoveSensor(int sensor_id);
  Expected<std::vector<SensorInfo>> ListSensors();

  Expected<std::vector<SensorInfo>> AddDevice(const std::string& device_name, const std::optional<Pose>& pose = std::nullopt, bool show_mesh = false,
                                              const std::string& link = "", const std::string& mount = "");
  Expected<std::vector<DeviceInfo>> ListDevices();

  // -1 lists every camera the robot has
  Expected<std::vector<wire::CameraInfoT>> GetCameraInfo(int sensor_id = -1);

  /* //} */

  /* configs //{ */

  // also the depth, instance and legacy segmentation cameras' config
  Expected<wire::RgbCameraConfigT> GetRgbCameraConfig(int sensor_id = -1);
  Status                           SetRgbCameraConfig(const wire::RgbCameraConfigT& config, int sensor_id = -1);

  Expected<wire::StereoCameraConfigT> GetStereoCameraConfig(int sensor_id = -1);
  Status                              SetStereoCameraConfig(const wire::StereoCameraConfigT& config, int sensor_id = -1);

  Expected<wire::FisheyeCameraConfigT> GetFisheyeCameraConfig(int sensor_id = -1);
  Status                               SetFisheyeCameraConfig(const wire::FisheyeCameraConfigT& config, int sensor_id = -1);

  Expected<wire::EventCameraConfigT> GetEventCameraConfig(int sensor_id = -1);
  Status                             SetEventCameraConfig(const wire::EventCameraConfigT& config, int sensor_id = -1);

  Expected<wire::LidarConfigT> GetLidarConfig(int sensor_id = -1);
  Status                       SetLidarConfig(const wire::LidarConfigT& config, int sensor_id = -1);

  /* //} */

  /* data //{ */

  Expected<Image>       GetRgbCameraData(int sensor_id = -1);
  Expected<StereoImage> GetStereoCameraData(int sensor_id = -1);
  Expected<Image>       GetFisheyeCameraData(int sensor_id = -1);
  Expected<DepthImage>  GetDepthCameraData(int sensor_id = -1);
  Expected<Image>       GetRgbSegCameraData(int sensor_id = -1);

  // RGBA8: instance id = R + (G << 8) + (B << 16), A the class id
  Expected<Image>                                  GetInstanceSegData(int sensor_id = -1);
  Expected<std::vector<wire::InstanceSegEntryT>> GetInstanceSegMap(int sensor_id = -1);

  // drains the events since the previous call
  Expected<CameraEvents> GetEventCameraData(int sensor_id = -1);

  Expected<LidarScan>          GetLidarScan(bool with_intensity = false, bool with_labels = false, int sensor_id = -1);
  Expected<RangefinderReading> GetRangefinderData(int sensor_id = -1);

  /* //} */

private:
  std::optional<int32_t> Sensor(int sensor_id) const;

  std::shared_ptr<Connection> connection_;
  int                         robot_id_;
};

}  // namespace flightforge
