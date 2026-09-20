#pragma once

#include <string>
#include <vector>

#include <flight_forge_connector/data_types.h>
#include <flight_forge_connector/socket_client.h>
#include <flight_forge_connector/serialization/serializable_shared.h>

namespace ueds_connector
{

// Drone-side client. Since API 0.14 every sensor call takes a sensor id;
// the default -1 addresses the default sensor of that type, which the
// simulator creates lazily on first use. Additional sensors come from
// AddSensor/AddDevice and are addressed by the returned ids.
class UedsConnector : public SocketClient {

public:
  UedsConnector() : SocketClient() {
    scope_ = Serializable::Registry::Scope::Drone;
  }

  UedsConnector(const std::string& address, uint16_t port) : SocketClient(address, port) {
    scope_ = Serializable::Registry::Scope::Drone;
  }

  /* pose + state //{ */

  std::pair<bool, Coordinates> GetLocation();

  std::pair<bool, bool> GetCrashState();

  std::tuple<bool, Coordinates, bool, Coordinates> SetLocation(const Coordinates& coordinates, bool checkCollisions);

  std::pair<bool, Rotation> GetRotation();

  std::tuple<bool, Rotation, bool, Coordinates> SetRotation(const Rotation& rotation);

  std::tuple<bool, Coordinates, Rotation, bool, Coordinates> SetLocationAndRotation(const Coordinates& coordinate, const Rotation& rotation,
                                                                                    const bool should_collide);

  std::tuple<bool> SetLocationAndRotationAsync(const Coordinates& coordinate, const Rotation& rotation, const bool should_collide);

  // a pose timeline the pawn interpolates along per tick; replaces the previous one, empty detaches
  bool SetPoseTrajectory(const std::vector<PoseSample>& samples);

  /**
   * Angular rate per propeller [rad/s], in the frame's propeller order: rear-left,
   * rear-right, front-left, front-right, then any an external model added. Sign gives
   * the direction of turn. Fewer values than the frame has leaves the rest untouched;
   * an empty list hands them back to their default spin. out_count reports how many
   * propellers the frame actually has.
   */
  bool SetPropellerSpeeds(const std::vector<double>& speeds, int32_t& out_count);

  std::pair<bool, bool> GetMoveLineVisible();

  bool SetMoveLineVisible(bool visible);

  //}

  /* cameras //{ */

  std::tuple<bool, std::vector<unsigned char>, double, uint32_t> GetRgbCameraData(int sensorId = -1);

  std::tuple<bool, std::vector<unsigned char>, std::vector<unsigned char>, double> GetStereoCameraData(int sensorId = -1);

  std::tuple<bool, std::vector<unsigned char>, double, uint32_t> GetRgbSegmented(int sensorId = -1);

  std::pair<bool, RgbCameraConfig> GetRgbCameraConfig(int sensorId = -1);

  bool SetRgbCameraConfig(const RgbCameraConfig& config, int sensorId = -1);

  // stereo config passes the wire struct through; independent left/right eye
  // poses since API 0.12 (the fixed baseline is gone)
  std::pair<bool, Serializable::Drone::StereoCameraConfig> GetStereoCameraConfig(int sensorId = -1);

  bool SetStereoCameraConfig(const Serializable::Drone::StereoCameraConfig& config, int sensorId = -1);

  //}

  /* event camera //{ */

  // drains the events since the previous call, time-sorted; stamp is the sim time the batch is complete up to
  std::pair<bool, CameraEvents> GetEventCameraData(int sensorId = -1);

  std::pair<bool, EventCameraConfig> GetEventCameraConfig(int sensorId = -1);

  bool SetEventCameraConfig(const EventCameraConfig& config, int sensorId = -1);

  //}

  /* fisheye camera //{ */

  std::tuple<bool, std::vector<unsigned char>, double, uint32_t> GetFisheyeCameraData(int sensorId = -1);

  std::pair<bool, FisheyeCameraConfig> GetFisheyeCameraConfig(int sensorId = -1);

  bool SetFisheyeCameraConfig(const FisheyeCameraConfig& config, int sensorId = -1);

  //}

  /* lidar + rangefinder //{ */

  std::tuple<bool, double> GetRangefinderData(int sensorId = -1);

  // Livox mode returns slices sized by elapsed time rather than the beam grid;
  // the stamp is taken at scan start.
  std::tuple<bool, std::vector<LidarData>, Coordinates, double> GetLidarData(int sensorId = -1);

  std::tuple<bool, std::vector<LidarSegData>, Coordinates, double> GetLidarSegData(int sensorId = -1);

  std::tuple<bool, std::vector<LidarIntData>, Coordinates, double> GetLidarIntData(int sensorId = -1);

  std::pair<bool, LidarConfig> GetLidarConfig(int sensorId = -1);

  bool SetLidarConfig(const LidarConfig& config, int sensorId = -1);

  //}

  /* sensor + device management //{ */

  // Creates a sensor of the given SensorTypeEnum; returns its id.
  std::pair<bool, int> AddSensor(int sensorType);

  bool RemoveSensor(int sensorId);

  std::pair<bool, std::vector<SensorInfo>> ListSensors();

  // Instantiates a preconfigured real-world device (see ListDevices) at a
  // mount pose; a device may create several sensors (e.g. a RealSense D435i
  // is a stereo pair plus an RGB camera). showMesh attaches a housing-sized
  // placeholder mesh at the mount pose for visual debugging.
  std::pair<bool, std::vector<SensorInfo>> AddDevice(const std::string& deviceName, const Coordinates& offset, const Rotation& rotation, bool showMesh);

  std::pair<bool, std::vector<DeviceInfo>> ListDevices();

  /**
   * The instance-segmentation image: each pixel is a 24-bit per-actor id
   * (decode as r + (g<<8) + (b<<16); 0 = background), rendered by a dedicated
   * mesh pass with correct occlusion and no 255-instance stencil cap.
   */
  bool GetInstanceSegData(std::vector<unsigned char>& image, double& stamp, int sensorId = -1);

  /** The id -> actor-path map snapshotted with the last instance-segmentation frame. */
  std::pair<bool, std::vector<Serializable::Drone::InstanceSegMapEntry>> GetInstanceSegMap(int sensorId = -1);

  /**
   * Depth image in uint16 millimetres (0 invalid, 65535 clamp - the
   * Kinect/RealSense convention), row-major width x height, uncompressed.
   */
  bool GetDepthCameraData(std::vector<uint16_t>& image, int& width, int& height, double& stamp, int sensorId = -1);

  //}
};

}  // namespace ueds_connector
