#include <flight_forge_connector/robot_client.h>

#include <flight_forge_connector/wire_convert.h>

#include "reply_helpers.h"

namespace flightforge
{

using FBB = flatbuffers::FlatBufferBuilder;

namespace
{

std::vector<SensorInfo> ReadSensors(const flatbuffers::Vector<flatbuffers::Offset<wire::SensorInfo>>* sensors) {
  std::vector<SensorInfo> out;
  if (sensors) {
    for (const wire::SensorInfo* sensor : *sensors) {
      out.push_back(SensorInfo{sensor->id(), sensor->type(), sensor->link() ? sensor->link()->str() : std::string()});
    }
  }
  return out;
}

}  // namespace

RobotClient::RobotClient(std::shared_ptr<Connection> connection, const int robot_id) : connection_(std::move(connection)), robot_id_(robot_id) {
}

RobotClient::RobotClient(const std::string& address, const uint16_t port, const int robot_id)
    : connection_(std::make_shared<Connection>(address, port)), robot_id_(robot_id) {
}

std::optional<int32_t> RobotClient::Sensor(const int sensor_id) const {
  return sensor_id >= 0 ? std::optional<int32_t>(sensor_id) : std::nullopt;
}

/* state //{ */

Expected<StampedState> RobotClient::GetState() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetStateRequest(b), robot_id_);
  Status     status;
  const auto body = Body<wire::GetStateResponse>(reply, status);
  if (!body) {
    return status;
  }
  return StampedState{body->stamp(), ReadState(body->state())};
}

Expected<SetStateResult> RobotClient::SetState(const RobotState& state, const bool collide) {
  FBB        b;
  const auto wire_state = BuildState(b, state);
  const auto reply      = connection_->Call(b, wire::CreateSetStateRequest(b, wire_state, collide, true), robot_id_);
  Status     status;
  const auto body = Body<wire::SetStateResponse>(reply, status);
  if (!body) {
    return status;
  }
  return ReadStateResult(body);
}

Status RobotClient::SetStateAsync(const RobotState& state, const bool collide) {
  FBB        b;
  const auto wire_state = BuildState(b, state);
  return AsStatus(connection_->Call(b, wire::CreateSetStateRequest(b, wire_state, collide, false), robot_id_));
}

Expected<SetStateResult> RobotClient::SetPose(const Pose& pose, const bool collide) {
  RobotState state;
  state.base = pose;
  return SetState(state, collide);
}

Status RobotClient::SetPoseAsync(const Pose& pose, const bool collide) {
  RobotState state;
  state.base = pose;
  return SetStateAsync(state, collide);
}

Status RobotClient::SetJointVelocities(const std::vector<double>& velocities) {
  FBB        b;
  const auto speeds = b.CreateVector(velocities);
  const auto state  = wire::CreateRobotState(b, nullptr, 0, speeds);
  return AsStatus(connection_->Call(b, wire::CreateSetStateRequest(b, state, false, true), robot_id_));
}

Status RobotClient::SetTrajectory(const std::vector<TrajectorySample>& samples) {
  FBB                                                       b;
  std::vector<flatbuffers::Offset<wire::TrajectorySample>> entries;
  for (const TrajectorySample& sample : samples) {
    const wire::Pose base  = ToWire(sample.base);
    const auto       state = wire::CreateRobotState(b, &base);
    entries.push_back(wire::CreateTrajectorySample(b, sample.t, state));
  }
  const auto list = b.CreateVector(entries);
  return AsStatus(connection_->Call(b, wire::CreateSetTrajectoryRequest(b, list), robot_id_));
}

Expected<wire::GetDescriptionResponseT> RobotClient::GetDescription() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetDescriptionRequest(b), robot_id_);
  Status     status;
  const auto body = Body<wire::GetDescriptionResponse>(reply, status);
  if (!body) {
    return status;
  }
  wire::GetDescriptionResponseT out;
  body->UnPackTo(&out);
  return out;
}

Expected<CollisionState> RobotClient::GetCollision() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetCollisionRequest(b), robot_id_);
  Status     status;
  const auto body = Body<wire::GetCollisionResponse>(reply, status);
  if (!body) {
    return status;
  }
  return CollisionState{body->collided(), body->impact_point() ? FromWire(*body->impact_point()) : Vec3{}};
}

Expected<bool> RobotClient::GetMoveLineVisible() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetMoveLineVisibleRequest(b), robot_id_);
  Status     status;
  const auto body = Body<wire::GetMoveLineVisibleResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->visible();
}

Status RobotClient::SetMoveLineVisible(const bool visible) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateSetMoveLineVisibleRequest(b, visible), robot_id_));
}

/* //} */

/* sensors and devices //{ */

Expected<int> RobotClient::AddSensor(const wire::SensorType type, const std::optional<Pose>& pose, const std::string& link, const std::string& mount) {
  FBB                             b;
  const auto                      link_name  = b.CreateString(link);
  const auto                      mount_name = b.CreateString(mount);
  const std::optional<wire::Pose> p          = pose ? std::optional<wire::Pose>(ToWire(*pose)) : std::nullopt;
  const auto reply = connection_->Call(b, wire::CreateAddSensorRequest(b, type, link_name, mount_name, p ? &*p : nullptr), robot_id_);
  Status     status;
  const auto body = Body<wire::AddSensorResponse>(reply, status);
  if (!body) {
    return status;
  }
  return body->sensor_id();
}

Status RobotClient::RemoveSensor(const int sensor_id) {
  FBB b;
  return AsStatus(connection_->Call(b, wire::CreateRemoveSensorRequest(b), robot_id_, sensor_id));
}

Expected<std::vector<SensorInfo>> RobotClient::ListSensors() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateListSensorsRequest(b), robot_id_);
  Status     status;
  const auto body = Body<wire::ListSensorsResponse>(reply, status);
  if (!body) {
    return status;
  }
  return ReadSensors(body->sensors());
}

Expected<std::vector<SensorInfo>> RobotClient::AddDevice(const std::string& device_name, const std::optional<Pose>& pose, const bool show_mesh,
                                                         const std::string& link, const std::string& mount) {
  FBB                             b;
  const auto                      name       = b.CreateString(device_name);
  const auto                      link_name  = b.CreateString(link);
  const auto                      mount_name = b.CreateString(mount);
  const std::optional<wire::Pose> p          = pose ? std::optional<wire::Pose>(ToWire(*pose)) : std::nullopt;
  const auto reply = connection_->Call(b, wire::CreateAddDeviceRequest(b, name, link_name, mount_name, p ? &*p : nullptr, show_mesh), robot_id_);
  Status     status;
  const auto body = Body<wire::AddDeviceResponse>(reply, status);
  if (!body) {
    return status;
  }
  return ReadSensors(body->sensors());
}

Expected<std::vector<DeviceInfo>> RobotClient::ListDevices() {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateListDevicesRequest(b), robot_id_);
  Status     status;
  const auto body = Body<wire::ListDevicesResponse>(reply, status);
  if (!body) {
    return status;
  }
  std::vector<DeviceInfo> devices;
  if (body->devices()) {
    for (const wire::DeviceInfo* device : *body->devices()) {
      devices.push_back(DeviceInfo{device->name() ? device->name()->str() : std::string(), device->description() ? device->description()->str() : std::string()});
    }
  }
  return devices;
}

Expected<std::vector<wire::CameraInfoT>> RobotClient::GetCameraInfo(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetCameraInfoRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::GetCameraInfoResponse>(reply, status);
  if (!body) {
    return status;
  }
  std::vector<wire::CameraInfoT> cameras;
  if (body->cameras()) {
    for (const wire::CameraInfo* camera : *body->cameras()) {
      cameras.emplace_back();
      camera->UnPackTo(&cameras.back());
    }
  }
  return cameras;
}

/* //} */

/* configs //{ */

#define FF_CONFIG_OPS(Kind)                                                                                                   \
  Expected<wire::Kind##ConfigT> RobotClient::Get##Kind##Config(const int sensor_id) {                                         \
    FBB        b;                                                                                                             \
    const auto reply = connection_->Call(b, wire::CreateGet##Kind##ConfigRequest(b), robot_id_, Sensor(sensor_id));           \
    Status     status;                                                                                                        \
    const auto body = Body<wire::Get##Kind##ConfigResponse>(reply, status);                                                   \
    if (!body) {                                                                                                              \
      return status;                                                                                                          \
    }                                                                                                                         \
    if (!body->config()) {                                                                                                    \
      return Status::Fail("the reply carries no config");                                                                     \
    }                                                                                                                         \
    wire::Kind##ConfigT config;                                                                                               \
    body->config()->UnPackTo(&config);                                                                                        \
    return config;                                                                                                            \
  }                                                                                                                           \
                                                                                                                              \
  Status RobotClient::Set##Kind##Config(const wire::Kind##ConfigT& config, const int sensor_id) {                             \
    FBB        b;                                                                                                             \
    const auto table = wire::Create##Kind##Config(b, &config);                                                                \
    return AsStatus(connection_->Call(b, wire::CreateSet##Kind##ConfigRequest(b, table), robot_id_, Sensor(sensor_id)));      \
  }

FF_CONFIG_OPS(RgbCamera)
FF_CONFIG_OPS(StereoCamera)
FF_CONFIG_OPS(FisheyeCamera)
FF_CONFIG_OPS(EventCamera)
FF_CONFIG_OPS(Lidar)

#undef FF_CONFIG_OPS

/* //} */

/* data //{ */

Expected<Image> RobotClient::GetRgbCameraData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetRgbCameraDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::RgbCameraDataResponse>(reply, status);
  if (!body) {
    return status;
  }
  return ReadImage(body->image(), body->stamp());
}

Expected<StereoImage> RobotClient::GetStereoCameraData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetStereoCameraDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::StereoCameraDataResponse>(reply, status);
  if (!body) {
    return status;
  }
  return StereoImage{body->stamp(), ReadImage(body->left(), body->stamp()), ReadImage(body->right(), body->stamp())};
}

Expected<Image> RobotClient::GetFisheyeCameraData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetFisheyeCameraDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::FisheyeCameraDataResponse>(reply, status);
  if (!body) {
    return status;
  }
  return ReadImage(body->image(), body->stamp());
}

Expected<DepthImage> RobotClient::GetDepthCameraData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetDepthCameraDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::DepthCameraDataResponse>(reply, status);
  if (!body) {
    return status;
  }

  DepthImage depth;
  depth.stamp = body->stamp();
  if (const wire::Image* image = body->image()) {
    depth.width  = image->width();
    depth.height = image->height();
    if (image->data()) {
      depth.mm.resize(image->data()->size() / sizeof(uint16_t));
      std::memcpy(depth.mm.data(), image->data()->Data(), depth.mm.size() * sizeof(uint16_t));
    }
  }
  return depth;
}

Expected<Image> RobotClient::GetRgbSegCameraData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetRgbSegCameraDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::RgbSegCameraDataResponse>(reply, status);
  if (!body) {
    return status;
  }
  return ReadImage(body->image(), body->stamp());
}

Expected<Image> RobotClient::GetInstanceSegData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetInstanceSegDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::InstanceSegDataResponse>(reply, status);
  if (!body) {
    return status;
  }
  return ReadImage(body->image(), body->stamp());
}

Expected<std::vector<wire::InstanceSegEntryT>> RobotClient::GetInstanceSegMap(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetInstanceSegMapRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::InstanceSegMapResponse>(reply, status);
  if (!body) {
    return status;
  }
  std::vector<wire::InstanceSegEntryT> entries;
  if (body->entries()) {
    for (const wire::InstanceSegEntry* entry : *body->entries()) {
      entries.emplace_back();
      entry->UnPackTo(&entries.back());
    }
  }
  return entries;
}

Expected<CameraEvents> RobotClient::GetEventCameraData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetEventCameraDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::EventCameraDataResponse>(reply, status);
  if (!body) {
    return status;
  }

  CameraEvents events;
  events.base_stamp = body->base_stamp();
  events.stamp      = body->stamp();
  events.dropped    = body->dropped();
  events.width      = body->width();
  events.height     = body->height();
  if (body->xyp()) {
    events.xyp.assign(body->xyp()->begin(), body->xyp()->end());
  }
  if (body->t_us()) {
    events.t_us.assign(body->t_us()->begin(), body->t_us()->end());
  }
  return events;
}

Expected<LidarScan> RobotClient::GetLidarScan(const bool with_intensity, const bool with_labels, const int sensor_id) {
  static_assert(sizeof(std::array<float, 3>) == sizeof(wire::Vec3f));

  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetLidarScanRequest(b, with_intensity, with_labels), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::LidarScanResponse>(reply, status);
  if (!body) {
    return status;
  }

  LidarScan scan;
  scan.stamp = body->stamp();
  scan.rows  = body->rows();
  scan.cols  = body->cols();
  if (body->sensor_pose()) {
    scan.sensor_pose = FromWire(*body->sensor_pose());
  }
  if (body->points()) {
    scan.points.resize(body->points()->size());
    std::memcpy(scan.points.data(), body->points()->Data(), scan.points.size() * sizeof(wire::Vec3f));
  }
  if (body->intensity()) {
    scan.intensity.assign(body->intensity()->begin(), body->intensity()->end());
  }
  if (body->class_id()) {
    scan.class_id.assign(body->class_id()->begin(), body->class_id()->end());
  }
  if (body->instance_id()) {
    scan.instance_id.assign(body->instance_id()->begin(), body->instance_id()->end());
  }
  return scan;
}

Expected<RangefinderReading> RobotClient::GetRangefinderData(const int sensor_id) {
  FBB        b;
  const auto reply = connection_->Call(b, wire::CreateGetRangefinderDataRequest(b), robot_id_, Sensor(sensor_id));
  Status     status;
  const auto body = Body<wire::RangefinderDataResponse>(reply, status);
  if (!body) {
    return status;
  }
  return RangefinderReading{body->stamp(), body->range()};
}

/* //} */

}  // namespace flightforge
