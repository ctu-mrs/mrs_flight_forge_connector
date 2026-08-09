// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#include <flight_forge_connector/flight_forge_connector.h>

#include <sstream>

using kissnet::socket_status;
using ueds_connector::Coordinates;
using ueds_connector::LidarConfig;
using ueds_connector::LidarData;
using ueds_connector::LidarIntData;
using ueds_connector::LidarSegData;
using ueds_connector::RgbCameraConfig;
using ueds_connector::Rotation;
using ueds_connector::CameraEvent;
using ueds_connector::DeviceInfo;
using ueds_connector::EventCameraConfig;
using ueds_connector::FisheyeCameraConfig;
using ueds_connector::SensorInfo;
using ueds_connector::UedsConnector;

/* getLocation() //{ */

std::pair<bool, Coordinates> UedsConnector::GetLocation() {

  Serializable::Drone::GetLocation::Request request{};

  Serializable::Drone::GetLocation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Coordinates coordinates{};

  if (success) {
    coordinates.x = response.x;
    coordinates.y = response.y;
    coordinates.z = response.z;
  }

  return std::make_pair(success, coordinates);
}

//}

/* getCrashState() //{ */

std::pair<bool, bool> UedsConnector::GetCrashState() {

  Serializable::Drone::GetCrashState::Request request{};

  Serializable::Drone::GetCrashState::Response response{};
  const auto                                   status  = Request(request, response);
  const auto                                   success = status && response.status;

  bool crashed = response.crashed;

  return std::make_pair(success, crashed);
}

//}

/* setLocation() //{ */

std::tuple<bool, Coordinates, bool, Coordinates> UedsConnector::SetLocation(const Coordinates& coordinate, bool checkCollisions) {

  Serializable::Drone::SetLocation::Request request{};

  request.x               = coordinate.x;
  request.y               = coordinate.y;
  request.z               = coordinate.z;
  request.checkCollisions = checkCollisions;

  Serializable::Drone::SetLocation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Coordinates teleportedTo{};
  Coordinates impactPoint{};
  if (success) {
    teleportedTo.x = response.teleportedToX;
    teleportedTo.y = response.teleportedToY;
    teleportedTo.z = response.teleportedToZ;
    impactPoint.x  = response.impactPointX;
    impactPoint.y  = response.impactPointY;
    impactPoint.z  = response.impactPointZ;
  }

  return std::make_tuple(success, teleportedTo, response.isHit, impactPoint);
}

//}

/* GetRgbCameraData() //{ */

std::tuple<bool, std::vector<unsigned char>, double, uint32_t> UedsConnector::GetRgbCameraData(int sensorId) {

  Serializable::Drone::GetRgbCameraData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetRgbCameraData::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return std::make_tuple(success, success ? response.image_ : std::vector<unsigned char>(), success ? response.stamp_ : 0.0,
                         success ? response.image_.size() : 0);
}

//}

/* GetStereoCameraData() //{ */

std::tuple<bool, std::vector<unsigned char>, std::vector<unsigned char>, double> UedsConnector::GetStereoCameraData(int sensorId) {

  Serializable::Drone::GetStereoCameraData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetStereoCameraData::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return std::make_tuple(success, success ? response.image_left_ : std::vector<unsigned char>(), success ? response.image_right_ : std::vector<unsigned char>(),
                         success ? response.stamp_ : 0.0);
}

//}

/* GetRgbSegmented() //{ */

std::tuple<bool, std::vector<unsigned char>, double, uint32_t> UedsConnector::GetRgbSegmented(int sensorId) {

  Serializable::Drone::GetRgbSegCameraData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetRgbSegCameraData::Response response{};
  const auto                                         status  = Request(request, response);
  const auto                                         success = status && response.status;

  return std::make_tuple(success, success ? response.image_ : std::vector<unsigned char>(), success ? response.stamp_ : 0.0,
                         success ? response.image_.size() : 0);
}

//}

/* getRotation() //{ */

std::pair<bool, Rotation> UedsConnector::GetRotation() {

  Serializable::Drone::GetRotation::Request request{};

  Serializable::Drone::GetRotation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Rotation rotation{};
  if (success) {
    rotation.pitch = response.pitch;
    rotation.yaw   = response.yaw;
    rotation.roll  = response.roll;
  }

  return std::make_pair(success, rotation);
}

//}

/* setRotation() //{ */

std::tuple<bool, Rotation, bool, Coordinates> UedsConnector::SetRotation(const Rotation& rotation) {

  Serializable::Drone::SetRotation::Request request{};
  request.pitch = rotation.pitch;
  request.yaw   = rotation.yaw;
  request.roll  = rotation.roll;

  Serializable::Drone::SetRotation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Rotation    rotatedTo{};
  Coordinates impactPoint{};
  bool        isHit = false;
  if (success) {
    rotatedTo.pitch = response.rotatedToPitch;
    rotatedTo.yaw   = response.rotatedToYaw;
    rotatedTo.roll  = response.rotatedToRoll;
    isHit           = response.isHit;
    impactPoint.x   = response.impactPointX;
    impactPoint.y   = response.impactPointY;
    impactPoint.z   = response.impactPointZ;
  }

  return std::make_tuple(success, rotatedTo, isHit, impactPoint);
}

//}

/* setLocationAndRotation() //{ */

std::tuple<bool, Coordinates, Rotation, bool, Coordinates> UedsConnector::SetLocationAndRotation(const Coordinates& coordinate, const Rotation& rotation,
                                                                                                 const bool should_collide) {

  Serializable::Drone::SetLocationAndRotation::Request request{};

  request.x = coordinate.x;
  request.y = coordinate.y;
  request.z = coordinate.z;

  request.pitch = rotation.pitch;
  request.yaw   = rotation.yaw;
  request.roll  = rotation.roll;

  request.should_collide = should_collide;

  Serializable::Drone::SetLocationAndRotation::Response response{};
  const auto                                            status  = Request(request, response);
  const auto                                            success = status && response.status;

  Coordinates teleportedTo{};
  Rotation    rotatedTo{};
  Coordinates impactPoint{};
  bool        isHit = false;

  if (success) {
    teleportedTo.x  = response.teleportedToX;
    teleportedTo.y  = response.teleportedToY;
    teleportedTo.z  = response.teleportedToZ;
    rotatedTo.pitch = response.rotatedToPitch;
    rotatedTo.yaw   = response.rotatedToYaw;
    rotatedTo.roll  = response.rotatedToRoll;
    isHit           = response.isHit;
    impactPoint.x   = response.impactPointX;
    impactPoint.y   = response.impactPointY;
    impactPoint.z   = response.impactPointZ;
  }

  return std::make_tuple(success, teleportedTo, rotatedTo, isHit, impactPoint);
}

//}

/* setLocationAndRotationAsync() //{ */

std::tuple<bool> UedsConnector::SetLocationAndRotationAsync(const Coordinates& coordinate, const Rotation& rotation, const bool should_collide) {

  Serializable::Drone::SetLocationAndRotationAsync::Request request{};

  request.x = coordinate.x;
  request.y = coordinate.y;
  request.z = coordinate.z;

  request.pitch = rotation.pitch;
  request.yaw   = rotation.yaw;
  request.roll  = rotation.roll;

  request.should_collide = should_collide;

  Serializable::Drone::SetLocationAndRotationAsync::Response response{};
  const auto                                                 status  = Request(request, response);
  const auto                                                 success = status && response.status;

  return std::make_tuple(success);
}

//}

/* getLidarData() //{ */

std::tuple<bool, std::vector<LidarData>, Coordinates, double> UedsConnector::GetLidarData(int sensorId) {

  /* std::cout << "GetLidarData()" << std::endl; */
  Serializable::Drone::GetLidarData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetLidarData::Response response{};
  const auto                                  status  = Request(request, response);
  const auto                                  success = status && response.status;
  std::vector<LidarData>                      lidarData;
  Coordinates                                 start{};
  double stamp;

  if (success) {

    const auto arrSize = response.lidarData.size();
    lidarData.resize(arrSize);

    for (size_t i = 0; i < arrSize; i++) {
      lidarData[i]            = LidarData{};
      lidarData[i].distance   = response.lidarData[i].distance;
      lidarData[i].directionX = response.lidarData[i].directionX;
      lidarData[i].directionY = response.lidarData[i].directionY;
      lidarData[i].directionZ = response.lidarData[i].directionZ;
    }

    start.x = response.startX;
    start.y = response.startY;
    start.z = response.startZ;

    stamp = response.stamp_;
  }

  // std::cout << "Get lidar data drone controller: " << success << std::endl;
  return std::make_tuple(success, lidarData, start, stamp);
}

std::tuple<bool, double> ueds_connector::UedsConnector::GetRangefinderData(int sensorId)
{
  Serializable::Drone::GetRangefinderData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetRangefinderData::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  return std::make_pair(success, success ? response.range : -1);
} //}

/* getLidarSegData() //{ */

std::tuple<bool, std::vector<LidarSegData>, Coordinates, double> UedsConnector::GetLidarSegData(int sensorId) {
  Serializable::Drone::GetLidarSegData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetLidarSegData::Response response{};
  const auto                                     status  = Request(request, response);
  const auto                                     success = status && response.status;
  std::vector<LidarSegData>                      lidarSegData;
  Coordinates                                    start{};
  double stamp;

  if (success) {

    const auto arrSize = response.lidarSegData.size();
    lidarSegData.resize(arrSize);

    for (size_t i = 0; i < arrSize; i++) {
      lidarSegData[i]              = LidarSegData{};
      lidarSegData[i].distance     = response.lidarSegData[i].distance;
      lidarSegData[i].directionX   = response.lidarSegData[i].directionX;
      lidarSegData[i].directionY   = response.lidarSegData[i].directionY;
      lidarSegData[i].directionZ   = response.lidarSegData[i].directionZ;
      lidarSegData[i].segmentation = response.lidarSegData[i].segmentation;
    }

    start.x = response.startX;
    start.y = response.startY;
    start.z = response.startZ;
    stamp = response.stamp_;
  }

  // std::cout << "Get lidar data drone controller: " << success << std::endl;
  return std::make_tuple(success, lidarSegData, start, stamp);
}
//}

/* getLidarIntData() //{ */

std::tuple<bool, std::vector<LidarIntData>, Coordinates, double> UedsConnector::GetLidarIntData(int sensorId) {

  /* std::cout << "GetLidarIntData()" << std::endl; */

  Serializable::Drone::GetLidarIntData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetLidarIntData::Response response{};
  const auto                                     status  = Request(request, response);
  const auto                                     success = status && response.status;
  std::vector<LidarIntData>                      lidarIntData;
  Coordinates                                    start{};
  double stamp;

  if (success) {

    const auto arrSize = response.lidarIntData.size();
    lidarIntData.resize(arrSize);

    for (size_t i = 0; i < arrSize; i++) {
      lidarIntData[i]            = LidarIntData{};
      lidarIntData[i].distance   = response.lidarIntData[i].distance;
      lidarIntData[i].directionX = response.lidarIntData[i].directionX;
      lidarIntData[i].directionY = response.lidarIntData[i].directionY;
      lidarIntData[i].directionZ = response.lidarIntData[i].directionZ;
      lidarIntData[i].intensity  = response.lidarIntData[i].intensity;
    }

    start.x = response.startX;
    start.y = response.startY;
    start.z = response.startZ;
    stamp = response.stamp_;
  }

  return std::make_tuple(success, lidarIntData, start, stamp);
}
//}

/* getLidarConfig() //{ */

std::pair<bool, LidarConfig> UedsConnector::GetLidarConfig(int sensorId) {

  Serializable::Drone::GetLidarConfig::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetLidarConfig::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  /* bool success = true; */
  LidarConfig config{};

  if (success) {

    config.Enable       = response.config.Enable;
    config.showBeams    = response.config.ShowBeams;
    config.BeamHorRays  = response.config.BeamHorRays;
    config.BeamVertRays = response.config.BeamVertRays;
    config.beamLength   = response.config.BeamLength;
    config.Frequency    = response.config.Frequency;
    config.offset       = Coordinates{response.config.OffsetX, response.config.OffsetY, response.config.OffsetZ};

    config.orientation = Rotation{response.config.OrientationPitch, response.config.OrientationYaw, response.config.OrientationRoll};

    config.FOVHorLeft = response.config.FOVHorLeft;
    config.FOVHorRight = response.config.FOVHorRight;
    config.FOVVertUp = response.config.FOVVertUp;
    config.FOVVertDown = response.config.FOVVertDown;
    config.Livox       = response.config.Livox;
    config.LivoxSensor = response.config.LivoxSensor;
  }

  return std::make_pair(success, config);
}

//}

/* setLidarConfig() //{ */

bool UedsConnector::SetLidarConfig(const LidarConfig& config, int sensorId) {

  Serializable::Drone::SetLidarConfig::Request request{};
  request.sensor_id_ = sensorId;

  request.config              = Serializable::Drone::LidarConfig{};
  request.config.Enable       = config.Enable;
  request.config.ShowBeams    = config.showBeams;
  request.config.BeamHorRays  = config.BeamHorRays;
  request.config.BeamVertRays = config.BeamVertRays;
  request.config.BeamLength   = config.beamLength;
  request.config.Frequency    = config.Frequency;

  request.config.OffsetX = config.offset.x;
  request.config.OffsetY = config.offset.y;
  request.config.OffsetZ = config.offset.z;

  request.config.OrientationPitch = config.orientation.pitch;
  request.config.OrientationYaw   = config.orientation.yaw;
  request.config.OrientationRoll  = config.orientation.roll;

  /* request.config.FOVHor  = config.FOVHor; */
  /* request.config.FOVVert = config.FOVVert; */

  request.config.FOVHorLeft  = config.FOVHorLeft;
  request.config.FOVHorRight = config.FOVHorRight;
  request.config.FOVVertUp   = config.FOVVertUp;
  request.config.FOVVertDown = config.FOVVertDown;
  request.config.Livox       = config.Livox;
  request.config.LivoxSensor = config.LivoxSensor;

  Serializable::Drone::SetLidarConfig::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return success;
}

//}

/* getRgbCameraConfig() //{ */

std::pair<bool, RgbCameraConfig> UedsConnector::GetRgbCameraConfig(int sensorId) {

  Serializable::Drone::GetRgbCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetRgbCameraConfig::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  RgbCameraConfig config{};

  if (success) {

    config.show_debug_camera_ = response.config.show_debug_camera_;

    config.fov_ = response.config.fov_;

    config.offset_ = Coordinates{response.config.offset_x_, response.config.offset_y_, response.config.offset_z_};

    config.orientation_ = Rotation{response.config.rotation_pitch_, response.config.rotation_yaw_, response.config.rotation_roll_};

    config.width_  = response.config.width_;
    config.height_ = response.config.height_;

    config.enable_temporal_aa_ = response.config.enable_temporal_aa_;
    config.enable_hdr_         = response.config.enable_hdr_;
    config.enable_raytracing_  = response.config.enable_raytracing_;

    config.enable_motion_blur_     = response.config.enable_motion_blur_;
    config.motion_blur_amount_     = response.config.motion_blur_amount_;
    config.motion_blur_distortion_ = response.config.motion_blur_distortion_;

    config.exposure_.manual_          = response.config.exposure_.manual_;
    config.exposure_.shutter_time_    = response.config.exposure_.shutter_time_;
    config.exposure_.iso_             = response.config.exposure_.iso_;
    config.exposure_.ev_compensation_ = response.config.exposure_.ev_compensation_;

    config.lens_.fstop_                    = response.config.lens_.fstop_;
    config.lens_.focal_distance_           = response.config.lens_.focal_distance_;
    config.lens_.sensor_width_mm_          = response.config.lens_.sensor_width_mm_;
    config.lens_.vignette_intensity_       = response.config.lens_.vignette_intensity_;
    config.lens_.chromatic_aberration_     = response.config.lens_.chromatic_aberration_;
    config.lens_.bloom_intensity_          = response.config.lens_.bloom_intensity_;
    config.lens_.lens_flare_intensity_     = response.config.lens_.lens_flare_intensity_;
    config.lens_.white_temp_               = response.config.lens_.white_temp_;
    config.lens_.white_tint_               = response.config.lens_.white_tint_;
    config.lens_.motion_blur_from_shutter_ = response.config.lens_.motion_blur_from_shutter_;

    config.intrinsics_.use_custom_ = response.config.intrinsics_.use_custom_;
    config.intrinsics_.fx_         = response.config.intrinsics_.fx_;
    config.intrinsics_.fy_         = response.config.intrinsics_.fy_;
    config.intrinsics_.cx_         = response.config.intrinsics_.cx_;
    config.intrinsics_.cy_         = response.config.intrinsics_.cy_;

    config.distortion_.enable_ = response.config.distortion_.enable_;
    config.distortion_.k1_     = response.config.distortion_.k1_;
    config.distortion_.k2_     = response.config.distortion_.k2_;
    config.distortion_.k3_     = response.config.distortion_.k3_;
    config.distortion_.p1_     = response.config.distortion_.p1_;
    config.distortion_.p2_     = response.config.distortion_.p2_;

    config.noise_.enable_     = response.config.noise_.enable_;
    config.noise_.shot_scale_ = response.config.noise_.shot_scale_;
    config.noise_.read_sigma_ = response.config.noise_.read_sigma_;
    config.noise_.row_sigma_  = response.config.noise_.row_sigma_;

    config.rolling_shutter_.enable_       = response.config.rolling_shutter_.enable_;
    config.rolling_shutter_.readout_time_ = response.config.rolling_shutter_.readout_time_;
  }

  return std::make_pair(success, config);
}

//}

/* getStereoCameraConfig() //{ */

std::pair<bool, Serializable::Drone::StereoCameraConfig> UedsConnector::GetStereoCameraConfig(int sensorId) {

  Serializable::Drone::GetStereoCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetStereoCameraConfig::Response response{};
  const auto                                           status  = Request(request, response);
  const auto                                           success = status && response.status;

  Serializable::Drone::StereoCameraConfig config{};

  if (success) {
    config = response.config;
/* 
    config.show_debug_camera_ = response.config.show_debug_camera_;

    config.fov_ = response.config.fov_;

    config.offset_ = Coordinates{response.config.offset_x_, response.config.offset_y_, response.config.offset_z_};

    config.orientation_ = Rotation{response.config.rotation_pitch_, response.config.rotation_yaw_, response.config.rotation_roll_};

    config.width_  = response.config.width_;
    config.height_ = response.config.height_;

    config.baseline_ = response.config.baseline_;

    config.enable_temporal_aa_ = response.config.enable_temporal_aa_;
    config.enable_hdr_         = response.config.enable_hdr_;
    config.enable_raytracing_  = response.config.enable_raytracing_; */
  }

  return std::make_pair(success, config);
}

//}

/* setRgbCameraConfig() //{ */

bool UedsConnector::SetRgbCameraConfig(const RgbCameraConfig& config, int sensorId) {

  Serializable::Drone::SetRgbCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  request.config                    = Serializable::Drone::RgbCameraConfig{};
  request.config.show_debug_camera_ = config.show_debug_camera_;

  request.config.fov_ = config.fov_;

  request.config.offset_x_ = config.offset_.x;
  request.config.offset_y_ = config.offset_.y;
  request.config.offset_z_ = config.offset_.z;

  request.config.rotation_pitch_ = config.orientation_.pitch;
  request.config.rotation_yaw_   = config.orientation_.yaw;
  request.config.rotation_roll_  = config.orientation_.roll;

  request.config.width_  = config.width_;
  request.config.height_ = config.height_;

  request.config.enable_temporal_aa_ = config.enable_temporal_aa_;
  request.config.enable_hdr_         = config.enable_hdr_;
  request.config.enable_raytracing_  = config.enable_raytracing_;

  request.config.enable_motion_blur_ = config.enable_motion_blur_;
  request.config.motion_blur_amount_ = config.motion_blur_amount_;
  request.config.motion_blur_distortion_ = config.motion_blur_distortion_;

  request.config.exposure_.manual_          = config.exposure_.manual_;
  request.config.exposure_.shutter_time_    = config.exposure_.shutter_time_;
  request.config.exposure_.iso_             = config.exposure_.iso_;
  request.config.exposure_.ev_compensation_ = config.exposure_.ev_compensation_;

  request.config.lens_.fstop_                    = config.lens_.fstop_;
  request.config.lens_.focal_distance_           = config.lens_.focal_distance_;
  request.config.lens_.sensor_width_mm_          = config.lens_.sensor_width_mm_;
  request.config.lens_.vignette_intensity_       = config.lens_.vignette_intensity_;
  request.config.lens_.chromatic_aberration_     = config.lens_.chromatic_aberration_;
  request.config.lens_.bloom_intensity_          = config.lens_.bloom_intensity_;
  request.config.lens_.lens_flare_intensity_     = config.lens_.lens_flare_intensity_;
  request.config.lens_.white_temp_               = config.lens_.white_temp_;
  request.config.lens_.white_tint_               = config.lens_.white_tint_;
  request.config.lens_.motion_blur_from_shutter_ = config.lens_.motion_blur_from_shutter_;

  request.config.intrinsics_.use_custom_ = config.intrinsics_.use_custom_;
  request.config.intrinsics_.fx_         = config.intrinsics_.fx_;
  request.config.intrinsics_.fy_         = config.intrinsics_.fy_;
  request.config.intrinsics_.cx_         = config.intrinsics_.cx_;
  request.config.intrinsics_.cy_         = config.intrinsics_.cy_;

  request.config.distortion_.enable_ = config.distortion_.enable_;
  request.config.distortion_.k1_     = config.distortion_.k1_;
  request.config.distortion_.k2_     = config.distortion_.k2_;
  request.config.distortion_.k3_     = config.distortion_.k3_;
  request.config.distortion_.p1_     = config.distortion_.p1_;
  request.config.distortion_.p2_     = config.distortion_.p2_;

  request.config.noise_.enable_     = config.noise_.enable_;
  request.config.noise_.shot_scale_ = config.noise_.shot_scale_;
  request.config.noise_.read_sigma_ = config.noise_.read_sigma_;
  request.config.noise_.row_sigma_  = config.noise_.row_sigma_;

  request.config.rolling_shutter_.enable_       = config.rolling_shutter_.enable_;
  request.config.rolling_shutter_.readout_time_ = config.rolling_shutter_.readout_time_;

  Serializable::Drone::SetRgbCameraConfig::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return success;
}

//}

/* setStereoCameraConfig() //{ */

bool UedsConnector::SetStereoCameraConfig(const Serializable::Drone::StereoCameraConfig& config, int sensorId) {

  Serializable::Drone::SetStereoCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  request.config                    = config;
/*   request.config.show_debug_camera_ = config.show_debug_camera_;

  request.config.fov_ = config.fov_;

  request.config.offset_x_ = config.offset_.x;
  request.config.offset_y_ = config.offset_.y;
  request.config.offset_z_ = config.offset_.z;

  request.config.rotation_pitch_ = config.orientation_.pitch;
  request.config.rotation_yaw_   = config.orientation_.yaw;
  request.config.rotation_roll_  = config.orientation_.roll;

  request.config.width_  = config.width_;
  request.config.height_ = config.height_;

  request.config.baseline_ = config.baseline_;

  request.config.enable_temporal_aa_ = config.enable_temporal_aa_;
  request.config.enable_hdr_         = config.enable_hdr_;
  request.config.enable_raytracing_  = config.enable_raytracing_;
 */
  Serializable::Drone::SetStereoCameraConfig::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return success;
}

//}

/* getMoveLineVisible() //{ */

std::pair<bool, bool> UedsConnector::GetMoveLineVisible() {

  Serializable::Drone::GetMoveLineVisible::Request request{};

  Serializable::Drone::GetMoveLineVisible::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  return std::make_pair(success, success ? response.visible : false);
}

//}

/* setMoveLineVisible() //{ */

bool UedsConnector::SetMoveLineVisible(bool visible) {

  Serializable::Drone::SetMoveLineVisible::Request request{};
  request.visible = visible;

  Serializable::Drone::SetMoveLineVisible::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  return success;
}

//}

/* GetEventCameraData() //{ */

std::tuple<bool, std::vector<CameraEvent>, double> UedsConnector::GetEventCameraData(int sensorId) {

  Serializable::Drone::GetEventCameraData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetEventCameraData::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  std::vector<CameraEvent> events;

  if (success) {
    events.resize(response.events_.size());
    for (size_t i = 0; i < response.events_.size(); i++) {
      events[i].x        = response.events_[i].x;
      events[i].y        = response.events_[i].y;
      events[i].polarity = response.events_[i].polarity;
      events[i].stamp    = response.events_[i].stamp;
    }
  }

  return std::make_tuple(success, events, success ? response.stamp_ : 0.0);
}

//}

/* Get/SetEventCameraConfig() //{ */

std::pair<bool, EventCameraConfig> UedsConnector::GetEventCameraConfig(int sensorId) {

  Serializable::Drone::GetEventCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetEventCameraConfig::Response response{};
  const auto                                          status  = Request(request, response);
  const auto                                          success = status && response.status;

  EventCameraConfig config{};

  if (success) {
    config.show_debug_camera_      = response.config.show_debug_camera_;
    config.offset_                 = Coordinates{response.config.offset_x_, response.config.offset_y_, response.config.offset_z_};
    config.orientation_            = Rotation{response.config.rotation_pitch_, response.config.rotation_yaw_, response.config.rotation_roll_};
    config.fov_                    = response.config.fov_;
    config.width_                  = response.config.width_;
    config.height_                 = response.config.height_;
    config.contrast_threshold_pos_ = response.config.contrast_threshold_pos_;
    config.contrast_threshold_neg_ = response.config.contrast_threshold_neg_;
    config.use_sim_time_           = response.config.use_sim_time_;
  }

  return std::make_pair(success, config);
}

bool UedsConnector::SetEventCameraConfig(const EventCameraConfig& config, int sensorId) {

  Serializable::Drone::SetEventCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  request.config.show_debug_camera_      = config.show_debug_camera_;
  request.config.offset_x_               = config.offset_.x;
  request.config.offset_y_               = config.offset_.y;
  request.config.offset_z_               = config.offset_.z;
  request.config.rotation_pitch_         = config.orientation_.pitch;
  request.config.rotation_yaw_           = config.orientation_.yaw;
  request.config.rotation_roll_          = config.orientation_.roll;
  request.config.fov_                    = config.fov_;
  request.config.width_                  = config.width_;
  request.config.height_                 = config.height_;
  request.config.contrast_threshold_pos_ = config.contrast_threshold_pos_;
  request.config.contrast_threshold_neg_ = config.contrast_threshold_neg_;
  request.config.use_sim_time_           = config.use_sim_time_;

  Serializable::Drone::SetEventCameraConfig::Response response{};
  const auto                                          status = Request(request, response);

  return status && response.status;
}

//}

/* GetFisheyeCameraData() //{ */

std::tuple<bool, std::vector<unsigned char>, double, uint32_t> UedsConnector::GetFisheyeCameraData(int sensorId) {

  Serializable::Drone::GetFisheyeCameraData::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetFisheyeCameraData::Response response{};
  const auto                                          status  = Request(request, response);
  const auto                                          success = status && response.status;

  return std::make_tuple(success, success ? response.image_ : std::vector<unsigned char>(), success ? response.stamp_ : 0.0,
                         success ? response.image_.size() : 0);
}

//}

/* Get/SetFisheyeCameraConfig() //{ */

std::pair<bool, FisheyeCameraConfig> UedsConnector::GetFisheyeCameraConfig(int sensorId) {

  Serializable::Drone::GetFisheyeCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::GetFisheyeCameraConfig::Response response{};
  const auto                                            status  = Request(request, response);
  const auto                                            success = status && response.status;

  FisheyeCameraConfig config{};

  if (success) {
    config.show_debug_camera_ = response.config.show_debug_camera_;
    config.offset_            = Coordinates{response.config.offset_x_, response.config.offset_y_, response.config.offset_z_};
    config.orientation_       = Rotation{response.config.rotation_pitch_, response.config.rotation_yaw_, response.config.rotation_roll_};
    config.fov_               = response.config.fov_;
    config.width_             = response.config.width_;
    config.height_            = response.config.height_;
    config.lens_model_        = response.config.lens_model_;
  }

  return std::make_pair(success, config);
}

bool UedsConnector::SetFisheyeCameraConfig(const FisheyeCameraConfig& config, int sensorId) {

  Serializable::Drone::SetFisheyeCameraConfig::Request request{};
  request.sensor_id_ = sensorId;

  request.config.show_debug_camera_ = config.show_debug_camera_;
  request.config.offset_x_          = config.offset_.x;
  request.config.offset_y_          = config.offset_.y;
  request.config.offset_z_          = config.offset_.z;
  request.config.rotation_pitch_    = config.orientation_.pitch;
  request.config.rotation_yaw_      = config.orientation_.yaw;
  request.config.rotation_roll_     = config.orientation_.roll;
  request.config.fov_               = config.fov_;
  request.config.width_             = config.width_;
  request.config.height_            = config.height_;
  request.config.lens_model_        = config.lens_model_;

  Serializable::Drone::SetFisheyeCameraConfig::Response response{};
  const auto                                            status = Request(request, response);

  return status && response.status;
}

//}

/* sensor management //{ */

std::pair<bool, int> UedsConnector::AddSensor(int sensorType) {

  Serializable::Drone::AddSensor::Request request{};
  request.sensor_type_ = sensorType;

  Serializable::Drone::AddSensor::Response response{};
  const auto                               status  = Request(request, response);
  const auto                               success = status && response.status;

  return std::make_pair(success, success ? response.sensor_id_ : -1);
}

bool UedsConnector::RemoveSensor(int sensorId) {

  Serializable::Drone::RemoveSensor::Request request{};
  request.sensor_id_ = sensorId;

  Serializable::Drone::RemoveSensor::Response response{};
  const auto                                  status = Request(request, response);

  return status && response.status;
}

std::pair<bool, std::vector<SensorInfo>> UedsConnector::ListSensors() {

  Serializable::Drone::ListSensors::Request request{};

  Serializable::Drone::ListSensors::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  std::vector<SensorInfo> sensors;

  if (success) {
    for (size_t i = 0; i < response.sensor_ids_.size() && i < response.sensor_types_.size(); i++) {
      sensors.push_back(SensorInfo{response.sensor_ids_[i], response.sensor_types_[i]});
    }
  }

  return std::make_pair(success, sensors);
}

//}

/* device management //{ */

std::pair<bool, std::vector<SensorInfo>> UedsConnector::AddDevice(const std::string& deviceName, const Coordinates& offset, const Rotation& rotation,
                                                                  bool showMesh) {

  Serializable::Drone::AddDevice::Request request{};
  request.device_name_    = deviceName;
  request.offset_x_       = offset.x;
  request.offset_y_       = offset.y;
  request.offset_z_       = offset.z;
  request.rotation_pitch_ = rotation.pitch;
  request.rotation_yaw_   = rotation.yaw;
  request.rotation_roll_  = rotation.roll;
  request.show_mesh_      = showMesh;

  Serializable::Drone::AddDevice::Response response{};
  const auto                               status  = Request(request, response);
  const auto                               success = status && response.status;

  std::vector<SensorInfo> sensors;

  if (success) {
    for (size_t i = 0; i < response.sensor_ids_.size() && i < response.sensor_types_.size(); i++) {
      sensors.push_back(SensorInfo{response.sensor_ids_[i], response.sensor_types_[i]});
    }
  }

  return std::make_pair(success, sensors);
}

std::pair<bool, std::vector<DeviceInfo>> UedsConnector::ListDevices() {

  Serializable::Drone::ListDevices::Request request{};

  Serializable::Drone::ListDevices::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  std::vector<DeviceInfo> devices;

  if (success) {
    for (size_t i = 0; i < response.names_.size() && i < response.descriptions_.size(); i++) {
      devices.push_back(DeviceInfo{response.names_[i], response.descriptions_[i]});
    }
  }

  return std::make_pair(success, devices);
}

//}
