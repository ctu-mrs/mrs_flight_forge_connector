// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <map>
#include <vector>

namespace ueds_connector
{

struct Daytime
{
  int hour;
  int minute;
};

/* sensor + device identity //{ */

// Matches the simulator's SensorType enum. Sensor requests take a sensor id;
// -1 addresses the default (lazily created) sensor of the message's type.
enum SensorTypeEnum : int
{
  SENSOR_RGB_CAMERA     = 0,
  SENSOR_LIDAR          = 1,
  SENSOR_RANGEFINDER    = 2,
  SENSOR_LIDAR_LIVOX    = 3,
  SENSOR_RGB_SEG_CAMERA = 4,
  SENSOR_STEREO_CAMERA  = 5,
  SENSOR_EVENT_CAMERA   = 6,
  SENSOR_FISHEYE_CAMERA = 7,
  SENSOR_INSTANCE_SEG_CAMERA = 8,
  SENSOR_DEPTH_CAMERA = 9,
};

struct SensorInfo
{
  int id;
  int type;
};

struct DeviceInfo
{
  std::string name;
  std::string description;
};

//}

/* name helpers //{ */

struct UavFrameType
{
  // Frames are addressed by mesh name since API 0.12; any name resolving to a
  // packed mesh or an on-disk .glb (with optional YAML propeller sidecar) works.
  static const std::vector<std::string>& KnownNames() {
    static const std::vector<std::string> names = {"x500", "t650", "a300", "robofly", "wing", "wing_2", "gimbal", "empty", "drone10", "drone20"};
    return names;
  }
};

struct GraphicsSettings
{
  static const std::map<std::string, int>& Name2Id() {
    static const std::map<std::string, int> map = {{"low", 0}, {"medium", 1}, {"high", 2}, {"epic", 3}, {"cinematic", 4}, {"custom", 5}};
    return map;
  }
};

struct WorldName
{
  // Since API 0.12 worlds are addressed by level name (or a full package path,
  // which additionally goes through the fast-switch loading screen). This maps
  // the legacy friendly names onto the actual level names.
  static const std::map<std::string, std::string>& Name2Level() {
    static const std::map<std::string, std::string> map = {
        {"valley", "Valley"},
        {"forest", "Forest"},
        {"infinite_forest", "InfinityForest"},
        {"warehouse", "Warehouse"},
        {"cave", "CaveTunnel"},
        {"erding_airbase", "ErdingAirBase"},
        {"temesvar", "Temesvar_annotated"},
        {"electric_towers", "ElectricTowers"},
        {"race_1", "Race_1"},
        {"race_2", "Race_2"},
        {"race_3", "Race_3"},
        {"industrial_warehouse", "IndustialWarehouse"},
        {"service_tunnel", "ServiceTunnel"},
        {"dead_spruce_forest", "DeadSpruceForestBiome_Example_Daytime"},
        {"mala_skala", "MalaSkala"},
        {"spring_lab", "SprindLab"},
    };
    return map;
  }
};

struct WeatherType
{
  static const std::map<std::string, int>& Type2Id() {
    static const std::map<std::string, int> map = {{"sunny", 0},           {"cloudy", 1},          {"foggy", 2},     {"rain", 3},
                                                   {"rain_light", 4},      {"rain_thunderstorm", 5}, {"sand_dust_calm", 6}, {"sand_dust_storm", 7},
                                                   {"snow", 8},            {"snow_blizzards", 9},  {"overcast", 10}};
    return map;
  }
};

//}

/* Coordinates / Rotation //{ */

struct Coordinates
{
  Coordinates() = default;
  Coordinates(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {
  }

  double x;
  double y;
  double z;

  std::string toString() const {
    return "(x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", z: " + std::to_string(z) + ")";
  }
};

struct Rotation
{
  Rotation() = default;
  Rotation(double _pitch, double _yaw, double _roll) : pitch(_pitch), yaw(_yaw), roll(_roll) {
  }

  double pitch;
  double yaw;
  double roll;

  std::string toString() const {
    return "(pitch: " + std::to_string(pitch) + ", yaw: " + std::to_string(yaw) + ", roll: " + std::to_string(roll) + ")";
  }
};

//}

/* lidar data //{ */

struct LidarData
{
  LidarData() = default;

  double distance;
  double directionX;
  double directionY;
  double directionZ;

  std::string toString() const {
    return "(distance: " + std::to_string(distance) + ", directionX: " + std::to_string(directionX) + ", directionY: " + std::to_string(directionY) +
           ", directionZ: " + std::to_string(directionZ) + ")";
  }
};

struct LidarSegData
{
  LidarSegData() = default;

  double      distance;
  double      directionX;
  double      directionY;
  double      directionZ;
  int         segmentation;
  std::string toString() const {
    return "(distance: " + std::to_string(distance) + ", directionX: " + std::to_string(directionX) + ", directionY: " + std::to_string(directionY) +
           ", directionZ: " + std::to_string(directionZ) + ", segmentation: " + std::to_string(segmentation) + ")";
  }
};

struct LidarIntData
{
  LidarIntData() = default;

  double      distance;
  double      directionX;
  double      directionY;
  double      directionZ;
  int         intensity;
  std::string toString() const {
    return "(distance: " + std::to_string(distance) + ", directionX: " + std::to_string(directionX) + ", directionY: " + std::to_string(directionY) +
           ", directionZ: " + std::to_string(directionZ) + ", intensity: " + std::to_string(intensity) + ")";
  }
};

struct LidarConfig
{
  LidarConfig() = default;

  bool        Enable;
  bool        showBeams;
  double      beamLength;
  double      BeamHorRays;
  double      BeamVertRays;
  double      Frequency;
  Coordinates offset;
  Rotation    orientation;
  double      FOVHorLeft;
  double      FOVHorRight;
  double      FOVVertUp;
  double      FOVVertDown;
  bool        Livox;

  // Which non-repetitive scan pattern to replay when Livox is set ("avia",
  // "mid360"); matches the simulator's Content/Lidar/<name>.ffpat stem.
  std::string LivoxSensor = "mid360";

  std::string toString() const {
    return "(showBeams: " + std::to_string(showBeams) + ", beamLength: " + std::to_string(beamLength) + ", offset: " + offset.toString() +
           ", orientation: " + orientation.toString() + ")";
  }
};

//}

enum CameraCaptureModeEnum : unsigned short
{
  CAPTURE_ALL_FRAMES  = 0x0,
  CAPTURE_ON_MOVEMENT = 0x1,
  CAPTURE_ON_DEMAND   = 0x2,
};

/* camera configs //{ */

// Physically-based exposure: when manual_ is set, auto-exposure is replaced by
// the EV100 model driven by shutter time, ISO and the lens f-stop.
struct CameraExposure
{
  bool   manual_          = false;
  double shutter_time_    = 1.0 / 120.0;  // seconds
  double iso_             = 100.0;
  double ev_compensation_ = 0.0;
};

struct CameraLensEffects
{
  double fstop_                = 4.0;
  double focal_distance_       = 0.0;  // meters; <= 0 disables depth of field
  double sensor_width_mm_      = 23.76;
  double vignette_intensity_   = 0.0;
  double chromatic_aberration_ = 0.0;
  double bloom_intensity_      = 0.675;
  double lens_flare_intensity_ = 0.0;
  double white_temp_           = 0.0;  // Kelvin; <= 0 keeps engine default
  double white_tint_           = 0.0;

  // derive the motion blur length from shutter_time_/frame time instead of
  // the free-standing motion_blur_amount_
  bool motion_blur_from_shutter_ = false;
};

// Pinhole intrinsics in pixels, image origin top-left. Leave use_custom_ unset to
// drive the projection from fov_; Get always returns the effective values, derived
// from fov_ and the resolution when not custom.
struct CameraIntrinsics
{
  bool   use_custom_ = false;
  double fx_         = 0.0;
  double fy_         = 0.0;
  double cx_         = 0.0;
  double cy_         = 0.0;
};

// Brown-Conrady lens distortion (OpenCV convention). The simulator renders an
// automatically-sized overscan frustum and warps: bilinear for RGB, nearest for
// segmentation. K (camera_intrinsics) is unchanged by distortion - together they
// are the calibration.
struct CameraDistortion
{
  bool   enable_ = false;
  double k1_     = 0.0;
  double k2_     = 0.0;
  double k3_     = 0.0;
  double p1_     = 0.0;
  double p2_     = 0.0;
};

// Sensor noise applied server-side in the linearized domain:
// sigma = sqrt(shot_scale * signal + read_sigma^2) per channel (signal in [0,1]),
// plus optional per-row offsets (CMOS banding). Deterministic per frame stamp.
struct CameraNoise
{
  bool   enable_     = false;
  double shot_scale_ = 2.0e-4;
  double read_sigma_ = 3.0e-3;
  double row_sigma_  = 0.0;
};

// First-order rolling shutter: rows shifted by the rotational image flow over
// their readout delay, from the camera's measured angular velocity. Rotation
// only (the standard gyro approximation) - translation parallax is ignored.
struct CameraRollingShutter
{
  bool   enable_       = false;
  double readout_time_ = 0.03;  // seconds, top-to-bottom
};

struct RgbCameraConfig
{
  RgbCameraConfig() = default;

  bool show_debug_camera_;

  Coordinates offset_;
  Rotation    orientation_;

  double fov_;

  int width_;
  int height_;

  bool enable_temporal_aa_;
  bool enable_raytracing_;
  bool enable_hdr_;

  bool   enable_motion_blur_;
  double motion_blur_amount_;
  double motion_blur_distortion_;

  CameraExposure       exposure_;
  CameraLensEffects    lens_;
  CameraIntrinsics     intrinsics_;
  CameraDistortion     distortion_;
  CameraNoise          noise_;
  CameraRollingShutter rolling_shutter_;
};

struct EventCameraConfig
{
  EventCameraConfig() = default;

  bool show_debug_camera_ = false;

  Coordinates offset_;
  Rotation    orientation_;

  double fov_    = 90.0;
  int    width_  = 640;
  int    height_ = 480;

  // log-intensity contrast thresholds; an event fires each time a pixel's log
  // intensity moves by one threshold since its last event
  double contrast_threshold_pos_ = 0.2;
  double contrast_threshold_neg_ = 0.2;

  // stamp events with simulation time instead of wall clock; pair with a
  // fixed engine timestep for an exact-rate stream regardless of GPU load
  bool use_sim_time_ = false;
};

struct CameraEvent
{
  unsigned short x;
  unsigned short y;
  signed char    polarity;  // +1 brighter, -1 darker
  double         stamp;
};

enum FisheyeLensModelEnum : int
{
  FISHEYE_EQUIDISTANT   = 0,  // r = f * theta
  FISHEYE_EQUISOLID     = 1,  // r = 2f * sin(theta/2)
  FISHEYE_STEREOGRAPHIC = 2,  // r = 2f * tan(theta/2)
};

struct FisheyeCameraConfig
{
  FisheyeCameraConfig() = default;

  bool show_debug_camera_ = false;

  Coordinates offset_;
  Rotation    orientation_;

  // full field of view in degrees, up to 220
  double fov_ = 180.0;

  int width_  = 640;
  int height_ = 640;

  int lens_model_ = FISHEYE_EQUIDISTANT;
};

//}

}  // namespace ueds_connector
