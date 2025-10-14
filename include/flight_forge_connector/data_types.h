// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <map>

namespace ueds_connector
{

struct Daytime
{
  int hour;
  int minute;
};  

  
struct UavFrameType
{
    static const std::map<std::string, int>& Type2IdMesh() {
        static const std::map<std::string, int> map = {
            {"x500", 0},
            {"t650", 1},
            {"a300", 2},
            {"robofly", 3},
            {"wing", 4},
            {"wing_2", 5},
            {"gimbal", 6},
            {"empty", 7}
        };
        return map;
    }
};

struct GraphicsSettings
{
    static const std::map<std::string, int>& Name2Id() {
        static const std::map<std::string, int> map = {
            {"low", 0},
            {"medium", 1},
            {"high", 2},
            {"epic", 3},
            {"cinematic", 4},
            {"custom", 5}
        };
        return map;
    }
};  

struct WorldName
{
    static const std::map<std::string, short>& Name2Id() {
        static const std::map<std::string, short> map = {
            {"valley", 0},
            {"forest", 1},
            {"infinite_forest", 2},
            {"warehouse", 3},
            {"cave", 4},
            {"erding_airbase", 5},
            {"temesvar", 6},
            {"electric_towers", 7},
            {"race_1", 8},
            {"race_2", 9},
            {"industrial_warehouse", 10},
            {"service_tunnel", 11},
            {"dead_spruce_forest", 12},
            {"race_3", 13},
            {"mala_skala", 14}
        };
        return map;
    }
};

struct WeatherType
{
    static const std::map<std::string, int>& Type2Id() {
        static const std::map<std::string, int> map = {
            {"sunny", 0},
            {"cloudy", 1},
            {"foggy", 2},
            {"rain", 3},
            {"rain_light", 4},
            {"rain_thunderstorm", 5},
            {"sand_dust_calm", 6},
            {"sand_dust_storm", 7},
            {"snow", 8},
            {"snow_blizzards", 9},
            {"overcast", 10}
        };
        return map;
    }
};

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
  int      intensity;
  std::string toString() const {
    return "(distance: " + std::to_string(distance) + ", directionX: " + std::to_string(directionX) + ", directionY: " + std::to_string(directionY) +
           ", directionZ: " + std::to_string(directionZ) + ", intensity: " + std::to_string(intensity) + ")";
  }
};

struct LidarConfig
{
  LidarConfig() = default;
  LidarConfig(bool Enable, bool showBeams, double BeamHorRays, double BeamVertRays, double beamLength, double Frequency, const Coordinates offset,
              const Rotation orientation, double FOVHorLeft, double FOVHorRight, double FOVVertUp, double FOVVertDown, bool Livox)
      : Enable(Enable),
        showBeams(showBeams),
        beamLength(beamLength),
        BeamHorRays(BeamHorRays),
        BeamVertRays(BeamVertRays),
        Frequency(Frequency),
        offset(offset),
        orientation(orientation),
        FOVHorLeft(FOVHorLeft),
        FOVHorRight(FOVHorRight),
        FOVVertUp(FOVVertUp),
        FOVVertDown(FOVVertDown),
        Livox(Livox)
  {
  }

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
  bool       Livox;

  std::string toString() const {
    return "(showBeams: " + std::to_string(showBeams) + ", beamLength: " + std::to_string(beamLength) + ", offset: " + offset.toString() +
           ", orientation: " + orientation.toString() + ")";
  }
};

enum CameraCaptureModeEnum : unsigned short
{
  CAPTURE_ALL_FRAMES  = 0x0,
  CAPTURE_ON_MOVEMENT = 0x1,
  CAPTURE_ON_DEMAND   = 0x2,
};

struct RgbCameraConfig
{
  RgbCameraConfig() = default;
  RgbCameraConfig(bool show_debug_camera, const Coordinates offset, const Rotation orientation, double fov, int width, int height, bool enable_temporal_aa,
                  bool enable_raytracing, bool enable_hdr, bool enable_motion_blur, double motion_blur_amount, double motion_blur_distortion)
      : show_debug_camera_(show_debug_camera),
        offset_(offset),
        orientation_(orientation),
        fov_(fov),
        width_(width),
        height_(height),
        enable_temporal_aa_(enable_temporal_aa),
        enable_raytracing_(enable_raytracing),
        enable_hdr_(enable_hdr),
        enable_motion_blur_(enable_motion_blur),
        motion_blur_amount_(motion_blur_amount),
        motion_blur_distortion_(motion_blur_distortion) 
  {
  }

  bool show_debug_camera_;

  Coordinates offset_;

  Rotation orientation_;

  double fov_;

  int width_;
  int height_;

  bool enable_temporal_aa_;
  bool enable_raytracing_;
  bool enable_hdr_;

  bool     enable_motion_blur_;
  double   motion_blur_amount_;
  double   motion_blur_distortion_;
};

struct StereoCameraConfig
{
  StereoCameraConfig() = default;
  StereoCameraConfig(bool show_debug_camera, const Coordinates offset, const Rotation orientation, double fov, int width, int height, double baseline,
                     bool enable_temporal_aa, bool enable_raytracing, bool enable_hdr)
      : show_debug_camera_(show_debug_camera),
        offset_(offset),
        orientation_(orientation),
        fov_(fov),
        width_(width),
        height_(height),
        baseline_(baseline),
        enable_temporal_aa_(enable_temporal_aa),
        enable_raytracing_(enable_raytracing),
        enable_hdr_(enable_hdr) {
  }

  bool show_debug_camera_;

  Coordinates offset_;

  Rotation orientation_;

  double fov_;

  int width_;
  int height_;

  double baseline_;

  bool enable_temporal_aa_;
  bool enable_raytracing_;
  bool enable_hdr_;
};

}  // namespace ueds_connector
