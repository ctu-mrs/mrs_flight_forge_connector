#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <flight_forge_connector/wire/flightforge_generated.h>

// Everything here is in the wire's frame: the ground frame (x = Unreal +X, y = Unreal -Y, z up), metres, rad.
namespace flightforge
{

/* results //{ */

struct Status
{
  bool        ok = false;
  std::string error;

  explicit operator bool() const {
    return ok;
  }

  static Status Ok() {
    return Status{true, {}};
  }

  static Status Fail(std::string message) {
    return Status{false, std::move(message)};
  }
};

template <typename T>
struct Expected : Status
{
  T value{};

  Expected() = default;
  Expected(const Status& status) : Status(status) {}
  Expected(T v) : Status{true, {}}, value(std::move(v)) {}

  const T& operator*() const {
    return value;
  }
  T& operator*() {
    return value;
  }
  const T* operator->() const {
    return &value;
  }
  T* operator->() {
    return &value;
  }
};

/* //} */

/* geometry //{ */

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// unit quaternion, body to parent
struct Quat
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  // rad, applied yaw * pitch * roll about the parent's z, y, x
  static Quat FromRpy(const double roll, const double pitch, const double yaw) {
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    return Quat{sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
  }

  static Quat FromYaw(const double yaw) {
    return FromRpy(0.0, 0.0, yaw);
  }

  double Yaw() const {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  Vec3 Rotate(const Vec3& v) const {
    const double tx = 2.0 * (y * v.z - z * v.y), ty = 2.0 * (z * v.x - x * v.z), tz = 2.0 * (x * v.y - y * v.x);
    return Vec3{v.x + w * tx + (y * tz - z * ty), v.y + w * ty + (z * tx - x * tz), v.z + w * tz + (x * ty - y * tx)};
  }

  Quat Conjugate() const {
    return Quat{-x, -y, -z, w};
  }

  Quat operator*(const Quat& o) const {
    return Quat{w * o.x + x * o.w + y * o.z - z * o.y, w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w,
                w * o.w - x * o.x - y * o.y - z * o.z};
  }
};

struct Pose
{
  Vec3 position;
  Quat orientation;

  static Pose At(const double x, const double y, const double z, const double yaw = 0.0) {
    return Pose{Vec3{x, y, z}, Quat::FromYaw(yaw)};
  }
};

/* //} */

/* robots //{ */

struct RobotState
{
  std::optional<Pose>  base;                // absent leaves the base where it is
  std::vector<double>  joint_positions;     // rad or m, GetDescription's joint order
  std::vector<double>  joint_velocities;    // rad/s or m/s
  std::optional<Vec3>  base_linear_velocity;   // m/s
  std::optional<Vec3>  base_angular_velocity;  // rad/s, body frame
};

struct StampedState
{
  double     stamp = 0.0;  // s, sim
  RobotState state;
};

struct SetStateResult
{
  int  robot_id = -1;
  Pose base;
  bool hit = false;
  Vec3 impact_point;
};

struct TrajectorySample
{
  double t = 0.0;  // s, sim
  Pose   base;
};

struct CollisionState
{
  bool collided = false;
  Vec3 impact_point;
};

struct RobotInfo
{
  int             id = -1;
  std::string     name;
  wire::RobotKind kind = wire::RobotKind::GENERIC;
  std::string     label;
};

/* //} */

/* sensors //{ */

struct SensorInfo
{
  int              id   = -1;
  wire::SensorType type = wire::SensorType::RGB_CAMERA;
  std::string      link;
};

struct DeviceInfo
{
  std::string name;
  std::string description;
};

// data holds the codec's output, or packed rows of format when compression is NONE
struct Image
{
  double             stamp       = 0.0;
  uint32_t           width       = 0;
  uint32_t           height      = 0;
  wire::PixelFormat  format      = wire::PixelFormat::UNKNOWN;
  wire::Compression  compression = wire::Compression::NONE;
  std::vector<uint8_t> data;
};

struct StereoImage
{
  double stamp = 0.0;
  Image  left;
  Image  right;
};

// 0 invalid or too close, 65535 clamped
struct DepthImage
{
  double                stamp  = 0.0;
  uint32_t              width  = 0;
  uint32_t              height = 0;
  std::vector<uint16_t> mm;
};

struct LidarScan
{
  double                           stamp = 0.0;
  Pose                             sensor_pose;  // ground frame, at scan time
  uint32_t                         rows = 0;     // 0 when not organised
  uint32_t                         cols = 0;
  std::vector<std::array<float, 3>> points;      // m, sensor frame, NaN for no return
  std::vector<int32_t>             intensity;
  std::vector<int32_t>             class_id;
  std::vector<uint32_t>            instance_id;
};

struct RangefinderReading
{
  double stamp = 0.0;
  double range = 0.0;  // m, NaN for no return
};

struct CameraEvent
{
  uint16_t x        = 0;
  uint16_t y        = 0;
  int8_t   polarity = 0;  // +1 brighter, -1 darker
  double   stamp    = 0.0;
};

// xyp = x | y << 16 | brighter << 31, t_us after base_stamp, time-sorted
struct CameraEvents
{
  std::vector<uint32_t> xyp;
  std::vector<uint32_t> t_us;

  double   base_stamp = 0.0;
  double   stamp      = 0.0;  // the batch is complete up to here
  uint32_t dropped    = 0;
  uint32_t width      = 0;
  uint32_t height     = 0;

  size_t size() const {
    return xyp.size();
  }

  CameraEvent at(const size_t i) const {
    return CameraEvent{static_cast<uint16_t>(xyp[i] & 0xFFFFu), static_cast<uint16_t>((xyp[i] >> 16) & 0x7FFFu),
                       static_cast<int8_t>((xyp[i] & 0x80000000u) ? 1 : -1), base_stamp + t_us[i] * 1e-6};
  }

  std::vector<CameraEvent> unpack() const {
    std::vector<CameraEvent> events(xyp.size());
    for (size_t i = 0; i < xyp.size(); ++i) {
      events[i] = at(i);
    }
    return events;
  }
};

/* //} */

/* simulator //{ */

struct ApiInfo
{
  uint16_t             major = 0;
  uint16_t             minor = 0;
  std::vector<uint8_t> schema;  // flightforge.bfbs
};

struct TimeInfo
{
  double wall_time     = 0.0;
  double sim_time      = 0.0;
  double time_dilation = 1.0;
};

// where the level animates the clock it advances per simulated second and holds while the world is paused
struct DaytimeInfo
{
  double      hour     = 0.0;  // since midnight, NaN on a world lit by its own lights until SetDaytime
  bool        animated = false;
  std::string sky;  // ultra_dynamic_sky, sun_sky or lights
};

struct StepResult
{
  double   sim_time   = 0.0;
  uint64_t tick_count = 0;
};

struct SpawnedObject
{
  int         id = -1;
  std::string asset;
  int         stencil = -1;
  Pose        pose;
  Vec3        scale{1.0, 1.0, 1.0};
};

struct AssetList
{
  std::vector<std::string> assets;
  bool                     truncated = false;
};

struct LibraryPage
{
  std::vector<wire::LibraryObjectT> objects;
  uint32_t                          total = 0;
};

struct WorldMesh
{
  std::vector<std::array<float, 3>> vertices;
  std::vector<uint32_t>             indices;
  bool                              truncated    = false;
  int                               source_count = 0;
};

/* //} */

/* name helpers //{ */

struct UavFrameType
{
  // any name resolving to a packed mesh or an on-disk .glb works
  static const std::vector<std::string>& KnownNames() {
    static const std::vector<std::string> names = {"x500", "t650", "a300", "robofly", "wing", "wing_2", "gimbal", "empty", "drone10", "drone20"};
    return names;
  }
};

struct WorldName
{
  // legacy friendly names onto level names; a full package path also works
  static const std::map<std::string, std::string>& Name2Level() {
    static const std::map<std::string, std::string> map = {
        {"valley", "Valley"},
        {"forest", "Forest"},
        {"infinite_forest", "infinite_forest"},
        {"warehouse", "Warehouse"},
        {"cave", "cave"},
        {"erding_airbase", "erding_airbase"},
        {"temesvar", "Temesvar_annotated"},
        {"electric_towers", "electric_towers"},
        {"eletric_towers", "electric_towers"},
        {"race_1", "Race_1"},
        {"race_2", "Race_2"},
        {"race_3", "Race_3"},
        {"industrial_warehouse", "/Game/Worlds/IndustrialWarehouse/Maps/industrial_warehouse"},
        {"service_tunnel", "service_tunnel"},
        {"dead_spruce_forest", "dead_spruce_forest"},
        {"mala_skala", "mala_skala"},
        {"kayenta_mine", "kayenta_mine"},
        {"sprind_lab", "sprind_lab"},
        {"spring_lab", "sprind_lab"},
    };
    return map;
  }
};

/* //} */

}  // namespace flightforge
