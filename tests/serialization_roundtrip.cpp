#include <sstream>
#include <string>

#include <cereal/archives/binary.hpp>

#include <flight_forge_connector/serialization/serializable_shared.h>

#include "test_framework.h"

namespace
{

void Check(const bool bCondition, const std::string& What)
{
  fftest::Report(bCondition, What.c_str(), "serialization_roundtrip", 0);
}

/* RoundTrip //{ */

// serialize -> deserialize into a default-constructed instance
template <typename T>
T RoundTrip(const T& Input)
{
  std::stringstream Stream;
  {
    cereal::BinaryOutputArchive Archive(Stream);
    const_cast<T&>(Input).serialize(Archive);
  }

  T Output{};
  {
    cereal::BinaryInputArchive Archive(Stream);
    Output.serialize(Archive);
  }

  return Output;
}

//}

/* camera config blocks //{ */

FF_TEST(TestCameraBlocks)
{
  using namespace Serializable::Drone;

  {
    CameraIntrinsics In;
    In.use_custom_ = true;
    In.fx_         = 460.5;
    In.fy_         = 461.5;
    In.cx_         = 316.25;
    In.cy_         = 243.75;

    const CameraIntrinsics Out = RoundTrip(In);
    Check(Out.use_custom_ == In.use_custom_ && Out.fx_ == In.fx_ && Out.fy_ == In.fy_ && Out.cx_ == In.cx_ && Out.cy_ == In.cy_, "CameraIntrinsics");
  }

  {
    CameraDistortion In;
    In.enable_ = true;
    In.k1_     = -0.28;
    In.k2_     = 0.07;
    In.k3_     = -0.001;
    In.p1_     = 0.0004;
    In.p2_     = -0.0002;

    const CameraDistortion Out = RoundTrip(In);
    Check(Out.enable_ == In.enable_ && Out.k1_ == In.k1_ && Out.k2_ == In.k2_ && Out.k3_ == In.k3_ && Out.p1_ == In.p1_ && Out.p2_ == In.p2_,
          "CameraDistortion");
  }

  {
    CameraNoise In;
    In.enable_     = true;
    In.shot_scale_ = 3.0e-4;
    In.read_sigma_ = 4.0e-3;
    In.row_sigma_  = 1.0e-3;

    const CameraNoise Out = RoundTrip(In);
    Check(Out.enable_ == In.enable_ && Out.shot_scale_ == In.shot_scale_ && Out.read_sigma_ == In.read_sigma_ && Out.row_sigma_ == In.row_sigma_,
          "CameraNoise");
  }

  {
    CameraRollingShutter In;
    In.enable_       = true;
    In.readout_time_ = 0.025;

    const CameraRollingShutter Out = RoundTrip(In);
    Check(Out.enable_ == In.enable_ && Out.readout_time_ == In.readout_time_, "CameraRollingShutter");
  }

  {
    CameraExposure In;
    In.manual_          = true;
    In.shutter_time_    = 1.0 / 200.0;
    In.iso_             = 200.0;
    In.ev_compensation_ = -0.5;

    const CameraExposure Out = RoundTrip(In);
    Check(Out.manual_ == In.manual_ && Out.shutter_time_ == In.shutter_time_ && Out.iso_ == In.iso_ && Out.ev_compensation_ == In.ev_compensation_,
          "CameraExposure");
  }
}

//}

/* full RgbCameraConfig //{ */

FF_TEST(TestRgbCameraConfig)
{
  using namespace Serializable::Drone;

  RgbCameraConfig In{};
  In.show_debug_camera_      = true;
  In.offset_x_               = 1.0;
  In.offset_y_               = 2.0;
  In.offset_z_               = 3.0;
  In.rotation_pitch_         = 4.0;
  In.rotation_yaw_           = 5.0;
  In.rotation_roll_          = 6.0;
  In.fov_                    = 91.5;
  In.width_                  = 1280;
  In.height_                 = 720;
  In.enable_temporal_aa_     = true;
  In.enable_raytracing_      = false;
  In.enable_hdr_             = true;
  In.enable_motion_blur_     = true;
  In.motion_blur_amount_     = 0.7;
  In.motion_blur_distortion_ = 42.0;

  In.exposure_.manual_       = true;
  In.exposure_.shutter_time_ = 0.005;
  In.lens_.fstop_            = 2.8;
  In.lens_.white_temp_       = 6500.0;

  In.intrinsics_.use_custom_ = true;
  In.intrinsics_.fx_         = 500.0;
  In.intrinsics_.cy_         = 359.5;

  In.distortion_.enable_ = true;
  In.distortion_.k1_     = -0.2;

  In.noise_.enable_     = true;
  In.noise_.shot_scale_ = 1.0e-4;

  In.rolling_shutter_.enable_       = true;
  In.rolling_shutter_.readout_time_ = 0.03;

  const RgbCameraConfig Out = RoundTrip(In);

  Check(Out.show_debug_camera_ == In.show_debug_camera_ && Out.offset_x_ == In.offset_x_ && Out.offset_y_ == In.offset_y_ && Out.offset_z_ == In.offset_z_ &&
            Out.rotation_pitch_ == In.rotation_pitch_ && Out.rotation_yaw_ == In.rotation_yaw_ && Out.rotation_roll_ == In.rotation_roll_ &&
            Out.fov_ == In.fov_ && Out.width_ == In.width_ && Out.height_ == In.height_,
        "RgbCameraConfig basics");
  Check(Out.enable_temporal_aa_ == In.enable_temporal_aa_ && Out.enable_raytracing_ == In.enable_raytracing_ && Out.enable_hdr_ == In.enable_hdr_ &&
            Out.enable_motion_blur_ == In.enable_motion_blur_ && Out.motion_blur_amount_ == In.motion_blur_amount_ &&
            Out.motion_blur_distortion_ == In.motion_blur_distortion_,
        "RgbCameraConfig flags");
  Check(Out.exposure_.manual_ == In.exposure_.manual_ && Out.exposure_.shutter_time_ == In.exposure_.shutter_time_, "RgbCameraConfig.exposure");
  Check(Out.lens_.fstop_ == In.lens_.fstop_ && Out.lens_.white_temp_ == In.lens_.white_temp_, "RgbCameraConfig.lens");
  Check(Out.intrinsics_.use_custom_ == In.intrinsics_.use_custom_ && Out.intrinsics_.fx_ == In.intrinsics_.fx_ && Out.intrinsics_.cy_ == In.intrinsics_.cy_,
        "RgbCameraConfig.intrinsics");
  Check(Out.distortion_.enable_ == In.distortion_.enable_ && Out.distortion_.k1_ == In.distortion_.k1_, "RgbCameraConfig.distortion");
  Check(Out.noise_.enable_ == In.noise_.enable_ && Out.noise_.shot_scale_ == In.noise_.shot_scale_, "RgbCameraConfig.noise");
  Check(Out.rolling_shutter_.enable_ == In.rolling_shutter_.enable_ && Out.rolling_shutter_.readout_time_ == In.rolling_shutter_.readout_time_,
        "RgbCameraConfig.rolling_shutter");
}

//}

/* stereo config //{ */

FF_TEST(TestStereoCameraConfig)
{
  using namespace Serializable::Drone;

  StereoCameraConfig In{};
  In.show_debug_camera_    = true;
  In.offset_x_left_        = -1.0;
  In.offset_x_right_       = 1.0;
  In.offset_y_left_        = -2.0;
  In.offset_y_right_       = 2.0;
  In.offset_z_left_        = -3.0;
  In.offset_z_right_       = 3.0;
  In.rotation_pitch_left_  = 0.1;
  In.rotation_yaw_left_    = 0.2;
  In.rotation_roll_left_   = 0.3;
  In.rotation_pitch_right_ = 0.4;
  In.rotation_yaw_right_   = 0.5;
  In.rotation_roll_right_  = 0.6;
  In.fov_                  = 85.0;
  In.width_                = 848;
  In.height_               = 480;
  In.enable_temporal_aa_   = true;
  In.enable_raytracing_    = true;
  In.enable_hdr_           = false;

  In.intrinsics_.use_custom_ = true;
  In.intrinsics_.fx_         = 420.0;

  In.distortion_.enable_ = true;
  In.distortion_.p2_     = 0.001;

  const StereoCameraConfig Out = RoundTrip(In);

  Check(Out.offset_x_left_ == In.offset_x_left_ && Out.offset_x_right_ == In.offset_x_right_ && Out.rotation_roll_right_ == In.rotation_roll_right_ &&
            Out.fov_ == In.fov_ && Out.width_ == In.width_ && Out.enable_raytracing_ == In.enable_raytracing_,
        "StereoCameraConfig basics");
  Check(Out.intrinsics_.use_custom_ == In.intrinsics_.use_custom_ && Out.intrinsics_.fx_ == In.intrinsics_.fx_, "StereoCameraConfig.intrinsics");
  Check(Out.distortion_.enable_ == In.distortion_.enable_ && Out.distortion_.p2_ == In.distortion_.p2_, "StereoCameraConfig.distortion");
}

//}

/* scene editor messages //{ */

FF_TEST(TestSceneEditorMessages)
{
  using namespace Serializable::GameMode;

  {
    SpawnedObjectInfo In{};
    In.id          = 7;
    In.asset       = "Cube";
    In.stencil     = 203;
    In.position    = {1.5, -2.5, 3.5};
    In.orientation = {10.0, 20.0, 30.0};
    In.scale       = {2.0, 2.0, 0.5};

    const SpawnedObjectInfo Out = RoundTrip(In);
    Check(Out.id == In.id && Out.asset == In.asset && Out.stencil == In.stencil && Out.position == In.position && Out.orientation == In.orientation &&
              Out.scale == In.scale,
          "SpawnedObjectInfo");
  }

  {
    MoveSpawnedObject::Request In{};
    In.object_id   = 3;
    In.position    = {4.0, 5.0, 6.0};
    In.orientation = {7.0, 8.0, 9.0};
    In.scale       = {1.0, 2.0, 3.0};

    const MoveSpawnedObject::Request Out = RoundTrip(In);
    Check(Out.object_id == In.object_id && Out.position == In.position && Out.orientation == In.orientation && Out.scale == In.scale,
          "MoveSpawnedObject::Request");
  }

  {
    ListSpawnedObjects::Response In{};
    SpawnedObjectInfo            Info{};
    Info.id    = 1;
    Info.asset = "Sphere";
    In.objects.push_back(Info);

    const ListSpawnedObjects::Response Out = RoundTrip(In);
    Check(Out.objects.size() == 1 && Out.objects[0].id == 1 && Out.objects[0].asset == "Sphere", "ListSpawnedObjects::Response");
  }

  {
    ListAssets::Response In{};
    In.assets    = {"/Game/A", "/Game/B", "tree"};
    In.truncated = true;

    const ListAssets::Response Out = RoundTrip(In);
    Check(Out.assets == In.assets && Out.truncated == In.truncated, "ListAssets::Response");
  }

  {
    ExportScene::Response In{};
    In.yaml = "objects:\n  - asset: Cube\n";

    const ExportScene::Response Out = RoundTrip(In);
    Check(Out.yaml == In.yaml, "ExportScene::Response");
  }
}

//}

/* instance segmentation messages //{ */

FF_TEST(TestInstanceSegMessages)
{
  using namespace Serializable::Drone;

  {
    GetInstanceSegData::Request In{};
    In.sensor_id_ = 5;

    const GetInstanceSegData::Request Out = RoundTrip(In);
    Check(Out.sensor_id_ == 5, "GetInstanceSegData::Request");
  }

  {
    GetInstanceSegData::Response In{};
    In.image_ = {1, 2, 3, 250};
    In.stamp_ = 12.5;
    In.size_  = 4;

    const GetInstanceSegData::Response Out = RoundTrip(In);
    Check(Out.image_ == In.image_ && Out.stamp_ == In.stamp_ && Out.size_ == In.size_, "GetInstanceSegData::Response");
  }

  {
    GetInstanceSegMap::Response In{};
    InstanceSegMapEntry          Entry{};
    Entry.id    = 42;
    Entry.actor = "/Game/Map.Map:PersistentLevel.Cube_3";
    In.entries_.push_back(Entry);

    const GetInstanceSegMap::Response Out = RoundTrip(In);
    Check(Out.entries_.size() == 1 && Out.entries_[0].id == 42 && Out.entries_[0].actor == Entry.actor, "GetInstanceSegMap::Response");
  }
}

//}

/* depth camera message //{ */

FF_TEST(TestDepthCameraMessage)
{
  using namespace Serializable::Drone;

  {
    GetDepthCameraData::Request In{};
    In.sensor_id_ = 3;

    const GetDepthCameraData::Request Out = RoundTrip(In);
    Check(Out.sensor_id_ == 3, "GetDepthCameraData::Request");
  }

  {
    GetDepthCameraData::Response In{};
    In.image_  = {0, 1000, 65535, 500};
    In.width_  = 2;
    In.height_ = 2;
    In.stamp_  = 3.25;

    const GetDepthCameraData::Response Out = RoundTrip(In);
    Check(Out.image_ == In.image_ && Out.width_ == In.width_ && Out.height_ == In.height_ && Out.stamp_ == In.stamp_, "GetDepthCameraData::Response");
  }
}

//}

/* lidar + event + fisheye configs //{ */

FF_TEST(TestOtherSensorConfigs)
{
  using namespace Serializable::Drone;

  {
    LidarConfig In{};
    In.Enable       = true;
    In.BeamLength   = 2000.0;
    In.BeamHorRays  = 128;
    In.BeamVertRays = 64;
    In.Frequency    = 10.0;
    In.OffsetX      = 1.0;
    In.FOVHorLeft   = 180.0;
    In.Livox        = true;
    In.LivoxSensor  = "mid360";

    const LidarConfig Out = RoundTrip(In);
    Check(Out.Enable == In.Enable && Out.BeamLength == In.BeamLength && Out.BeamHorRays == In.BeamHorRays && Out.Frequency == In.Frequency &&
              Out.OffsetX == In.OffsetX && Out.FOVHorLeft == In.FOVHorLeft && Out.Livox == In.Livox && Out.LivoxSensor == In.LivoxSensor,
          "LidarConfig");
  }
}

//}

}  // namespace

int main()
{
  return fftest::RunAll();
}
