#include <algorithm>
#include <cstdint>

#include <flight_forge_connector/data_types.h>

#include "test_framework.h"

using namespace ueds_connector;

/* frame + world catalogs //{ */

FF_TEST(UavFrameCatalog)
{
  const auto& Names = UavFrameType::KnownNames();

  FF_CHECK(!Names.empty());
  FF_CHECK(std::find(Names.begin(), Names.end(), "x500") != Names.end());
  FF_CHECK(std::find(Names.begin(), Names.end(), "t650") != Names.end());
}

FF_TEST(WorldNameCatalog)
{
  const auto& Levels = WorldName::Name2Level();

  FF_CHECK(!Levels.empty());

  // every friendly name maps to a non-empty level identifier
  for (const auto& [Name, Level] : Levels) {
    FF_CHECK(!Name.empty());
    FF_CHECK(!Level.empty());
  }
}

/* //} */

/* config defaults //{ */

FF_TEST(CameraConfigDefaults)
{
  const CameraIntrinsics Intrinsics;
  FF_CHECK(!Intrinsics.use_custom_ && Intrinsics.fx_ == 0.0);

  const CameraDistortion Distortion;
  FF_CHECK(!Distortion.enable_ && Distortion.k1_ == 0.0 && Distortion.p2_ == 0.0);

  const CameraNoise Noise;
  FF_CHECK(!Noise.enable_);
  FF_CHECK_NEAR(Noise.shot_scale_, 2.0e-4, 1.0e-12);
  FF_CHECK_NEAR(Noise.read_sigma_, 3.0e-3, 1.0e-12);

  const CameraRollingShutter RollingShutter;
  FF_CHECK(!RollingShutter.enable_);
  FF_CHECK_NEAR(RollingShutter.readout_time_, 0.03, 1.0e-12);

  const CameraExposure Exposure;
  FF_CHECK(!Exposure.manual_);
  FF_CHECK_NEAR(Exposure.iso_, 100.0, 1.0e-9);
}

FF_TEST(SensorTypeValues)
{
  // wire-stable values; the server's SensorType enum must agree
  FF_CHECK(static_cast<int>(SENSOR_RGB_CAMERA) == 0);
  FF_CHECK(static_cast<int>(SENSOR_LIDAR) == 1);
  FF_CHECK(static_cast<int>(SENSOR_RANGEFINDER) == 2);
  FF_CHECK(static_cast<int>(SENSOR_LIDAR_LIVOX) == 3);
  FF_CHECK(static_cast<int>(SENSOR_RGB_SEG_CAMERA) == 4);
  FF_CHECK(static_cast<int>(SENSOR_STEREO_CAMERA) == 5);
  FF_CHECK(static_cast<int>(SENSOR_EVENT_CAMERA) == 6);
  FF_CHECK(static_cast<int>(SENSOR_FISHEYE_CAMERA) == 7);
  FF_CHECK(static_cast<int>(SENSOR_INSTANCE_SEG_CAMERA) == 8);
  FF_CHECK(static_cast<int>(SENSOR_DEPTH_CAMERA) == 9);
}

/* //} */

/* instance id convention //{ */

FF_TEST(InstanceIdEncoding)
{
  // ids travel little-endian in RGB: id = r + (g << 8) + (b << 16)
  const auto Decode = [](const uint8_t R, const uint8_t G, const uint8_t B) { return uint32_t(R) + (uint32_t(G) << 8) + (uint32_t(B) << 16); };

  FF_CHECK(Decode(42, 0, 0) == 42u);
  FF_CHECK(Decode(0, 1, 0) == 256u);
  FF_CHECK(Decode(1, 1, 1) == 65793u);
  FF_CHECK(Decode(255, 255, 255) == 16777215u);

  // and the encoding the server uses round-trips
  for (const uint32_t Id : {0u, 1u, 255u, 256u, 65793u, 16777215u}) {
    const uint8_t R = Id & 0xFF;
    const uint8_t G = (Id >> 8) & 0xFF;
    const uint8_t B = (Id >> 16) & 0xFF;
    FF_CHECK(Decode(R, G, B) == Id);
  }
}

/* //} */

int main()
{
  return fftest::RunAll();
}
