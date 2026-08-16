#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

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

/* ground-truth query logic //{ */

// The sphere-overlap search that backs get_distance_batch, against a predicate whose
// answer is known exactly. The loop is what decides whether an ESDF ground truth is
// trustworthy, and it needs no simulator to check.
FF_TEST(SphereSearchConvergesOnTheSurface)
{
  const auto Search = [](const auto& Blocked, const double MaxRadius, const double Tolerance) {
    if (Blocked(0.001)) {
      return 0.0;
    }
    if (!Blocked(MaxRadius)) {
      return MaxRadius;
    }

    double Lo = 0.0, Hi = MaxRadius;
    while (Hi - Lo > Tolerance) {
      const double Mid = 0.5 * (Lo + Hi);
      (Blocked(Mid) ? Hi : Lo) = Mid;
    }
    return 0.5 * (Lo + Hi);
  };

  // a surface at a known distance: the sphere is blocked once its radius reaches it
  for (const double Truth : {0.05, 0.5, 1.0, 2.75, 4.999}) {
    const auto Blocked = [Truth](const double Radius) { return Radius >= Truth; };
    const double Found = Search(Blocked, 5.0, 0.001);
    FF_CHECK(std::fabs(Found - Truth) <= 0.001);
  }

  // nothing within reach reports exactly the truncation radius, not a guess
  FF_CHECK(Search([](double) { return false; }, 5.0, 0.001) == 5.0);

  // a point inside geometry reports zero, whatever else is around
  FF_CHECK(Search([](double) { return true; }, 5.0, 0.001) == 0.0);

  // the tolerance is honoured rather than merely approached
  const auto Blocked = [](const double Radius) { return Radius >= 1.234567; };
  FF_CHECK(std::fabs(Search(Blocked, 5.0, 0.01) - 1.234567) <= 0.01);
}

// Triangle clipping keeps anything that touches the box, so a query never opens a hole
// where its own boundary crosses the geometry it asked about.
FF_TEST(TriangleClippingKeepsStraddlers)
{
  struct FBox2
  {
    double MinX, MinY, MaxX, MaxY;

    bool Intersects(const FBox2& Other) const {
      return !(Other.MinX > MaxX || Other.MaxX < MinX || Other.MinY > MaxY || Other.MaxY < MinY);
    }
  };

  const FBox2 Query{-10.0, -10.0, 10.0, 10.0};

  const auto TriBounds = [](const double Ax, const double Ay, const double Bx, const double By, const double Cx, const double Cy) {
    return FBox2{std::min({Ax, Bx, Cx}), std::min({Ay, By, Cy}), std::max({Ax, Bx, Cx}), std::max({Ay, By, Cy})};
  };

  FF_CHECK(Query.Intersects(TriBounds(0, 0, 1, 0, 0, 1)));        // wholly inside
  FF_CHECK(Query.Intersects(TriBounds(9, 9, 11, 9, 9, 11)));      // straddling a corner
  FF_CHECK(Query.Intersects(TriBounds(-20, 0, 20, 0, 0, 1)));     // spanning the box
  FF_CHECK(!Query.Intersects(TriBounds(11, 11, 12, 11, 11, 12))); // wholly outside
  FF_CHECK(!Query.Intersects(TriBounds(-40, -40, -30, -40, -40, -30)));
}

// Emitting a vertex only when a triangle keeps it is what stops a clipped export
// carrying orphans; the remap has to stay consistent while it does that.
FF_TEST(ClippedExportCompactsVertices)
{
  const std::vector<uint32_t> Indices = {0, 1, 2, 3, 4, 5};
  const std::vector<bool>     Keep    = {true, false};  // per triangle

  std::vector<int32_t>  Remap(6, -1);
  std::vector<uint32_t> Out;
  uint32_t              NextVertex = 0;

  for (size_t t = 0; t < Keep.size(); ++t) {
    if (!Keep[t]) {
      continue;
    }
    for (size_t c = 0; c < 3; ++c) {
      const uint32_t Corner = Indices[t * 3 + c];
      if (Remap[Corner] < 0) {
        Remap[Corner] = static_cast<int32_t>(NextVertex++);
      }
      Out.push_back(static_cast<uint32_t>(Remap[Corner]));
    }
  }

  FF_CHECK(Out.size() == 3);
  FF_CHECK(NextVertex == 3);                       // only the kept triangle's vertices exist
  FF_CHECK(Out[0] == 0 && Out[1] == 1 && Out[2] == 2);
  FF_CHECK(Remap[3] < 0 && Remap[4] < 0 && Remap[5] < 0);

  // a shared vertex is emitted once and referenced twice
  std::vector<int32_t> Shared(4, -1);
  uint32_t             Next = 0;
  std::vector<uint32_t> Quad;
  for (const uint32_t Corner : {0u, 1u, 2u, 0u, 2u, 3u}) {
    if (Shared[Corner] < 0) {
      Shared[Corner] = static_cast<int32_t>(Next++);
    }
    Quad.push_back(static_cast<uint32_t>(Shared[Corner]));
  }
  FF_CHECK(Next == 4);
  FF_CHECK(Quad.size() == 6);
  FF_CHECK(Quad[3] == Quad[0] && Quad[4] == Quad[2]);
}

/* //} */

int main()
{
  return fftest::RunAll();
}
