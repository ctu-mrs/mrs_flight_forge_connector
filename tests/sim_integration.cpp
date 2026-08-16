#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <vector>

#include <flight_forge_connector/flight_forge_connector.h>
#include <flight_forge_connector/game_mode_controller.h>

#include "test_framework.h"

namespace
{

constexpr const char* kAddress      = "127.0.0.1";
constexpr uint16_t    kGameModePort = 8551;

std::unique_ptr<ueds_connector::GameModeController> GameMode;
std::unique_ptr<ueds_connector::UedsConnector>      Drone;
int                                                 DronePort = -1;

// An /Engine path resolves without search_paths and exists in every install, so the test
// does not depend on what content the host project happens to have.
const std::string kSceneYaml = "objects:\n"
                               "  - asset: /Engine/BasicShapes/Cube\n"
                               "    position: [5.0, 0.0, 1.0]\n"
                               "  - asset: /Engine/BasicShapes/Cube\n"
                               "    position: [5.0, 2.0, 1.0]\n"
                               "    scale: 0.5\n";

/* basics //{ */

FF_TEST(ApiVersion)
{
  const auto [Ok, Version] = GameMode->GetApiVersion();
  FF_REQUIRE(Ok);
  FF_CHECK(Version.first == 0 && Version.second >= 14);
}

FF_TEST(FpsReports)
{
  const auto [Ok, Fps] = GameMode->GetFps();
  FF_CHECK(Ok && Fps > 0.0f);
}

FF_TEST(RgbCapture)
{
  const auto [Ok, Image, Stamp, Size] = Drone->GetRgbCameraData();

  FF_REQUIRE(Ok);
  FF_CHECK(!Image.empty());
  FF_CHECK(Stamp > 0.0);
}

/* //} */

/* camera calibration round-trips //{ */

FF_TEST(IntrinsicsRoundTrip)
{
  auto [Ok0, Config] = Drone->GetRgbCameraConfig();
  FF_REQUIRE(Ok0);

  // the FOV path publishes a centered effective K
  FF_CHECK(Config.intrinsics_.fx_ > 0.0);
  FF_CHECK_NEAR(Config.intrinsics_.cx_, Config.width_ / 2.0, 1.0);

  Config.intrinsics_.use_custom_ = true;
  Config.intrinsics_.fx_         = 500.0;
  Config.intrinsics_.fy_         = 505.0;
  Config.intrinsics_.cx_         = 300.0;
  Config.intrinsics_.cy_         = 250.0;
  FF_REQUIRE(Drone->SetRgbCameraConfig(Config));

  const auto [Ok1, Read] = Drone->GetRgbCameraConfig();
  FF_REQUIRE(Ok1);
  FF_CHECK_NEAR(Read.intrinsics_.fx_, 500.0, 1.0e-9);
  FF_CHECK_NEAR(Read.intrinsics_.fy_, 505.0, 1.0e-9);
  FF_CHECK_NEAR(Read.intrinsics_.cx_, 300.0, 1.0e-9);
  FF_CHECK_NEAR(Read.intrinsics_.cy_, 250.0, 1.0e-9);
}

FF_TEST(DistortionRoundTrip)
{
  auto [Ok0, Config] = Drone->GetRgbCameraConfig();
  FF_REQUIRE(Ok0);

  Config.distortion_.enable_ = true;
  Config.distortion_.k1_     = -0.2;
  Config.distortion_.k2_     = 0.05;
  Config.distortion_.p1_     = 0.001;
  FF_REQUIRE(Drone->SetRgbCameraConfig(Config));

  const auto [Ok1, Read] = Drone->GetRgbCameraConfig();
  FF_REQUIRE(Ok1);
  FF_CHECK(Read.distortion_.enable_);
  FF_CHECK_NEAR(Read.distortion_.k1_, -0.2, 1.0e-9);
  FF_CHECK_NEAR(Read.distortion_.k2_, 0.05, 1.0e-9);
  FF_CHECK_NEAR(Read.distortion_.p1_, 0.001, 1.0e-9);

  // the served image stays at the nominal resolution despite the overscan render
  const auto [ImageOk, Image, Stamp, Size] = Drone->GetRgbCameraData();
  FF_CHECK(ImageOk && !Image.empty());

  Config.distortion_.enable_ = false;
  FF_CHECK(Drone->SetRgbCameraConfig(Config));
}

FF_TEST(NoiseAndRollingShutterRoundTrip)
{
  auto [Ok0, Config] = Drone->GetRgbCameraConfig();
  FF_REQUIRE(Ok0);

  Config.noise_.enable_               = true;
  Config.noise_.shot_scale_           = 3.0e-4;
  Config.noise_.read_sigma_           = 4.0e-3;
  Config.rolling_shutter_.enable_     = true;
  Config.rolling_shutter_.readout_time_ = 0.025;
  FF_REQUIRE(Drone->SetRgbCameraConfig(Config));

  const auto [Ok1, Read] = Drone->GetRgbCameraConfig();
  FF_REQUIRE(Ok1);
  FF_CHECK(Read.noise_.enable_);
  FF_CHECK_NEAR(Read.noise_.shot_scale_, 3.0e-4, 1.0e-12);
  FF_CHECK(Read.rolling_shutter_.enable_);
  FF_CHECK_NEAR(Read.rolling_shutter_.readout_time_, 0.025, 1.0e-9);

  Config.noise_.enable_           = false;
  Config.rolling_shutter_.enable_ = false;
  FF_CHECK(Drone->SetRgbCameraConfig(Config));
}

/* //} */

/* scene editor //{ */

FF_TEST(SceneEditorRoundTrip)
{
  // Counts and the exported document below are only meaningful against a known scene, and
  // anything run before this against the same simulator may have left objects behind.
  GameMode->RemoveAllSpawnedObjects();

  std::string Error;
  const auto [SpawnOk, Ids] = GameMode->SpawnObjectsFromYaml(kSceneYaml, Error);
  FF_REQUIRE(SpawnOk && Error.empty());
  FF_REQUIRE(Ids.size() == 2);

  {
    const auto [ListOk, Objects] = GameMode->ListSpawnedObjects();
    FF_REQUIRE(ListOk && Objects.size() == 2);
    FF_CHECK_NEAR(Objects[0].position[0], 5.0, 0.05);
    FF_CHECK_NEAR(Objects[0].position[2], 1.0, 0.05);
  }

  FF_CHECK(GameMode->MoveSpawnedObject(Ids[0], {6.0, -1.0, 2.0}, {0.0, 0.0, 45.0}));

  {
    const auto [ListOk, Objects] = GameMode->ListSpawnedObjects();
    FF_REQUIRE(ListOk);
    bool bFound = false;
    for (const auto& Object : Objects) {
      if (Object.id == Ids[0]) {
        bFound = true;
        FF_CHECK_NEAR(Object.position[0], 6.0, 0.05);
        FF_CHECK_NEAR(Object.position[1], -1.0, 0.05);
        FF_CHECK_NEAR(Object.orientation[2], 45.0, 0.5);
      }
    }
    FF_CHECK(bFound);
  }

  const auto [ExportOk, Yaml] = GameMode->ExportScene();
  FF_REQUIRE(ExportOk);
  FF_CHECK(Yaml.find("Cube") != std::string::npos);

  {
    const auto [RemoveOk, Removed] = GameMode->RemoveAllSpawnedObjects();
    FF_CHECK(RemoveOk && Removed == 2);
  }

  // the exported document recreates the scene, moves included
  const auto [RestoreOk, RestoredIds] = GameMode->SpawnObjectsFromYaml(Yaml, Error);
  FF_REQUIRE(RestoreOk && RestoredIds.size() == 2);

  {
    const auto [ListOk, Objects] = GameMode->ListSpawnedObjects();
    FF_REQUIRE(ListOk && !Objects.empty());
    FF_CHECK_NEAR(Objects[0].position[0], 6.0, 0.05);
  }

  GameMode->RemoveAllSpawnedObjects();
}

FF_TEST(AssetCatalog)
{
  bool bTruncated           = false;
  const auto [Ok, Names]    = GameMode->ListAssets("", "cube", bTruncated);
  FF_CHECK(Ok);
}

/* //} */

/* visibility + sensors + devices //{ */

FF_TEST(MutualVisibilityToggles)
{
  FF_CHECK(GameMode->SetMutualDroneVisibility(false));
  FF_CHECK(GameMode->SetMutualDroneVisibility(true));
}

FF_TEST(SensorLifecycle)
{
  const auto [AddOk, SensorId] = Drone->AddSensor(static_cast<int>(ueds_connector::SENSOR_RGB_CAMERA));
  FF_REQUIRE(AddOk && SensorId >= 0);

  const auto [ListOk, Sensors] = Drone->ListSensors();
  FF_REQUIRE(ListOk);

  bool bFound = false;
  for (const auto& Sensor : Sensors) {
    bFound = bFound || Sensor.id == SensorId;
  }
  FF_CHECK(bFound);

  FF_CHECK(Drone->RemoveSensor(SensorId));
}

FF_TEST(DevicesListed)
{
  const auto [Ok, Devices] = Drone->ListDevices();
  FF_REQUIRE(Ok);

  bool bFoundRealsense = false;
  bool bFoundKinect    = false;
  for (const auto& Device : Devices) {
    bFoundRealsense = bFoundRealsense || Device.name == "realsense_d435i";
    bFoundKinect    = bFoundKinect || Device.name == "azure_kinect_dk";
  }
  FF_CHECK(bFoundRealsense);
  FF_CHECK(bFoundKinect);
}

/* //} */

/* depth camera //{ */

FF_TEST(DepthCapture)
{
  std::vector<uint16_t> Depth;
  int                   Width  = 0;
  int                   Height = 0;
  double                Stamp  = 0.0;

  FF_REQUIRE(Drone->GetDepthCameraData(Depth, Width, Height, Stamp));
  FF_CHECK(Width > 0 && Height > 0);
  FF_CHECK(static_cast<int>(Depth.size()) == Width * Height);
  FF_CHECK(Stamp > 0.0);

  // over open ground something must be in range: not all zeros, not all clamp
  size_t NonZero = 0, Clamped = 0;
  for (const uint16_t Value : Depth) {
    NonZero += Value > 0 ? 1 : 0;
    Clamped += Value == 65535 ? 1 : 0;
  }
  FF_CHECK(NonZero > 0);
  FF_CHECK(Clamped < Depth.size());
}

/* //} */

/* instance segmentation //{ */

FF_TEST(InstanceSegmentationSmoke)
{
  std::string Error;
  const auto [SpawnOk, Ids] = GameMode->SpawnObjectsFromYaml(kSceneYaml, Error);
  FF_REQUIRE(SpawnOk);

  std::vector<unsigned char> Image;
  double                     Stamp = 0.0;
  const bool                 bGotImage = Drone->GetInstanceSegData(Image, Stamp);

  FF_CHECK(bGotImage);
  if (bGotImage) {
    FF_CHECK(!Image.empty());
    FF_CHECK(Stamp > 0.0);

    const auto [MapOk, Entries] = Drone->GetInstanceSegMap();
    FF_CHECK(MapOk);
  }

  GameMode->RemoveAllSpawnedObjects();
}

/* //} */

/* regressions //{ */

// Each of these covers a defect that shipped while returning a success status and a
// correctly sized buffer, which is exactly why none of them was caught earlier.

// A room the sensor sits inside, so "did the beams reach anything" has a known answer.
const std::string kRoomYaml = "objects:\n"
                              "  - asset: /Engine/BasicShapes/Cube\n"
                              "    position: [8.0, 0.0, 6.0]\n"
                              "    scale: [0.5, 16.0, 16.0]\n"
                              "  - asset: /Engine/BasicShapes/Cube\n"
                              "    position: [-8.0, 0.0, 6.0]\n"
                              "    scale: [0.5, 16.0, 16.0]\n"
                              "  - asset: /Engine/BasicShapes/Cube\n"
                              "    position: [0.0, 8.0, 6.0]\n"
                              "    scale: [16.0, 0.5, 16.0]\n"
                              "  - asset: /Engine/BasicShapes/Cube\n"
                              "    position: [0.0, -8.0, 6.0]\n"
                              "    scale: [16.0, 0.5, 16.0]\n"
                              "  - asset: /Engine/BasicShapes/Cube\n"
                              "    position: [0.0, 0.0, 14.0]\n"
                              "    scale: [16.0, 16.0, 0.5]\n"
                              "  - asset: /Engine/BasicShapes/Cube\n"
                              "    position: [0.0, 0.0, -2.0]\n"
                              "    scale: [16.0, 16.0, 0.5]\n";

constexpr double PluginDefaultBeamCm = 1000.0;

ueds_connector::LidarConfig LivoxConfig(const ueds_connector::LidarConfig& Base)
{
  ueds_connector::LidarConfig Config = Base;
  Config.Enable                      = true;
  Config.Livox                       = true;
  Config.LivoxSensor                 = "mid360";
  Config.beamLength                  = 7000.0;
  return Config;
}

// The traces used to hit the drone's own airframe first, and because that hit sits inside
// the sensor's minimum range it was dropped - so the beam was consumed and reported as
// "nothing there". Inside a closed room almost every beam must come back.
FF_TEST(LidarDoesNotSeeItsOwnAirframe)
{
  std::string Error;
  const auto [SpawnOk, Ids] = GameMode->SpawnObjectsFromYaml(kRoomYaml, Error);
  FF_REQUIRE(SpawnOk);

  Drone->SetLocationAndRotationAsync(ueds_connector::Coordinates(0.0, 0.0, 600.0), ueds_connector::Rotation(0.0, 0.0, 0.0), false);

  const auto [AddOk, SensorId] = Drone->AddSensor(static_cast<int>(ueds_connector::SENSOR_LIDAR_LIVOX));
  FF_REQUIRE(AddOk && SensorId >= 0);

  const auto [ConfigOk, Base] = Drone->GetLidarConfig(SensorId);
  FF_REQUIRE(ConfigOk);
  FF_REQUIRE(Drone->SetLidarConfig(LivoxConfig(Base), SensorId));

  Drone->GetLidarData(SensorId);  // drain the slice that spans the reconfigure

  const auto [DataOk, Points, Start, Stamp] = Drone->GetLidarData(SensorId);
  FF_REQUIRE(DataOk && !Points.empty());

  size_t Hits = 0;
  for (const auto& Point : Points) {
    Hits += Point.distance > 0.0 ? 1 : 0;
  }

  const double HitFraction = static_cast<double>(Hits) / static_cast<double>(Points.size());
  FF_CHECK(HitFraction > 0.5);

  Drone->RemoveSensor(SensorId);
  GameMode->RemoveAllSpawnedObjects();

  // put the drone back where the other tests expect it
  Drone->SetLocationAndRotationAsync(ueds_connector::Coordinates(0.0, 0.0, 200.0), ueds_connector::Rotation(0.0, 0.0, 0.0), false);
}

// Every Livox cursor used to start at pattern position zero, so two of them created
// together walked the pattern in lockstep and returned the same directions.
FF_TEST(TwoLivoxesDoNotShareAPhase)
{
  const auto [AddA, SensorA] = Drone->AddSensor(static_cast<int>(ueds_connector::SENSOR_LIDAR_LIVOX));
  const auto [AddB, SensorB] = Drone->AddSensor(static_cast<int>(ueds_connector::SENSOR_LIDAR_LIVOX));
  FF_REQUIRE(AddA && AddB && SensorA != SensorB);

  for (const int SensorId : {SensorA, SensorB}) {
    const auto [ConfigOk, Base] = Drone->GetLidarConfig(SensorId);
    FF_REQUIRE(ConfigOk);
    FF_REQUIRE(Drone->SetLidarConfig(LivoxConfig(Base), SensorId));
  }

  const auto [OkA, PointsA, StartA, StampA] = Drone->GetLidarData(SensorA);
  const auto [OkB, PointsB, StartB, StampB] = Drone->GetLidarData(SensorB);
  FF_REQUIRE(OkA && OkB);
  FF_REQUIRE(!PointsA.empty() && !PointsB.empty());

  const size_t Compared = std::min<size_t>(2000, std::min(PointsA.size(), PointsB.size()));
  size_t       Same     = 0;
  for (size_t i = 0; i < Compared; ++i) {
    const bool bSameDirection = std::fabs(PointsA[i].directionX - PointsB[i].directionX) < 1e-6 &&
                                std::fabs(PointsA[i].directionY - PointsB[i].directionY) < 1e-6 &&
                                std::fabs(PointsA[i].directionZ - PointsB[i].directionZ) < 1e-6;
    Same += bSameDirection ? 1 : 0;
  }

  // identical directions across the whole prefix means one shared playback position
  FF_CHECK(Same < Compared);

  Drone->RemoveSensor(SensorA);
  Drone->RemoveSensor(SensorB);
}

// The segmentation camera silently degraded to an ordinary colour camera whenever its
// post-process material was missing: same resolution, same byte count, plausible picture.
// A class mask has a handful of flat colours; a photograph has thousands.
FF_TEST(SegmentationReturnsAMaskNotAPhotograph)
{
  const auto [AddOk, SensorId] = Drone->AddSensor(static_cast<int>(ueds_connector::SENSOR_RGB_SEG_CAMERA));
  FF_REQUIRE(AddOk && SensorId >= 0);

  const auto [ConfigOk, Config] = Drone->GetRgbCameraConfig(SensorId);
  FF_REQUIRE(ConfigOk);

  // A freshly created capture serves photographs for a few frames: the custom-stencil
  // input is empty until the renderer has been through it once, and the material passes
  // scene colour through for unlabelled pixels. Steady state, not the boot transient,
  // is what this test is about.
  size_t     ColourCount = 0;
  const auto Deadline    = std::chrono::steady_clock::now() + std::chrono::seconds(3);

  while (std::chrono::steady_clock::now() < Deadline) {

    const auto [DataOk, Raw, Stamp, Size] = Drone->GetRgbSegmented(SensorId);
    FF_REQUIRE(DataOk);
    FF_REQUIRE(Raw.size() >= static_cast<size_t>(Config.width_) * Config.height_ * 3);

    std::vector<uint32_t> Colours;
    Colours.reserve(4096);
    for (size_t i = 0; i + 2 < Raw.size(); i += 3) {
      const uint32_t Packed = (static_cast<uint32_t>(Raw[i]) << 16) | (static_cast<uint32_t>(Raw[i + 1]) << 8) | Raw[i + 2];
      Colours.push_back(Packed);
    }

    std::sort(Colours.begin(), Colours.end());
    Colours.erase(std::unique(Colours.begin(), Colours.end()), Colours.end());
    ColourCount = Colours.size();

    if (ColourCount > 1 && ColourCount < 512) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  FF_CHECK(ColourCount > 1);     // a single colour means nothing is being labelled
  FF_CHECK(ColourCount < 512);   // thousands means this is scene colour, not a mask

  Drone->RemoveSensor(SensorId);
}

// Clearing a scene that is already empty is the requested state, not a failure; reporting
// otherwise silently broke every reset that happened to run twice.
FF_TEST(ClearingAnEmptySceneSucceeds)
{
  GameMode->RemoveAllSpawnedObjects();

  const auto [Ok, Removed] = GameMode->RemoveAllSpawnedObjects();
  FF_CHECK(Ok);
  FF_CHECK(Removed == 0);
}

// A slice covers the time since the last request. Without a bound, a client that pauses
// gets back everything that accumulated - which both smears the scan and takes longer to
// serve than the span it represents, so the next poll is later still.
FF_TEST(LivoxBacklogStaysBounded)
{
  const auto [AddOk, SensorId] = Drone->AddSensor(static_cast<int>(ueds_connector::SENSOR_LIDAR_LIVOX));
  FF_REQUIRE(AddOk);

  const auto [ConfigOk, Base] = Drone->GetLidarConfig(SensorId);
  FF_REQUIRE(ConfigOk);

  ueds_connector::LidarConfig Config = LivoxConfig(Base);
  Config.Frequency                   = 10.0;
  FF_REQUIRE(Drone->SetLidarConfig(Config, SensorId));

  Drone->GetLidarData(SensorId);

  // stay away long enough that an unbounded slice would be enormous
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  const auto [DataOk, Points, Start, Stamp] = Drone->GetLidarData(SensorId);
  FF_REQUIRE(DataOk);

  // 200 kHz over the 2-frame bound is 40k points; a whole second would be 200k
  FF_CHECK(Points.size() <= 60000);

  Drone->RemoveSensor(SensorId);
}

// The pattern file carries the sensor's real reach, and nothing read it: an Avia came up
// with the generic 10 m beam, which looks exactly like a working sensor that sees nothing
// far away. Selecting a pattern must lift the beam to the sensor's own range.
FF_TEST(LivoxPatternSetsTheBeamLength)
{
  const auto [AddOk, SensorId] = Drone->AddSensor(static_cast<int>(ueds_connector::SENSOR_LIDAR_LIVOX));
  FF_REQUIRE(AddOk);

  const auto [ConfigOk, Base] = Drone->GetLidarConfig(SensorId);
  FF_REQUIRE(ConfigOk);

  struct Expectation
  {
    const char* Sensor;
    double      MinBeamCm;
  };

  for (const Expectation& Case : {Expectation{"avia", 40000.0}, Expectation{"mid360", 6000.0}}) {

    ueds_connector::LidarConfig Config = Base;
    Config.Enable                      = true;
    Config.Livox                       = true;
    Config.LivoxSensor                 = Case.Sensor;
    Config.beamLength                  = PluginDefaultBeamCm;
    FF_REQUIRE(Drone->SetLidarConfig(Config, SensorId));

    const auto [ReadOk, Applied] = Drone->GetLidarConfig(SensorId);
    FF_REQUIRE(ReadOk);
    FF_CHECK(Applied.beamLength >= Case.MinBeamCm);
  }

  Drone->RemoveSensor(SensorId);
}

// Coarse guards, not benchmarks. A 2 ms sleep once sat on every reply, putting a 2.1 ms
// floor under the pose read a control loop makes most; these thresholds are loose enough
// to survive a busy machine but would catch that class of regression.
FF_TEST(RequestLatencyStaysLow)
{
  std::vector<double> Samples;
  for (int i = 0; i < 20; ++i) {
    const auto Start = std::chrono::steady_clock::now();
    Drone->GetLocation();
    Samples.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count());
  }

  std::sort(Samples.begin(), Samples.end());
  const double Median = Samples[Samples.size() / 2];

  std::cout << "    pose read median " << Median << " ms" << std::endl;
  FF_CHECK(Median < 1.0);
}

/* //} */

}  // namespace

int main()
{
  GameMode = std::make_unique<ueds_connector::GameModeController>(kAddress, kGameModePort);
  if (!GameMode->ConnectSimple()) {
    std::cout << "SKIP: no simulator reachable on " << kAddress << ":" << kGameModePort << std::endl;
    return FF_SKIP_EXIT_CODE;
  }

  ueds_connector::Coordinates SpawnLocation(0.0, 0.0, 200.0);
  std::string                 Frame          = "x500";
  const auto [SpawnOk, Port]                 = GameMode->SpawnDroneAtLocation(SpawnLocation, Frame);
  if (!SpawnOk) {
    std::cerr << "FAIL: could not spawn a drone" << std::endl;
    return EXIT_FAILURE;
  }
  DronePort = Port;

  Drone = std::make_unique<ueds_connector::UedsConnector>(kAddress, DronePort);
  if (!Drone->ConnectSimple()) {
    std::cerr << "FAIL: could not connect to the spawned drone on port " << DronePort << std::endl;
    return EXIT_FAILURE;
  }

  const int Result = fftest::RunAll();

  GameMode->RemoveDrone(DronePort);
  Drone->Disconnect();
  GameMode->Disconnect();

  return Result;
}
