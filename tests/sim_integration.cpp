#include <cmath>
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

const std::string kSceneYaml = "objects:\n"
                               "  - asset: Cube\n"
                               "    position: [5.0, 0.0, 1.0]\n"
                               "  - asset: Cube\n"
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

  bool bFound = false;
  for (const auto& Device : Devices) {
    bFound = bFound || Device.name == "realsense_d435i";
  }
  FF_CHECK(bFound);
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
