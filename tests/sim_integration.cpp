#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <flight_forge_connector/robot_client.h>
#include <flight_forge_connector/simulator_client.h>

#include "test_framework.h"

namespace
{

using namespace flightforge;
using Clock = std::chrono::steady_clock;

const std::string kAddress = "127.0.0.1";
uint16_t          Port     = Connection::kDefaultPort;

std::unique_ptr<SimulatorClient> Sim;
std::unique_ptr<RobotClient>     Drone;

constexpr double kDeg = M_PI / 180.0;

const Pose kHome = Pose::At(0.0, 0.0, 2.0);

// an /Engine path exists in every install, so nothing depends on the host project's content
const std::string kSceneYaml = "objects:\n"
                               "  - asset: /Engine/BasicShapes/Cube\n"
                               "    position: [5.0, 0.0, 1.0]\n"
                               "  - asset: /Engine/BasicShapes/Cube\n"
                               "    position: [5.0, 2.0, 1.0]\n"
                               "    scale: 0.5\n";

/* helpers //{ */

bool Finite(const std::array<float, 3>& p)
{
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

double Distance(const Vec3& a, const Vec3& b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

std::unique_ptr<wire::ImageEncodingT> Encoding(const wire::Compression compression)
{
  auto encoding         = std::make_unique<wire::ImageEncodingT>();
  encoding->compression = compression;
  return encoding;
}

// width and height from a baseline or progressive JPEG's frame header
std::pair<int, int> JpegSize(const std::vector<uint8_t>& jpeg)
{
  for (size_t i = 2; i + 8 < jpeg.size();) {
    if (jpeg[i] != 0xFF) {
      return {0, 0};
    }
    const uint8_t marker = jpeg[i + 1];
    const size_t  length = (jpeg[i + 2] << 8) | jpeg[i + 3];
    if (marker == 0xC0 || marker == 0xC2) {
      return {(jpeg[i + 7] << 8) | jpeg[i + 8], (jpeg[i + 5] << 8) | jpeg[i + 6]};
    }
    i += 2 + length;
  }
  return {0, 0};
}

wire::LidarConfigT LivoxConfig(const wire::LidarConfigT& base, const std::string& pattern = "mid360")
{
  wire::LidarConfigT config = base;
  config.enable             = true;
  config.livox              = true;
  config.livox_pattern      = pattern;
  config.range_m            = 70.0;
  return config;
}

std::shared_ptr<Connection> NewConnection()
{
  auto connection = std::make_shared<Connection>(kAddress, Port);
  connection->Connect(3);
  return connection;
}

/* //} */

/* basics //{ */

FF_TEST(ApiVersion)
{
  const auto info = Sim->GetInfo();
  FF_REQUIRE(info.ok);
  FF_CHECK(info->major == 1 && info->minor == 0);
}

// the simulator speaks exactly the schema this client was generated from
FF_TEST(SchemaMatchesThisBuild)
{
  const auto info = Sim->GetInfo();
  FF_REQUIRE(info.ok);

  std::ifstream              file(FF_WIRE_BFBS, std::ios::binary);
  const std::vector<uint8_t> local((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  FF_CHECK(!local.empty());
  FF_CHECK(info->schema == local);
}

FF_TEST(FpsReports)
{
  const auto fps = Sim->GetFps();
  FF_CHECK(fps.ok && *fps > 0.0f);
}

// raw frames skip the codec; the first frames of a new camera can still be black
FF_TEST(RgbCapture)
{
  const auto config = Drone->GetRgbCameraConfig();
  FF_REQUIRE(config.ok);

  wire::RgbCameraConfigT raw = *config;
  raw.encoding               = Encoding(wire::Compression::NONE);
  FF_REQUIRE(Drone->SetRgbCameraConfig(raw).ok);

  bool lit = false;
  for (int attempt = 0; attempt < 40 && !lit; ++attempt) {
    const auto image = Drone->GetRgbCameraData();
    FF_REQUIRE(image.ok);
    FF_REQUIRE(image->compression == wire::Compression::NONE && image->format == wire::PixelFormat::BGRA8);
    FF_REQUIRE(image->data.size() == size_t(image->width) * image->height * 4);
    FF_CHECK(image->stamp > 0.0);
    for (size_t i = 0; i + 3 < image->data.size() && !lit; i += 4) {
      lit = image->data[i] || image->data[i + 1] || image->data[i + 2];
    }
    if (!lit) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  FF_CHECK(lit);

  wire::RgbCameraConfigT jpeg = *config;
  jpeg.encoding               = Encoding(wire::Compression::JPEG);
  FF_REQUIRE(Drone->SetRgbCameraConfig(jpeg).ok);
  const auto image = Drone->GetRgbCameraData();
  FF_REQUIRE(image.ok);
  FF_CHECK(image->compression == wire::Compression::JPEG && image->format == wire::PixelFormat::RGB8);
  FF_CHECK(image->data.size() > 2 && image->data[0] == 0xFF && image->data[1] == 0xD8);
  FF_CHECK(JpegSize(image->data) == std::make_pair(int(image->width), int(image->height)));
}

/* //} */

/* camera calibration round-trips //{ */

FF_TEST(IntrinsicsRoundTrip)
{
  auto config = Drone->GetRgbCameraConfig();
  FF_REQUIRE(config.ok && config->intrinsics);

  // the FOV path publishes a centred effective K
  FF_CHECK(config->intrinsics->fx > 0.0);
  FF_CHECK_NEAR(config->intrinsics->cx, config->width / 2.0, 1.0);

  config->intrinsics->use_custom = true;
  config->intrinsics->fx         = 500.0;
  config->intrinsics->fy         = 505.0;
  config->intrinsics->cx         = 300.0;
  config->intrinsics->cy         = 250.0;
  FF_REQUIRE(Drone->SetRgbCameraConfig(*config).ok);

  const auto read = Drone->GetRgbCameraConfig();
  FF_REQUIRE(read.ok && read->intrinsics);
  FF_CHECK_NEAR(read->intrinsics->fx, 500.0, 1.0e-9);
  FF_CHECK_NEAR(read->intrinsics->fy, 505.0, 1.0e-9);
  FF_CHECK_NEAR(read->intrinsics->cx, 300.0, 1.0e-9);
  FF_CHECK_NEAR(read->intrinsics->cy, 250.0, 1.0e-9);
}

FF_TEST(DistortionRoundTrip)
{
  auto config = Drone->GetRgbCameraConfig();
  FF_REQUIRE(config.ok && config->distortion);

  config->distortion->enable = true;
  config->distortion->k1     = -0.2;
  config->distortion->k2     = 0.05;
  config->distortion->p1     = 0.001;
  FF_REQUIRE(Drone->SetRgbCameraConfig(*config).ok);

  const auto read = Drone->GetRgbCameraConfig();
  FF_REQUIRE(read.ok && read->distortion);
  FF_CHECK(read->distortion->enable);
  FF_CHECK_NEAR(read->distortion->k1, -0.2, 1.0e-9);
  FF_CHECK_NEAR(read->distortion->k2, 0.05, 1.0e-9);
  FF_CHECK_NEAR(read->distortion->p1, 0.001, 1.0e-9);

  // the served image stays at the nominal resolution despite the overscan render
  const auto image = Drone->GetRgbCameraData();
  FF_CHECK(image.ok && !image->data.empty());
  FF_CHECK(image.ok && image->width == read->width && image->height == read->height);

  config->distortion->enable = false;
  FF_CHECK(Drone->SetRgbCameraConfig(*config).ok);
}

FF_TEST(NoiseAndRollingShutterRoundTrip)
{
  auto config = Drone->GetRgbCameraConfig();
  FF_REQUIRE(config.ok && config->noise && config->rolling_shutter);

  config->noise->enable                = true;
  config->noise->shot_scale            = 3.0e-4;
  config->noise->read_sigma            = 4.0e-3;
  config->rolling_shutter->enable       = true;
  config->rolling_shutter->readout_time = 0.025;
  FF_REQUIRE(Drone->SetRgbCameraConfig(*config).ok);

  const auto read = Drone->GetRgbCameraConfig();
  FF_REQUIRE(read.ok && read->noise && read->rolling_shutter);
  FF_CHECK(read->noise->enable);
  FF_CHECK_NEAR(read->noise->shot_scale, 3.0e-4, 1.0e-12);
  FF_CHECK(read->rolling_shutter->enable);
  FF_CHECK_NEAR(read->rolling_shutter->readout_time, 0.025, 1.0e-9);

  config->noise->enable           = false;
  config->rolling_shutter->enable = false;
  FF_CHECK(Drone->SetRgbCameraConfig(*config).ok);
}

/* //} */

/* scene editor //{ */

FF_TEST(GroundFrameIsTheWireFrame)
{
  Sim->RemoveAllSpawnedObjects();

  const auto ids = Sim->SpawnObjectsFromYaml("frame: ground\nobjects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [5.0, 0.0, 1.0]\n");
  FF_REQUIRE(ids.ok && ids->size() == 1);

  const auto objects = Sim->ListSpawnedObjects();
  FF_REQUIRE(objects.ok && objects->size() == 1);
  FF_CHECK_NEAR((*objects)[0].pose.position.x, 5.0, 0.01);
  FF_CHECK_NEAR((*objects)[0].pose.position.z, 1.0, 0.01);

  const auto yaml = Sim->ExportScene();
  FF_REQUIRE(yaml.ok);
  FF_CHECK(yaml->find("frame: ground") != std::string::npos);

  Sim->RemoveAllSpawnedObjects();
  const auto restored = Sim->SpawnObjectsFromYaml(*yaml);
  FF_REQUIRE(restored.ok && restored->size() == 1);
  const auto relisted = Sim->ListSpawnedObjects();
  FF_REQUIRE(relisted.ok && relisted->size() == 1);
  FF_CHECK_NEAR((*relisted)[0].pose.position.z, 1.0, 0.01);
  Sim->RemoveAllSpawnedObjects();

  // frame: unreal is absolute Unreal cm, and Unreal -Y is ground +y
  const auto origin = Sim->GetWorldOrigin();
  FF_REQUIRE(origin.ok);
  const std::string unreal = "frame: unreal\nobjects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [" + std::to_string(origin->x + 500.0) + ", " +
                             std::to_string(origin->y - 200.0) + ", " + std::to_string(origin->z + 100.0) + "]\n";
  FF_REQUIRE(Sim->SpawnObjectsFromYaml(unreal).ok);
  const auto placed = Sim->ListSpawnedObjects();
  FF_REQUIRE(placed.ok && placed->size() == 1);
  FF_CHECK_NEAR((*placed)[0].pose.position.x, 5.0, 0.01);
  FF_CHECK_NEAR((*placed)[0].pose.position.y, 2.0, 0.01);
  FF_CHECK_NEAR((*placed)[0].pose.position.z, 1.0, 0.01);
  Sim->RemoveAllSpawnedObjects();

  // the old frames are refused, naming what replaces them
  const auto origin_frame = Sim->SpawnObjectsFromYaml("frame: origin\nobjects:\n  - asset: /Engine/BasicShapes/Cube\n");
  FF_CHECK(!origin_frame.ok && origin_frame.error.find("ground") != std::string::npos);
  const auto world_frame = Sim->SpawnObjectsFromYaml("frame: world\nobjects:\n  - asset: /Engine/BasicShapes/Cube\n");
  FF_CHECK(!world_frame.ok && world_frame.error.find("unreal") != std::string::npos);
  const auto nothing = Sim->ListSpawnedObjects();
  FF_CHECK(nothing.ok && nothing->empty());
}

FF_TEST(LibraryPlacesAtItsScaleOnItsBase)
{
  const std::string path = "/tmp/ff_test_library.yaml";

  std::ofstream(path) << "name: test\n"
                         "objects:\n"
                         "  - id: crate\n"
                         "    asset: /Engine/BasicShapes/Cube\n"
                         "    class: box\n"
                         "    description: a wooden crate\n"
                         "    scale: 2.0\n"
                         "    height: 2.0\n"
                         "    base_z: 0.5\n";

  const auto count = Sim->LoadLibrary(path);
  FF_REQUIRE(count.ok);
  FF_CHECK(*count == 1);

  const auto found = Sim->ListLibrary("test", "", "WOODEN", 0, 10);
  FF_REQUIRE(found.ok && found->objects.size() == 1);
  FF_CHECK(found->total == 1);
  FF_CHECK(found->objects[0].spawn_name == "lib:test/crate");

  FF_CHECK(Sim->ListLibrary("project", "", "", 0, 1).ok);

  Sim->RemoveAllSpawnedObjects();

  const auto ids = Sim->SpawnObjectsFromYaml("frame: ground\nobjects:\n  - asset: lib:test/crate\n    position: [5.0, 0.0, 0.0]\n");
  FF_REQUIRE(ids.ok && ids->size() == 1);

  // listed poses name the base, on the floor at ground z = 0
  const auto objects = Sim->ListSpawnedObjects();
  FF_REQUIRE(objects.ok && objects->size() == 1);
  FF_CHECK((*objects)[0].asset == "lib:test/crate");
  FF_CHECK_NEAR((*objects)[0].scale.z, 2.0, 0.001);
  FF_CHECK_NEAR((*objects)[0].pose.position.z, 0.0, 0.01);

  const auto yaml = Sim->ExportScene();
  FF_REQUIRE(yaml.ok);

  Sim->RemoveAllSpawnedObjects();

  const auto restored = Sim->SpawnObjectsFromYaml(*yaml);
  FF_REQUIRE(restored.ok && restored->size() == 1);

  const auto relisted = Sim->ListSpawnedObjects();
  FF_REQUIRE(relisted.ok && relisted->size() == 1);
  FF_CHECK_NEAR((*relisted)[0].pose.position.z, 0.0, 0.01);

  Sim->RemoveAllSpawnedObjects();

  const auto missing = Sim->SpawnObjectsFromYaml("objects:\n  - asset: lib:test/missing\n");
  FF_CHECK(!missing.ok || missing->empty());
}

FF_TEST(SceneEditorRoundTrip)
{
  // counts and the exported document only mean something against a known scene
  Sim->RemoveAllSpawnedObjects();

  const auto ids = Sim->SpawnObjectsFromYaml(kSceneYaml);
  FF_REQUIRE(ids.ok && ids.error.empty());
  FF_REQUIRE(ids->size() == 2);

  {
    const auto objects = Sim->ListSpawnedObjects();
    FF_REQUIRE(objects.ok && objects->size() == 2);
    FF_CHECK_NEAR((*objects)[0].pose.position.x, 5.0, 0.05);
    FF_CHECK_NEAR((*objects)[0].pose.position.z, 1.0, 0.05);
  }

  FF_CHECK(Sim->MoveSpawnedObject((*ids)[0], Pose::At(6.0, -1.0, 2.0, 45.0 * kDeg)).ok);

  {
    const auto objects = Sim->ListSpawnedObjects();
    FF_REQUIRE(objects.ok);
    bool found = false;
    for (const auto& object : *objects) {
      if (object.id == (*ids)[0]) {
        found = true;
        FF_CHECK_NEAR(object.pose.position.x, 6.0, 0.05);
        FF_CHECK_NEAR(object.pose.position.y, -1.0, 0.05);
        FF_CHECK_NEAR(object.pose.orientation.Yaw(), 45.0 * kDeg, 0.01);
      }
    }
    FF_CHECK(found);
  }

  const auto yaml = Sim->ExportScene();
  FF_REQUIRE(yaml.ok);
  FF_CHECK(yaml->find("Cube") != std::string::npos);

  {
    const auto removed = Sim->RemoveAllSpawnedObjects();
    FF_CHECK(removed.ok && *removed == 2);
  }

  // the exported document recreates the scene, moves included
  const auto restored = Sim->SpawnObjectsFromYaml(*yaml);
  FF_REQUIRE(restored.ok && restored->size() == 2);

  {
    const auto objects = Sim->ListSpawnedObjects();
    FF_REQUIRE(objects.ok && !objects->empty());
    FF_CHECK_NEAR((*objects)[0].pose.position.x, 6.0, 0.05);
  }

  Sim->RemoveAllSpawnedObjects();
}

// an on-disk model imports off the game thread, so this checks the reply still waits for its collision
FF_TEST(ModelFileCollidesOnceSpawned)
{
  Sim->RemoveAllSpawnedObjects();

  FF_REQUIRE(Drone->SetPose(Pose::At(0.0, 0.0, 6.0)).ok);

  const auto before = Drone->GetRangefinderData();
  FF_REQUIRE(before.ok);
  FF_REQUIRE(std::isnan(before->range) || before->range > 4.0);

  const auto ids = Sim->SpawnObjectsFromYaml("objects:\n  - asset: Models/helhest.glb\n    position: [0.0, 0.0, 3.0]\n");
  FF_REQUIRE(ids.ok && ids->size() == 1);

  const auto after = Drone->GetRangefinderData();
  FF_REQUIRE(after.ok);
  FF_CHECK(std::isfinite(after->range) && after->range < 3.0);

  Sim->RemoveAllSpawnedObjects();
  Drone->SetPose(kHome);
}

FF_TEST(AssetCatalog)
{
  FF_CHECK(Sim->ListAssets("", "cube").ok);
}

/* //} */

/* visibility + sensors + devices //{ */

FF_TEST(MutualVisibilityToggles)
{
  FF_CHECK(Sim->SetMutualVisibility(false).ok);
  FF_CHECK(Sim->SetMutualVisibility(true).ok);
}

FF_TEST(SensorLifecycle)
{
  const auto id = Drone->AddSensor(wire::SensorType::RGB_CAMERA);
  FF_REQUIRE(id.ok && *id >= 0);

  const auto sensors = Drone->ListSensors();
  FF_REQUIRE(sensors.ok);

  bool found = false;
  for (const auto& sensor : *sensors) {
    found = found || (sensor.id == *id && sensor.type == wire::SensorType::RGB_CAMERA && sensor.link == "base_link");
  }
  FF_CHECK(found);

  FF_CHECK(Drone->RemoveSensor(*id).ok);
  FF_CHECK(!Drone->RemoveSensor(*id).ok);
}

FF_TEST(CameraInfoMatchesTheConfigs)
{
  // the default RGB camera: K equals what GetRgbCameraConfig reports as effective intrinsics
  const auto config = Drone->GetRgbCameraConfig();
  FF_REQUIRE(config.ok && config->intrinsics && config->mount);

  const auto cameras = Drone->GetCameraInfo();
  FF_REQUIRE(cameras.ok && !cameras->empty());

  const wire::CameraInfoT* rgb = nullptr;
  for (const auto& camera : *cameras) {
    if (camera.sensor_type == wire::SensorType::RGB_CAMERA && (!rgb || camera.sensor_id < rgb->sensor_id)) {
      rgb = &camera;
    }
  }
  FF_REQUIRE(rgb != nullptr && rgb->mount);
  FF_CHECK(rgb->projection == wire::Projection::PINHOLE && rgb->distortion_model == "plumb_bob");
  FF_CHECK(rgb->width == config->width && rgb->height == config->height);
  FF_CHECK_NEAR(rgb->fx, config->intrinsics->fx, 1.0e-6);
  FF_CHECK_NEAR(rgb->cx, config->intrinsics->cx, 1.0e-6);
  if (!config->intrinsics->use_custom) {
    FF_CHECK_NEAR(rgb->fov, config->fov, 1.0e-6);
    FF_CHECK_NEAR(rgb->cx, config->width / 2.0 - 0.5, 1.0e-9);  // OpenCV pixel convention
  }
  FF_CHECK_NEAR(rgb->mount->position().x(), config->mount->position().x(), 1.0e-9);
  FF_CHECK_NEAR(rgb->mount->orientation().w(), config->mount->orientation().w(), 1.0e-9);
  FF_CHECK(rgb->link == "base_link");
  FF_CHECK(rgb->d.size() == 5);

  // a single id returns just that camera, an unknown id fails
  const auto one = Drone->GetCameraInfo(rgb->sensor_id);
  FF_CHECK(one.ok && one->size() == 1 && (*one)[0].sensor_id == rgb->sensor_id);
  FF_CHECK(!Drone->GetCameraInfo(9999).ok);

  // a stereo pair comes back as two eyes sharing K, with the baseline between the mounts
  const auto stereo = Drone->AddSensor(wire::SensorType::STEREO_CAMERA);
  FF_REQUIRE(stereo.ok);
  auto stereo_config = Drone->GetStereoCameraConfig(*stereo);
  FF_REQUIRE(stereo_config.ok && stereo_config->mount_left);
  const wire::Pose left = *stereo_config->mount_left;
  stereo_config->mount_right =
      std::make_unique<wire::Pose>(wire::Vec3(left.position().x(), left.position().y() - 0.12, left.position().z()), left.orientation());
  FF_REQUIRE(Drone->SetStereoCameraConfig(*stereo_config, *stereo).ok);
  const auto pair = Drone->GetCameraInfo(*stereo);
  FF_CHECK(pair.ok && pair->size() == 2);
  if (pair.ok && pair->size() == 2) {
    const auto& l = (*pair)[0];
    const auto& r = (*pair)[1];
    FF_CHECK(l.eye == wire::Eye::LEFT && r.eye == wire::Eye::RIGHT);
    FF_CHECK(l.fx == r.fx && l.baseline_m == r.baseline_m);
    FF_CHECK_NEAR(l.baseline_m, 0.12, 1.0e-6);
    FF_CHECK_NEAR(l.mount->position().y() - r.mount->position().y(), 0.12, 1.0e-6);
  }
  FF_CHECK(Drone->RemoveSensor(*stereo).ok);

  // a fisheye reports its lens model and a focal that puts theta = fov/2 on the image circle
  const auto fisheye = Drone->AddSensor(wire::SensorType::FISHEYE_CAMERA);
  FF_REQUIRE(fisheye.ok);
  const auto fish_config = Drone->GetFisheyeCameraConfig(*fisheye);
  const auto fish        = Drone->GetCameraInfo(*fisheye);
  FF_REQUIRE(fish_config.ok && fish.ok && fish->size() == 1 && fish_config->intrinsics);
  const auto& info = (*fish)[0];
  FF_CHECK(info.distortion_model == "equidistant" && info.d.size() == 4);
  if (info.projection == wire::Projection::FISHEYE_EQUIDISTANT && info.d.size() == 4) {
    const double half_fov = fish_config->fov * M_PI / 360.0;
    const double radius   = fish_config->full_frame
                                ? 0.5 * std::sqrt(double(fish_config->width) * fish_config->width + double(fish_config->height) * fish_config->height)
                                : 0.5 * std::min(fish_config->width, fish_config->height);
    FF_CHECK_NEAR(info.fx * half_fov, radius, 1.0e-3);
    FF_CHECK(info.d[0] == 0.0 && info.d[3] == 0.0);
  }
  FF_CHECK_NEAR(info.fx, fish_config->intrinsics->fx, 1.0e-9);

  // a Kannala-Brandt calibration goes in and comes back verbatim
  wire::FisheyeCameraConfigT kb = *fish_config;
  kb.lens                       = wire::FisheyeLens::KANNALA_BRANDT;
  kb.intrinsics->use_custom     = true;
  kb.intrinsics->fx             = 254.3;
  kb.intrinsics->fy             = 252.9;
  kb.intrinsics->cx             = 405.7;
  kb.intrinsics->cy             = 291.2;
  kb.kb_k1                      = -0.012;
  kb.kb_k2                      = 0.021;
  kb.kb_k3                      = -0.017;
  kb.kb_k4                      = 0.004;
  FF_REQUIRE(Drone->SetFisheyeCameraConfig(kb, *fisheye).ok);
  const auto kb_info = Drone->GetCameraInfo(*fisheye);
  FF_REQUIRE(kb_info.ok && kb_info->size() == 1);
  FF_CHECK((*kb_info)[0].projection == wire::Projection::FISHEYE_KANNALA_BRANDT);
  FF_CHECK_NEAR((*kb_info)[0].fx, 254.3, 1.0e-9);
  FF_CHECK_NEAR((*kb_info)[0].fy, 252.9, 1.0e-9);
  FF_CHECK_NEAR((*kb_info)[0].cx, 405.7, 1.0e-9);
  FF_CHECK_NEAR((*kb_info)[0].cy, 291.2, 1.0e-9);
  FF_CHECK((*kb_info)[0].d.size() == 4 && (*kb_info)[0].d[0] == -0.012 && (*kb_info)[0].d[3] == 0.004);
  const auto kb_config = Drone->GetFisheyeCameraConfig(*fisheye);
  FF_CHECK(kb_config.ok && kb_config->intrinsics->use_custom && kb_config->kb_k2 == 0.021 && std::abs(kb_config->intrinsics->cx - 405.7) < 1.0e-9);
  FF_CHECK(Drone->RemoveSensor(*fisheye).ok);
}

FF_TEST(DevicesListed)
{
  const auto devices = Drone->ListDevices();
  FF_REQUIRE(devices.ok);

  bool realsense = false;
  bool kinect    = false;
  for (const auto& device : *devices) {
    realsense = realsense || device.name == "realsense_d435i";
    kinect    = kinect || device.name == "azure_kinect_dk";
  }
  FF_CHECK(realsense);
  FF_CHECK(kinect);
}

/* //} */

/* depth camera //{ */

FF_TEST(DepthCapture)
{
  const auto depth = Drone->GetDepthCameraData();
  FF_REQUIRE(depth.ok);
  FF_CHECK(depth->width > 0 && depth->height > 0);
  FF_CHECK(depth->mm.size() == size_t(depth->width) * depth->height);
  FF_CHECK(depth->stamp > 0.0);

  // over open ground something must be in range: not all zeros, not all clamp
  size_t non_zero = 0, clamped = 0;
  for (const uint16_t value : depth->mm) {
    non_zero += value > 0 ? 1 : 0;
    clamped += value == 65535 ? 1 : 0;
  }
  FF_CHECK(non_zero > 0);
  FF_CHECK(clamped < depth->mm.size());
}

/* //} */

/* instance segmentation //{ */

FF_TEST(InstanceSegmentationSmoke)
{
  const auto ids = Sim->SpawnObjectsFromYaml(kSceneYaml);
  FF_REQUIRE(ids.ok);

  const auto image = Drone->GetInstanceSegData();
  FF_CHECK(image.ok);
  if (image.ok) {
    FF_CHECK(!image->data.empty());
    FF_CHECK(image->stamp > 0.0);
    FF_CHECK(image->compression == wire::Compression::PNG && image->format == wire::PixelFormat::RGBA8);
    FF_CHECK(Drone->GetInstanceSegMap().ok);
  }

  Sim->RemoveAllSpawnedObjects();
}

/* //} */

/* regressions //{ */

// Each of these covers a defect that shipped while returning a success status and a
// correctly sized buffer, which is exactly why none of them was caught earlier.

// a room the sensor sits inside, so "did the beams reach anything" has a known answer
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

// the traces used to hit the airframe first, inside the minimum range, and report nothing there
FF_TEST(LidarDoesNotSeeItsOwnAirframe)
{
  const auto ids = Sim->SpawnObjectsFromYaml(kRoomYaml);
  FF_REQUIRE(ids.ok);

  FF_REQUIRE(Drone->SetPose(Pose::At(0.0, 0.0, 6.0)).ok);

  const auto id = Drone->AddSensor(wire::SensorType::LIDAR);
  FF_REQUIRE(id.ok);

  const auto base = Drone->GetLidarConfig(*id);
  FF_REQUIRE(base.ok);
  FF_REQUIRE(Drone->SetLidarConfig(LivoxConfig(*base), *id).ok);

  Drone->GetLidarScan(false, false, *id);  // drain the slice that spans the reconfigure

  const auto scan = Drone->GetLidarScan(false, false, *id);
  FF_REQUIRE(scan.ok && !scan->points.empty());
  FF_CHECK(scan->rows == 0 && scan->cols == 0);

  const size_t hits = std::count_if(scan->points.begin(), scan->points.end(), Finite);
  FF_CHECK(double(hits) / double(scan->points.size()) > 0.5);

  Drone->RemoveSensor(*id);
  Sim->RemoveAllSpawnedObjects();
  Drone->SetPose(kHome);
}

// every Livox cursor once started at pattern position zero, so two walked in lockstep
FF_TEST(TwoLivoxesDoNotShareAPhase)
{
  const auto a = Drone->AddSensor(wire::SensorType::LIDAR);
  const auto b = Drone->AddSensor(wire::SensorType::LIDAR);
  FF_REQUIRE(a.ok && b.ok && *a != *b);

  for (const int id : {*a, *b}) {
    const auto base = Drone->GetLidarConfig(id);
    FF_REQUIRE(base.ok);
    FF_REQUIRE(Drone->SetLidarConfig(LivoxConfig(*base), id).ok);
  }

  const auto scan_a = Drone->GetLidarScan(false, false, *a);
  const auto scan_b = Drone->GetLidarScan(false, false, *b);
  FF_REQUIRE(scan_a.ok && scan_b.ok);
  FF_REQUIRE(!scan_a->points.empty() && !scan_b->points.empty());

  const size_t n        = std::min<size_t>(2000, std::min(scan_a->points.size(), scan_b->points.size()));
  size_t       compared = 0;
  size_t       same     = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto& p = scan_a->points[i];
    const auto& q = scan_b->points[i];
    if (!Finite(p) || !Finite(q)) {
      continue;
    }
    const double lp = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    const double lq = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
    ++compared;
    same += (std::fabs(p[0] / lp - q[0] / lq) < 1e-6 && std::fabs(p[1] / lp - q[1] / lq) < 1e-6 && std::fabs(p[2] / lp - q[2] / lq) < 1e-6) ? 1 : 0;
  }

  // identical directions across the whole prefix means one shared playback position
  FF_CHECK(compared > 100);
  FF_CHECK(same < compared);

  Drone->RemoveSensor(*a);
  Drone->RemoveSensor(*b);
}

// a class mask has a handful of flat colours; a photograph has thousands
FF_TEST(SegmentationReturnsAMaskNotAPhotograph)
{
  const auto id = Drone->AddSensor(wire::SensorType::RGB_SEG_CAMERA);
  FF_REQUIRE(id.ok);

  const auto config = Drone->GetRgbCameraConfig(*id);
  FF_REQUIRE(config.ok);

  // a fresh capture serves photographs for a few frames; the steady state is what matters
  size_t     colours  = 0;
  const auto deadline = Clock::now() + std::chrono::seconds(3);

  while (Clock::now() < deadline) {
    const auto image = Drone->GetRgbSegCameraData(*id);
    FF_REQUIRE(image.ok);
    FF_REQUIRE(image->compression == wire::Compression::NONE && image->format == wire::PixelFormat::BGR8);
    FF_REQUIRE(image->data.size() >= size_t(config->width) * config->height * 3);

    std::vector<uint32_t> packed;
    packed.reserve(image->data.size() / 3);
    for (size_t i = 0; i + 2 < image->data.size(); i += 3) {
      packed.push_back((uint32_t(image->data[i]) << 16) | (uint32_t(image->data[i + 1]) << 8) | image->data[i + 2]);
    }
    std::sort(packed.begin(), packed.end());
    packed.erase(std::unique(packed.begin(), packed.end()), packed.end());
    colours = packed.size();

    if (colours > 1 && colours < 512) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  FF_CHECK(colours > 1);
  FF_CHECK(colours < 512);

  Drone->RemoveSensor(*id);
}

// clearing an empty scene is the requested state, not a failure
FF_TEST(ClearingAnEmptySceneSucceeds)
{
  Sim->RemoveAllSpawnedObjects();

  const auto removed = Sim->RemoveAllSpawnedObjects();
  FF_CHECK(removed.ok);
  FF_CHECK(removed.ok && *removed == 0);
}

// a client that pauses must not get back everything that accumulated meanwhile
FF_TEST(LivoxBacklogStaysBounded)
{
  const auto id = Drone->AddSensor(wire::SensorType::LIDAR);
  FF_REQUIRE(id.ok);

  const auto base = Drone->GetLidarConfig(*id);
  FF_REQUIRE(base.ok);

  wire::LidarConfigT config = LivoxConfig(*base);
  config.frequency          = 10.0;
  FF_REQUIRE(Drone->SetLidarConfig(config, *id).ok);

  Drone->GetLidarScan(false, false, *id);

  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  const auto scan = Drone->GetLidarScan(false, false, *id);
  FF_REQUIRE(scan.ok);

  // 200 kHz over the 2-frame bound is 40k points; a whole second would be 200k
  FF_CHECK(scan->points.size() <= 60000);

  Drone->RemoveSensor(*id);
}

// the pattern file carries the sensor's real reach, and selecting a pattern must lift the range to it
FF_TEST(LivoxPatternSetsTheRange)
{
  const auto id = Drone->AddSensor(wire::SensorType::LIDAR);
  FF_REQUIRE(id.ok);

  const auto base = Drone->GetLidarConfig(*id);
  FF_REQUIRE(base.ok);

  for (const auto& [pattern, min_range_m] : {std::pair<const char*, double>{"avia", 400.0}, std::pair<const char*, double>{"mid360", 60.0}}) {
    wire::LidarConfigT config = LivoxConfig(*base, pattern);
    config.range_m            = 10.0;
    FF_REQUIRE(Drone->SetLidarConfig(config, *id).ok);

    const auto applied = Drone->GetLidarConfig(*id);
    FF_REQUIRE(applied.ok);
    FF_CHECK(applied->range_m >= min_range_m);
  }

  Drone->RemoveSensor(*id);
}

/* //} */

/* event camera in lockstep //{ */

// a 100 deg/s yaw over 200 stepped milliseconds yields a sorted stream inside the stepped interval
void CheckEventStream(const bool use_gpu)
{
  const auto id = Drone->AddSensor(wire::SensorType::EVENT_CAMERA);
  FF_REQUIRE(id.ok);

  auto config = Drone->GetEventCameraConfig(*id);
  FF_REQUIRE(config.ok);
  config->width   = 320;
  config->height  = 240;
  config->use_gpu = use_gpu;
  FF_REQUIRE(Drone->SetEventCameraConfig(*config, *id).ok);

  const auto applied = Sim->SetFixedTimestep(0.001);
  FF_REQUIRE(applied.ok);
  FF_CHECK(std::fabs(*applied - 0.001) < 1e-9);
  FF_REQUIRE(Sim->SetLockstep(true).ok);

  auto step = Sim->Step(20);
  FF_REQUIRE(step.ok);
  Drone->GetEventCameraData(*id);

  const auto state = Drone->GetState();
  FF_REQUIRE(state.ok && state->state.base);
  const Pose start_pose = *state->state.base;

  std::vector<TrajectorySample> samples;
  for (int i = 0; i <= 20; ++i) {
    TrajectorySample sample;
    sample.t                = step->sim_time + 0.01 * i;
    sample.base.position    = start_pose.position;
    sample.base.orientation = Quat::FromYaw(100.0 * kDeg * 0.01 * i) * start_pose.orientation;
    samples.push_back(sample);
  }
  FF_REQUIRE(Drone->SetTrajectory(samples).ok);

  const double start = step->sim_time;
  step               = Sim->Step(200);
  FF_REQUIRE(step.ok);
  FF_CHECK(std::fabs((step->sim_time - start) - 0.2) < 2e-3);

  const auto events = Drone->GetEventCameraData(*id);
  FF_REQUIRE(events.ok);
  FF_CHECK(events->width == 320 && events->height == 240);
  FF_CHECK(events->size() > 1000);
  FF_CHECK(events->dropped == 0);
  FF_CHECK(events->base_stamp >= start - 1e-9);
  FF_CHECK(events->stamp <= step->sim_time + 1e-9);

  bool sorted = true, in_bounds = true;
  for (size_t i = 0; i < events->size(); ++i) {
    const auto event = events->at(i);
    in_bounds        = in_bounds && event.x < 320 && event.y < 240;
    sorted           = sorted && (i == 0 || events->t_us[i] >= events->t_us[i - 1]);
  }
  FF_CHECK(sorted);
  FF_CHECK(in_bounds);
  FF_CHECK(events->size() == 0 || events->at(events->size() - 1).stamp <= events->stamp + 1e-6);

  std::cout << "    " << (use_gpu ? "gpu" : "cpu") << " events " << events->size() << " over " << (step->sim_time - start) << " s" << std::endl;

  FF_CHECK(Sim->SetLockstep(false).ok);
  FF_CHECK(Sim->SetFixedTimestep(0.0).ok);
  FF_CHECK(Drone->SetTrajectory({}).ok);
  FF_CHECK(Drone->RemoveSensor(*id).ok);
}

FF_TEST(EventCameraLockstepGpu)
{
  CheckEventStream(true);
}

FF_TEST(EventCameraLockstepCpu)
{
  CheckEventStream(false);
}

/* //} */

/* latency //{ */

// a 2 ms sleep once sat on every reply; loose enough for a busy machine, tight enough to catch that
FF_TEST(RequestLatencyStaysLow)
{
  std::vector<double> samples;
  for (int i = 0; i < 20; ++i) {
    const auto start = Clock::now();
    Drone->GetState();
    samples.push_back(std::chrono::duration<double, std::milli>(Clock::now() - start).count());
  }

  std::sort(samples.begin(), samples.end());
  const double median = samples[samples.size() / 2];

  std::cout << "    state read median " << median << " ms" << std::endl;
  FF_CHECK(median < 1.0);
}

// a reply typed as the sync op once made the client wait out its timeout on every call
FF_TEST(AsyncPoseIsAcknowledgedPromptly)
{
  const auto state = Drone->GetState();
  FF_REQUIRE(state.ok && state->state.base);

  const auto   start = Clock::now();
  const Status acked = Drone->SetPoseAsync(*state->state.base);
  const double ms    = std::chrono::duration<double, std::milli>(Clock::now() - start).count();

  std::cout << "    async pose " << ms << " ms" << std::endl;
  FF_CHECK(acked.ok);
  FF_CHECK(ms < 100.0);
}

/* //} */

/* cameras //{ */

FF_TEST(TwoCamerasOfOneTypeKeepTheirOwnResolution)
{
  const auto a = Drone->AddSensor(wire::SensorType::RGB_CAMERA);
  const auto b = Drone->AddSensor(wire::SensorType::RGB_CAMERA);
  FF_REQUIRE(a.ok && b.ok && *a != *b);

  auto config = Drone->GetRgbCameraConfig(*a);
  FF_REQUIRE(config.ok);
  config->encoding = Encoding(wire::Compression::JPEG);
  config->width    = 320;
  config->height   = 240;
  FF_REQUIRE(Drone->SetRgbCameraConfig(*config, *a).ok);
  config->width  = 160;
  config->height = 120;
  FF_REQUIRE(Drone->SetRgbCameraConfig(*config, *b).ok);

  const auto image_a = Drone->GetRgbCameraData(*a);
  const auto image_b = Drone->GetRgbCameraData(*b);
  FF_REQUIRE(image_a.ok && image_b.ok);
  FF_CHECK(JpegSize(image_a->data) == std::make_pair(320, 240) && image_a->width == 320);
  FF_CHECK(JpegSize(image_b->data) == std::make_pair(160, 120) && image_b->width == 160);

  FF_REQUIRE(Drone->RemoveSensor(*a).ok);
  const auto again = Drone->GetRgbCameraData(*b);
  FF_CHECK(again.ok && JpegSize(again->data) == std::make_pair(160, 120));

  Drone->RemoveSensor(*b);
}

// distinct stamps over N lockstep ticks
size_t StampsOverTicks(const int sensor, const int ticks)
{
  std::set<double> stamps;
  for (int i = 0; i < ticks; i++) {
    if (!Sim->Step(1).ok) {
      return 0;
    }
    const auto image = Drone->GetRgbCameraData(sensor);
    if (image.ok) {
      stamps.insert(image->stamp);
    }
  }
  return stamps.size();
}

FF_TEST(CaptureRateHoldsInLockstep)
{
  FF_REQUIRE(Sim->SetFixedTimestep(0.01).ok);
  FF_REQUIRE(Sim->SetLockstep(true).ok);

  const auto a = Drone->AddSensor(wire::SensorType::RGB_CAMERA);
  FF_REQUIRE(a.ok);
  auto config = Drone->GetRgbCameraConfig(*a);
  FF_REQUIRE(config.ok);
  config->width        = 320;
  config->height       = 240;
  config->capture_rate = 20.0;
  FF_REQUIRE(Drone->SetRgbCameraConfig(*config, *a).ok);

  FF_REQUIRE(Sim->Step(5).ok);
  Drone->GetRgbCameraData(*a);  // starts the stream

  const size_t stamps = StampsOverTicks(*a, 100);
  std::cout << "    20 Hz camera over 100 ticks of 10 ms: " << stamps << " distinct stamps" << std::endl;
  FF_CHECK(stamps >= 19 && stamps <= 21);

  config->capture_rate = 0.0;
  FF_REQUIRE(Drone->SetRgbCameraConfig(*config, *a).ok);
  Sim->Step(5);
  const size_t every_tick = StampsOverTicks(*a, 10);
  std::cout << "    every-tick camera over 10 ticks: " << every_tick << " distinct stamps" << std::endl;
  FF_CHECK(every_tick >= 9);

  Sim->SetLockstep(false);
  Sim->SetFixedTimestep(0.0);
  Drone->RemoveSensor(*a);
}

/* //} */

/* robot ops //{ */

FF_TEST(RobotDescribesItsPropellers)
{
  const auto description = Drone->GetDescription();
  FF_REQUIRE(description.ok);
  FF_REQUIRE(!description->links.empty());
  FF_CHECK(description->links[0]->name == "base_link" && description->links[0]->parent.empty());
  FF_CHECK(description->kind == wire::RobotKind::MULTIROTOR);
  FF_CHECK(description->joint_names.size() == 4);
  for (size_t i = 1; i < description->links.size(); ++i) {
    FF_CHECK(description->links[i]->parent == "base_link" && description->links[i]->joint_type == wire::JointType::CONTINUOUS);
  }
  FF_CHECK(description->yaml.find("base_link") != std::string::npos);

  const size_t joints = description->joint_names.size();
  FF_CHECK(Drone->SetJointVelocities(std::vector<double>(joints, 100.0)).ok);
  const auto state = Drone->GetState();
  FF_REQUIRE(state.ok);
  FF_CHECK(state->state.joint_velocities == std::vector<double>(joints, 100.0));
  FF_CHECK(state->state.joint_positions.size() == joints);

  FF_CHECK(!Drone->SetJointVelocities(std::vector<double>(joints + 1, 1.0)).ok);
  FF_CHECK(Drone->SetJointVelocities({}).ok);

  RobotState positions;
  positions.joint_positions = {0.1};
  const auto refused        = Drone->SetState(positions);
  FF_CHECK(!refused.ok && refused.error.find("joint_positions") != std::string::npos);
}

FF_TEST(RobotStateRoundTrips)
{
  RobotState state;
  state.base                  = Pose::At(1.0, -2.0, 3.0, 30.0 * kDeg);
  state.base_linear_velocity  = Vec3{1.0, 2.0, 3.0};
  state.base_angular_velocity = Vec3{0.1, 0.2, 0.3};

  const auto result = Drone->SetState(state);
  FF_REQUIRE(result.ok);
  FF_CHECK(result->robot_id == Drone->Id());
  FF_CHECK(Distance(result->base.position, state.base->position) < 0.01);

  const auto read = Drone->GetState();
  FF_REQUIRE(read.ok && read->state.base && read->state.base_linear_velocity && read->state.base_angular_velocity);
  FF_CHECK(Distance(read->state.base->position, state.base->position) < 0.01);
  FF_CHECK_NEAR(read->state.base->orientation.Yaw(), 30.0 * kDeg, 1e-4);
  FF_CHECK_NEAR(read->state.base_linear_velocity->y, 2.0, 1e-12);
  FF_CHECK_NEAR(read->state.base_angular_velocity->z, 0.3, 1e-12);
  FF_CHECK(read->stamp > 0.0);

  const auto robots = Sim->ListRobots();
  FF_REQUIRE(robots.ok);
  bool listed = false;
  for (const auto& robot : *robots) {
    listed = listed || (robot.id == Drone->Id() && robot.label == "test" && robot.name == "x500" && robot.kind == wire::RobotKind::MULTIROTOR);
  }
  FF_CHECK(listed);

  RobotState home;
  home.base          = kHome;
  const auto results = Sim->SetStates({{Drone->Id(), home}});
  FF_REQUIRE(results.ok && results->size() == 1);
  FF_CHECK((*results)[0].robot_id == Drone->Id());
  FF_CHECK(Distance((*results)[0].base.position, kHome.position) < 0.01);

  FF_CHECK(!Sim->SetStates({{99999, home}}).ok);
  FF_CHECK(Drone->GetCollision().ok);
}

/* //} */

/* conventions //{ */

// a cube at a known ground-frame pose, seen through every sensor and query; a mirrored axis or flipped yaw lands somewhere else
FF_TEST(ConventionCheck)
{
  Sim->RemoveAllSpawnedObjects();

  // facing +y at R, with C four metres ahead and D three metres to the left
  const Pose robot = Pose::At(0.0, -2.0, 3.0, 90.0 * kDeg);
  FF_REQUIRE(Sim->SpawnObjectsFromYaml("objects:\n"
                                       "  - asset: /Engine/BasicShapes/Cube\n"
                                       "    position: [0.0, 2.0, 3.0]\n"
                                       "  - asset: /Engine/BasicShapes/Cube\n"
                                       "    position: [-3.0, -2.0, 3.0]\n")
                 .ok);
  FF_REQUIRE(Drone->SetPose(robot).ok);

  const auto state = Drone->GetState();
  FF_REQUIRE(state.ok && state->state.base);
  FF_CHECK(Distance(state->state.base->position, robot.position) < 0.01);

  {
    const auto id = Drone->AddSensor(wire::SensorType::LIDAR);
    FF_REQUIRE(id.ok);
    const auto scan = Drone->GetLidarScan(false, false, *id);
    FF_REQUIRE(scan.ok && scan->rows > 1 && scan->cols > 1 && scan->points.size() == size_t(scan->rows) * scan->cols);

    FF_CHECK(Distance(scan->sensor_pose.position, robot.position) < 0.01);
    FF_CHECK_NEAR(scan->sensor_pose.orientation.Yaw(), 90.0 * kDeg, 1e-3);

    float ahead = 1e9f, left = 1e9f;
    for (const auto& p : scan->points) {
      if (!Finite(p)) {
        continue;
      }
      if (std::fabs(p[1]) < 0.25f && std::fabs(p[2]) < 0.25f && p[0] > 0.0f) {
        ahead = std::min(ahead, p[0]);
      }
      if (std::fabs(p[0]) < 0.25f && std::fabs(p[2]) < 0.25f && p[1] > 0.0f) {
        left = std::min(left, p[1]);
      }
    }
    std::cout << "    lidar: C's face " << ahead << " m ahead, D's face " << left << " m left" << std::endl;
    FF_CHECK(std::fabs(ahead - 3.5f) < 0.05f);
    FF_CHECK(std::fabs(left - 2.5f) < 0.05f);

    // organised: row 0 is the top beam
    const auto mean_z = [&scan](const uint32_t row) {
      double sum = 0.0;
      size_t n   = 0;
      for (uint32_t c = 0; c < scan->cols; ++c) {
        const auto& p = scan->points[size_t(row) * scan->cols + c];
        if (Finite(p)) {
          const double r = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
          sum += p[2] / r;
          ++n;
        }
      }
      return n ? sum / n : 0.0;
    };
    FF_CHECK(mean_z(0) > mean_z(scan->rows - 1));

    Drone->RemoveSensor(*id);
  }

  {
    // the rangefinder hangs 10 cm under the base and looks down onto C's top at 3.5 m
    FF_REQUIRE(Drone->SetPose(Pose::At(0.0, 2.0, 6.0)).ok);
    const auto range = Drone->GetRangefinderData();
    FF_REQUIRE(range.ok);
    std::cout << "    rangefinder: " << range->range << " m" << std::endl;
    FF_CHECK(std::fabs(range->range - 2.4) < 0.05);
  }

  {
    // E at body (5, 1.5, 0.5): left of and above the image centre, its near face 4.5 m ahead of the base
    Sim->RemoveAllSpawnedObjects();
    FF_REQUIRE(Drone->SetPose(robot).ok);
    FF_REQUIRE(Sim->SpawnObjectsFromYaml("objects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [-1.5, 3.0, 3.5]\n").ok);

    const auto id = Drone->AddSensor(wire::SensorType::DEPTH_CAMERA);
    FF_REQUIRE(id.ok);
    Drone->GetDepthCameraData(*id);
    const auto depth = Drone->GetDepthCameraData(*id);
    const auto info  = Drone->GetCameraInfo(*id);
    FF_REQUIRE(depth.ok && info.ok && info->size() == 1 && (*info)[0].mount);

    const wire::CameraInfoT& camera = (*info)[0];
    const Vec3               mount{camera.mount->position().x(), camera.mount->position().y(), camera.mount->position().z()};
    FF_CHECK_NEAR(std::fabs(camera.mount->orientation().w()), 1.0, 1e-9);

    // optical frame: z = mount x, x = -mount y, y = -mount z
    const Vec3   target{5.0 - mount.x, 1.5 - mount.y, 0.5 - mount.z};
    const double u = camera.fx * (-target.y / target.x) + camera.cx;
    const double v = camera.fy * (-target.z / target.x) + camera.cy;
    FF_REQUIRE(u >= 0 && v >= 0 && u < depth->width && v < depth->height);
    FF_CHECK(u < camera.cx && v < camera.cy);

    const double seen     = depth->mm[size_t(std::lround(v)) * depth->width + size_t(std::lround(u))] / 1000.0;
    const double expected = 4.5 - mount.x;
    std::cout << "    depth at the projected centre of E: " << seen << " m, expected " << expected << " m" << std::endl;
    FF_CHECK(std::fabs(seen - expected) < 0.1);

    Drone->RemoveSensor(*id);
  }

  {
    Sim->RemoveAllSpawnedObjects();
    FF_REQUIRE(Sim->SpawnObjectsFromYaml("objects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [0.0, 2.0, 3.0]\n").ok);

    const auto mesh = Sim->GetWorldMesh(Vec3{0.0, 2.0, 3.0}, Vec3{0.6, 0.6, 0.6});
    FF_REQUIRE(mesh.ok && !mesh->vertices.empty());
    int corners = 0;
    for (const double dx : {-0.5, 0.5}) {
      for (const double dy : {-0.5, 0.5}) {
        for (const double dz : {-0.5, 0.5}) {
          const Vec3 corner{dx, 2.0 + dy, 3.0 + dz};
          corners += std::any_of(mesh->vertices.begin(), mesh->vertices.end(),
                                 [&corner](const std::array<float, 3>& p) { return Distance(Vec3{p[0], p[1], p[2]}, corner) < 0.01; })
                         ? 1
                         : 0;
        }
      }
    }
    FF_CHECK(corners == 8);

    const auto distances = Sim->GetDistanceBatch({{0.0f, 2.0f, 4.5f}, {1.5f, 2.0f, 3.0f}, {0.0f, 3.5f, 3.0f}}, 5.0, 0.001);
    FF_REQUIRE(distances.ok && distances->size() == 3);
    for (const float d : *distances) {
      FF_CHECK(std::fabs(d - 1.0f) < 0.02f);
    }
  }

  Sim->RemoveAllSpawnedObjects();
  Drone->SetPose(kHome);
}

/* //} */

/* threading //{ */

// a request that holds the game thread delays nobody who does not need it
FF_TEST(SlowRequestDoesNotDelayPing)
{
  auto slow_connection = NewConnection();
  slow_connection->SetReceiveTimeout(std::chrono::seconds(120));
  SimulatorClient slow(slow_connection);
  SimulatorClient pinger(NewConnection());

  std::atomic<bool> done{false};
  double            slow_ms = 0.0;
  std::thread       worker([&] {
    const std::vector<std::array<float, 3>> points(400000, std::array<float, 3>{0.0f, 0.0f, 40.0f});
    const auto                              start = Clock::now();
    slow.GetDistanceBatch(points, 5.0, 0.001);
    slow_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    done    = true;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  double worst = 0.0;
  int    pings = 0;
  bool   ok    = true;
  while (!done) {
    const auto start = Clock::now();
    ok               = ok && pinger.Ping().ok && pinger.GetTime().ok;
    worst            = std::max(worst, std::chrono::duration<double, std::milli>(Clock::now() - start).count());
    ++pings;
  }
  worker.join();

  std::cout << "    slow request " << slow_ms << " ms, " << pings << " ping+time pairs alongside, worst " << worst << " ms" << std::endl;
  FF_CHECK(ok);
  FF_CHECK(slow_ms > 100.0);
  FF_CHECK(pings > 10);
  FF_CHECK(worst < 50.0);
}

FF_TEST(ManyConnectionsOnOneRobot)
{
  std::atomic<int>         failures{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 8; ++t) {
    threads.emplace_back([&failures, t] {
      RobotClient robot(NewConnection(), Drone->Id());
      for (int i = 0; i < 10; ++i) {
        bool ok = robot.GetState().ok;
        ok      = ok && (t % 2 ? robot.GetLidarScan().ok : robot.GetRgbCameraData().ok);
        ok      = ok && robot.GetRangefinderData().ok;
        failures += ok ? 0 : 1;
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  FF_CHECK(failures == 0);
}

FF_TEST(RemoveRobotWhileItsSensorsAreRead)
{
  const auto id = Sim->SpawnRobot("x500", Pose::At(2.0, 0.0, 2.0), "doomed");
  FF_REQUIRE(id.ok);

  std::atomic<int>         served{0};
  std::atomic<bool>        stop{false};
  std::vector<std::thread> readers;
  for (int t = 0; t < 4; ++t) {
    readers.emplace_back([&served, &stop, robot_id = *id, t] {
      RobotClient robot(NewConnection(), robot_id);
      while (!stop) {
        const bool ok = t == 0 ? robot.GetRgbCameraData().ok : t == 1 ? robot.GetLidarScan().ok : t == 2 ? robot.GetDepthCameraData().ok : robot.GetState().ok;
        if (!ok) {
          return;
        }
        ++served;
      }
    });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(800));

  const auto   start   = Clock::now();
  const Status removed = Sim->RemoveRobot(*id);
  const double ms      = std::chrono::duration<double, std::milli>(Clock::now() - start).count();

  // a reader stuck on the dead robot would hold the join until its receive timeout
  stop                    = true;
  const auto join_started = Clock::now();
  for (auto& reader : readers) {
    reader.join();
  }
  const double join_ms = std::chrono::duration<double, std::milli>(Clock::now() - join_started).count();

  std::cout << "    " << served << " reads before removal, removal took " << ms << " ms, readers stopped in " << join_ms << " ms" << std::endl;
  FF_CHECK(removed.ok);
  FF_CHECK(served > 0);
  FF_CHECK(join_ms < 5000.0);
  FF_CHECK(Sim->Ping().ok);

  const auto robots = Sim->ListRobots();
  FF_REQUIRE(robots.ok);
  FF_CHECK(std::none_of(robots->begin(), robots->end(), [&id](const RobotInfo& robot) { return robot.id == *id; }));
  FF_CHECK(!RobotClient(Sim->SharedConnection(), *id).GetState().ok);
}

/* //} */

/* malformed frames //{ */

class RawSocket {
public:
  RawSocket() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(Port);
    ::inet_pton(AF_INET, kAddress.c_str(), &addr.sin_addr);
    connected_ = ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
  }
  ~RawSocket() {
    ::close(fd_);
  }

  bool Connected() const {
    return connected_;
  }

  void Send(const std::vector<uint8_t>& bytes) {
    ::send(fd_, bytes.data(), bytes.size(), MSG_NOSIGNAL);
  }

  static std::vector<uint8_t> Frame(const std::vector<uint8_t>& body, const uint8_t flags = 0) {
    std::vector<uint8_t> frame(kFrameHeaderBytes);
    EncodeFrameHeader(static_cast<uint32_t>(body.size()), frame.data());
    frame[4] = flags;
    frame.insert(frame.end(), body.begin(), body.end());
    return frame;
  }

  // nullopt when the simulator closed the connection or said nothing within the timeout
  std::optional<std::vector<uint8_t>> Receive(const int timeout_ms = 3000) {
    uint8_t header[kFrameHeaderBytes];
    if (!Read(header, sizeof(header), timeout_ms)) {
      return std::nullopt;
    }
    std::vector<uint8_t> body(DecodeFrameLength(header));
    if (!Read(body.data(), body.size(), timeout_ms)) {
      return std::nullopt;
    }
    return body;
  }

  // true when the peer closed rather than went quiet
  bool Closed(const int timeout_ms = 3000) {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) {
      return false;
    }
    uint8_t byte;
    return ::recv(fd_, &byte, 1, 0) <= 0;
  }

private:
  bool Read(uint8_t* data, size_t size, const int timeout_ms) {
    while (size > 0) {
      pollfd p{fd_, POLLIN, 0};
      if (::poll(&p, 1, timeout_ms) <= 0) {
        return false;
      }
      const ssize_t got = ::recv(fd_, data, size, 0);
      if (got <= 0) {
        return false;
      }
      data += got;
      size -= static_cast<size_t>(got);
    }
    return true;
  }

  int  fd_        = -1;
  bool connected_ = false;
};

std::vector<uint8_t> PingBody(const uint32_t seq)
{
  flatbuffers::FlatBufferBuilder b;
  b.Finish(wire::CreateRequest(b, seq, flatbuffers::nullopt, flatbuffers::nullopt, wire::RequestBody::PingRequest, wire::CreatePingRequest(b).Union()));
  return std::vector<uint8_t>(b.GetBufferPointer(), b.GetBufferPointer() + b.GetSize());
}

bool RefusedReply(const std::optional<std::vector<uint8_t>>& body)
{
  if (!body) {
    return false;
  }
  flatbuffers::Verifier verifier(body->data(), body->size());
  if (!verifier.VerifyBuffer<wire::Response>(nullptr)) {
    return false;
  }
  const auto* response = flatbuffers::GetRoot<wire::Response>(body->data());
  return !response->ok() && response->error() && response->error()->size() > 0;
}

FF_TEST(MalformedFramesAreRefused)
{
  {
    RawSocket socket;
    FF_REQUIRE(socket.Connected());

    std::vector<uint8_t> garbage(64);
    for (size_t i = 0; i < garbage.size(); ++i) {
      garbage[i] = static_cast<uint8_t>(i * 37 + 11);
    }
    socket.Send(RawSocket::Frame(garbage));
    FF_CHECK(RefusedReply(socket.Receive()));

    socket.Send(RawSocket::Frame(PingBody(7), 1));
    FF_CHECK(RefusedReply(socket.Receive()));

    // a cut-off request body is refused, not read past
    std::vector<uint8_t> truncated = PingBody(8);
    truncated.resize(truncated.size() / 2);
    socket.Send(RawSocket::Frame(truncated));
    FF_CHECK(RefusedReply(socket.Receive()));

    // and the stream is still in step
    socket.Send(RawSocket::Frame(PingBody(9)));
    const auto reply = socket.Receive();
    FF_REQUIRE(reply.has_value());
    const auto* response = flatbuffers::GetRoot<wire::Response>(reply->data());
    FF_CHECK(response->ok() && response->seq() == 9);
  }

  {
    // a frame that never completes, then a hang-up
    RawSocket socket;
    FF_REQUIRE(socket.Connected());
    std::vector<uint8_t> partial = RawSocket::Frame(std::vector<uint8_t>(100, 0));
    partial.resize(kFrameHeaderBytes + 10);
    socket.Send(partial);
  }

  {
    RawSocket socket;
    FF_REQUIRE(socket.Connected());
    std::vector<uint8_t> huge(kFrameHeaderBytes);
    EncodeFrameHeader(0xFFFFFFF0u, huge.data());
    socket.Send(huge);
    FF_CHECK(socket.Closed());
  }

  FF_CHECK(Sim->Ping().ok);
  FF_CHECK(Drone->GetState().ok);
}

/* //} */

}  // namespace

int main()
{
  if (const char* env = std::getenv("FLIGHTFORGE_PORT")) {
    Port = static_cast<uint16_t>(std::atoi(env));
  }

  auto connection = std::make_shared<Connection>(kAddress, Port);
  if (!connection->Connect()) {
    std::cout << "SKIP: no simulator reachable on " << kAddress << ":" << Port << std::endl;
    return FF_SKIP_EXIT_CODE;
  }

  Sim = std::make_unique<SimulatorClient>(connection);

  const auto id = Sim->SpawnRobot("x500", kHome, "test");
  if (!id.ok) {
    std::cerr << "FAIL: could not spawn a robot: " << id.error << std::endl;
    return EXIT_FAILURE;
  }
  Drone = std::make_unique<RobotClient>(connection, *id);

  const int result = fftest::RunAll();

  Sim->RemoveRobot(*id);
  connection->Disconnect();

  return result;
}
