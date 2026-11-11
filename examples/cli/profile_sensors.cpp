// Times each sensor through the C++ API, as a C++ client sees it.
//   profile_sensors            (FLIGHTFORGE_ADDRESS and FLIGHTFORGE_PORT override 127.0.0.1:8600)

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include <flight_forge_connector/robot_client.h>
#include <flight_forge_connector/simulator_client.h>

#include "common.h"

using namespace flightforge;

namespace
{

struct FTiming
{
  double median_ms = 0.0;
  double p90_ms    = 0.0;
  bool   ok        = true;
};

FTiming Time(const std::function<bool()>& call, const int repeats = 20) {
  call();

  std::vector<double> samples;
  FTiming             timing;
  for (int i = 0; i < repeats; ++i) {
    const auto start = std::chrono::steady_clock::now();
    timing.ok        = call() && timing.ok;
    samples.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }
  std::sort(samples.begin(), samples.end());
  timing.median_ms = samples[samples.size() / 2];
  timing.p90_ms    = samples[samples.size() * 9 / 10];
  return timing;
}

void Print(const char* name, const FTiming& timing) {
  std::printf("%-22s median %8.2f ms   p90 %8.2f ms%s\n", name, timing.median_ms, timing.p90_ms, timing.ok ? "" : "   (errors)");
}

}  // namespace

int main() {
  auto            connection = std::make_shared<Connection>(ffcli::Address(), ffcli::Port());
  SimulatorClient sim(connection);
  if (!sim.Connect(5)) {
    std::fprintf(stderr, "no simulator on %s:%u\n", ffcli::Address().c_str(), ffcli::Port());
    return 1;
  }

  const auto id = sim.SpawnRobot("x500", Pose::At(0.0, 0.0, 2.0), "profile_sensors");
  if (!id.ok) {
    std::fprintf(stderr, "spawn failed: %s\n", id.error.c_str());
    return 1;
  }
  RobotClient robot(connection, *id);

  Print("ping", Time([&] { return sim.Ping().ok; }));
  Print("get_state", Time([&] { return robot.GetState().ok; }));
  Print("set_state", Time([&] { return robot.SetPose(Pose::At(0.0, 0.0, 2.0)).ok; }));
  Print("rgb jpeg", Time([&] { return robot.GetRgbCameraData().ok; }));

  if (auto config = robot.GetRgbCameraConfig(); config.ok) {
    config->encoding              = std::make_unique<wire::ImageEncodingT>();
    config->encoding->compression = wire::Compression::NONE;
    robot.SetRgbCameraConfig(*config);
    Print("rgb raw", Time([&] { return robot.GetRgbCameraData().ok; }));
  }

  Print("stereo", Time([&] { return robot.GetStereoCameraData().ok; }));
  Print("fisheye", Time([&] { return robot.GetFisheyeCameraData().ok; }));
  Print("depth", Time([&] { return robot.GetDepthCameraData().ok; }));
  Print("segmentation", Time([&] { return robot.GetRgbSegCameraData().ok; }));
  Print("instances", Time([&] { return robot.GetInstanceSegData().ok; }));
  Print("events", Time([&] { return robot.GetEventCameraData().ok; }));
  Print("lidar", Time([&] { return robot.GetLidarScan().ok; }));
  Print("lidar + channels", Time([&] { return robot.GetLidarScan(true, true).ok; }));
  Print("rangefinder", Time([&] { return robot.GetRangefinderData().ok; }));

  sim.RemoveRobot(*id);
  return 0;
}
