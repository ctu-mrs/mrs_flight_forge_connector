/**
 * Times each sensor through the C++ API, which is what actually consumes this simulator.
 *
 * Measuring through the python wrapper conflates the simulator with a per-point object
 * conversion that costs more than everything else combined; these numbers are the ones a
 * C++ client sees.
 *
 *   profile_sensors
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <flight_forge_connector/flight_forge_connector.h>
#include <flight_forge_connector/game_mode_controller.h>

namespace {

const std::string kAddress      = "127.0.0.1";
const int         kGameModePort = 8551;

enum SensorType
{
  RGB = 0,
  LIDAR,
  RANGEFINDER,
  LIDAR_LIVOX,
  RGB_SEG,
  STEREO,
  EVENT,
  FISHEYE,
  INSTANCE_SEG,
  DEPTH,
};

/* timing helpers //{ */

struct FTiming
{
  double MedianMs = 0.0;
  double P90Ms    = 0.0;
};

template <typename TCall>
FTiming Time(TCall&& Call, const int Repeats = 10)
{
  std::vector<double> Samples;
  Samples.reserve(Repeats);

  for (int i = 0; i < Repeats; ++i) {
    const auto Start = std::chrono::steady_clock::now();
    Call();
    Samples.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count());
  }

  std::sort(Samples.begin(), Samples.end());
  return {Samples[Samples.size() / 2], Samples[static_cast<size_t>(Samples.size() * 0.9)]};
}

//}

}  // namespace

int main()
{
  ueds_connector::GameModeController GameMode(kAddress, kGameModePort);
  if (!GameMode.ConnectSimple()) {
    std::printf("no simulator on %s:%d\n", kAddress.c_str(), kGameModePort);
    return 1;
  }

  ueds_connector::Coordinates Where(0.0, 0.0, 300.0);
  std::string                 Frame = "x500";

  const auto [SpawnOk, Port] = GameMode.SpawnDroneAtLocation(Where, Frame);
  if (!SpawnOk) {
    std::printf("could not spawn a drone\n");
    return 1;
  }

  ueds_connector::UedsConnector Drone(kAddress, Port);
  if (!Drone.ConnectSimple()) {
    std::printf("could not connect to the drone\n");
    return 1;
  }

  /* round trip floor //{ */

  std::printf("\n=== round trip floor ===\n");
  {
    const FTiming Pose = Time([&] { Drone.GetLocation(); }, 40);
    std::printf("  pose read                 %7.2f ms  (p90 %6.2f)\n", Pose.MedianMs, Pose.P90Ms);
  }

  //}

  /* grid lidar //{ */

  std::printf("\n=== grid lidar, by ray count ===\n");
  {
    const auto [AddOk, SensorId] = Drone.AddSensor(LIDAR);
    auto [ConfigOk, Config]      = Drone.GetLidarConfig(SensorId);

    Config.Enable     = true;
    Config.Livox      = false;
    Config.beamLength = 7000.0;

    for (const auto [Hor, Vert] : {std::pair{64, 16}, std::pair{128, 32}, std::pair{256, 32}, std::pair{256, 64}, std::pair{512, 64}}) {
      Config.BeamHorRays  = Hor;
      Config.BeamVertRays = Vert;
      Drone.SetLidarConfig(Config, SensorId);

      Drone.GetLidarData(SensorId);

      size_t        Points = 0;
      const FTiming T      = Time(
          [&] {
            const auto [Ok, P, Start, Stamp] = Drone.GetLidarData(SensorId);
            Points                           = P.size();
          },
          6);

      std::printf("  %6zu rays              %7.2f ms  (p90 %6.2f)   %5.2f us/point\n", Points, T.MedianMs, T.P90Ms,
                  1000.0 * T.MedianMs / std::max<size_t>(Points, 1));
    }

    Drone.RemoveSensor(SensorId);
  }

  //}

  /* livox //{ */

  std::printf("\n=== livox mid360, by frame rate ===\n");
  {
    const auto [AddOk, SensorId] = Drone.AddSensor(LIDAR_LIVOX);
    auto [ConfigOk, Config]      = Drone.GetLidarConfig(SensorId);

    Config.Enable      = true;
    Config.Livox       = true;
    Config.LivoxSensor = "mid360";
    Config.beamLength  = 7000.0;

    for (const double Hz : {10.0, 20.0, 50.0, 100.0}) {
      Config.Frequency = Hz;
      Drone.SetLidarConfig(Config, SensorId);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      const int FramePeriodMs = static_cast<int>(1000.0 / Hz);

      // A slice covers the time since the last request, so the reads have to be paced at
      // the sensor's own rate - but the pacing sleep must sit OUTSIDE the timed region,
      // or it is the measurement.
      size_t              Points = 0;
      std::vector<double> Samples;

      for (int i = 0; i < 6; ++i) {
        Drone.GetLidarData(SensorId);
        std::this_thread::sleep_for(std::chrono::milliseconds(FramePeriodMs));

        const auto Start                 = std::chrono::steady_clock::now();
        const auto [Ok, P, Origin, Stamp] = Drone.GetLidarData(SensorId);
        Samples.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count());

        Points = P.size();
      }

      std::sort(Samples.begin(), Samples.end());
      const FTiming T{Samples[Samples.size() / 2], Samples[static_cast<size_t>(Samples.size() * 0.9)]};

      const double Budget = 1000.0 / Hz;
      std::printf("  %5.0f Hz  %6zu returns  %7.2f ms  (p90 %6.2f)  %5.2f us/point  budget %5.1f ms  %s\n", Hz, Points, T.MedianMs, T.P90Ms,
                  1000.0 * T.MedianMs / std::max<size_t>(Points, 1), Budget, T.MedianMs <= Budget ? "REAL TIME" : "over budget");
    }

    Drone.RemoveSensor(SensorId);
  }

  //}

  /* cameras //{ */

  std::printf("\n=== cameras, one frame each ===\n");
  {
    struct FProbe
    {
      const char* Name;
      int         Type;
      int         Kind;  // 0 rgb-ish, 1 stereo, 2 depth, 3 instance, 4 seg, 5 event
    };

    const FProbe Probes[] = {
        {"rgb", RGB, 0}, {"fisheye", FISHEYE, 0},   {"stereo", STEREO, 1},        {"depth", DEPTH, 2},
        {"instance seg", INSTANCE_SEG, 3}, {"segmentation", RGB_SEG, 4}, {"event", EVENT, 5},
    };

    for (const FProbe& Probe : Probes) {
      const auto [AddOk, SensorId] = Drone.AddSensor(Probe.Type);
      if (!AddOk) {
        std::printf("  %-14s could not be added\n", Probe.Name);
        continue;
      }

      size_t Bytes = 0;

      const auto Read = [&] {
        switch (Probe.Kind) {
          case 1: {
            const auto [Ok, L, R, Stamp] = Drone.GetStereoCameraData(SensorId);
            Bytes                        = L.size() + R.size();
            break;
          }
          case 2: {
            std::vector<uint16_t> Depth;
            int                   W = 0, H = 0;
            double                Stamp = 0.0;
            Drone.GetDepthCameraData(Depth, W, H, Stamp, SensorId);
            Bytes = Depth.size() * sizeof(uint16_t);
            break;
          }
          case 3: {
            std::vector<unsigned char> Image;
            double                     Stamp = 0.0;
            Drone.GetInstanceSegData(Image, Stamp, SensorId);
            Bytes = Image.size();
            break;
          }
          case 4: {
            const auto [Ok, Raw, Stamp, Size] = Drone.GetRgbSegmented(SensorId);
            Bytes                             = Raw.size();
            break;
          }
          case 5: {
            const auto [Ok, Events] = Drone.GetEventCameraData(SensorId);
            Bytes                   = Events.size() * 2 * sizeof(uint32_t);
            break;
          }
          default: {
            const auto [Ok, Image, Stamp, Size] = Probe.Type == FISHEYE ? Drone.GetFisheyeCameraData(SensorId) : Drone.GetRgbCameraData(SensorId);
            Bytes                               = Image.size();
            break;
          }
        }
      };

      Read();
      const FTiming T = Time(Read, 8);

      std::printf("  %-14s %7.2f ms  (p90 %6.2f)   %7zu bytes\n", Probe.Name, T.MedianMs, T.P90Ms, Bytes);

      Drone.RemoveSensor(SensorId);
    }
  }

  //}

  GameMode.RemoveDrone(Port);
  Drone.Disconnect();
  GameMode.Disconnect();
  return 0;
}
