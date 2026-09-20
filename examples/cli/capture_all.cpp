/**
 * Captures one frame from every camera the drone can carry and writes them out as PNGs,
 * so the sensors can be checked by eye rather than by byte count.
 *
 * The sensors that do not hand back an image get mapped to something viewable: depth is
 * colourised over its valid range, events are drawn as polarity dots, and instance ids
 * get a distinct colour each (raw ids are small integers and look black).
 *
 *   capture_all [output_dir]
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <flight_forge_connector/flight_forge_connector.h>
#include <flight_forge_connector/game_mode_controller.h>

#include "fpng.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#include "stb_image.h"

namespace {

const std::string kAddress      = "127.0.0.1";
const int         kGameModePort = 8551;

// sensor type ids, matching Sensor.h in the plugin
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

std::string             OutputDir = "./shots";
std::map<std::string, std::string> Results;

/* image writing //{ */

bool WritePng(const std::string& Name, const std::vector<uint8_t>& Rgb, const uint32_t Width, const uint32_t Height)
{
  const std::string Path = OutputDir + "/" + Name + ".png";

  if (!fpng::fpng_encode_image_to_file(Path.c_str(), Rgb.data(), Width, Height, 3)) {
    std::cerr << "  could not write " << Path << std::endl;
    return false;
  }

  std::cout << "  wrote " << Path << "  " << Width << "x" << Height << std::endl;
  return true;
}

// The served frame is already encoded (JPEG or PNG); decode it so everything we write
// out is a plain PNG, and so a corrupt frame is caught here rather than by an eye later.
bool WriteEncodedFrame(const std::string& Name, const std::vector<unsigned char>& Encoded)
{
  if (Encoded.empty()) {
    std::cerr << "  empty frame" << std::endl;
    return false;
  }

  int Width = 0, Height = 0, Channels = 0;
  stbi_uc* Pixels = stbi_load_from_memory(Encoded.data(), static_cast<int>(Encoded.size()), &Width, &Height, &Channels, 3);

  if (!Pixels) {
    std::cerr << "  could not decode frame (" << Encoded.size() << " bytes): " << stbi_failure_reason() << std::endl;
    return false;
  }

  const bool bJpeg = Encoded.size() > 3 && Encoded[0] == 0xFF && Encoded[1] == 0xD8;
  std::cout << "  " << (bJpeg ? "JPEG" : "PNG") << " " << Encoded.size() << " bytes -> ";

  const std::vector<uint8_t> Rgb(Pixels, Pixels + static_cast<size_t>(Width) * Height * 3);
  stbi_image_free(Pixels);

  return WritePng(Name, Rgb, Width, Height);
}

//}

/* viewable mappings //{ */

std::vector<uint8_t> ColouriseDepth(const std::vector<uint16_t>& Depth, const int Width, const int Height)
{
  std::vector<uint16_t> Sorted;
  Sorted.reserve(Depth.size());

  for (const uint16_t D : Depth) {
    if (D > 0 && D < 65535) {
      Sorted.push_back(D);
    }
  }

  const size_t Valid = Sorted.size();

  if (Sorted.empty()) {
    std::cout << "  no valid depth" << std::endl;
    return std::vector<uint8_t>(static_cast<size_t>(Width) * Height * 3, 0);
  }

  // Clip to the 2nd-98th percentile: a handful of very near or very far pixels would
  // otherwise take the whole colour range and flatten everything between them.
  std::sort(Sorted.begin(), Sorted.end());
  uint16_t Lo = Sorted[Sorted.size() * 2 / 100];
  uint16_t Hi = Sorted[Sorted.size() * 98 / 100];

  std::cout << "  " << (100.0 * Valid / std::max<size_t>(Depth.size(), 1)) << "% valid, " << Sorted.front() << "-" << Sorted.back()
            << " mm (showing " << Lo << "-" << Hi << ")" << std::endl;

  std::vector<uint8_t> Rgb(static_cast<size_t>(Width) * Height * 3, 0);
  const float          Span = std::max(1.0f, static_cast<float>(Hi) - static_cast<float>(Lo));

  for (size_t i = 0; i < Depth.size(); ++i) {
    const uint16_t D = Depth[i];
    if (D == 0 || D == 65535) {
      continue;
    }

    // Grayscale, near white -> far black, matching how depth is normally shown. The
    // data is single-channel millimetres; colour would only be a viewing choice.
    const float T     = std::clamp((static_cast<float>(D) - Lo) / Span, 0.0f, 1.0f);
    const auto  Shade = static_cast<uint8_t>((1.0f - T) * 255.0f);

    Rgb[i * 3 + 0] = Shade;
    Rgb[i * 3 + 1] = Shade;
    Rgb[i * 3 + 2] = Shade;
  }

  return Rgb;
}

std::vector<uint8_t> ColouriseIds(const std::vector<uint8_t>& IdRgb, const int Width, const int Height, size_t& OutDistinct)
{
  std::vector<uint8_t>            Out(static_cast<size_t>(Width) * Height * 3, 0);
  std::map<uint32_t, std::array<uint8_t, 3>> Palette;

  for (size_t i = 0; i < static_cast<size_t>(Width) * Height; ++i) {
    // ids are packed 24-bit little-endian across RGB
    const uint32_t Id = IdRgb[i * 3 + 0] | (IdRgb[i * 3 + 1] << 8) | (IdRgb[i * 3 + 2] << 16);

    if (Id == 0) {
      continue;
    }

    auto Found = Palette.find(Id);
    if (Found == Palette.end()) {
      std::mt19937                          Rng(Id * 2654435761u);
      std::uniform_int_distribution<int>    Dist(60, 255);
      Found = Palette.emplace(Id, std::array<uint8_t, 3>{static_cast<uint8_t>(Dist(Rng)), static_cast<uint8_t>(Dist(Rng)),
                                                         static_cast<uint8_t>(Dist(Rng))}).first;
    }

    Out[i * 3 + 0] = Found->second[0];
    Out[i * 3 + 1] = Found->second[1];
    Out[i * 3 + 2] = Found->second[2];
  }

  OutDistinct = Palette.size();
  return Out;
}

//}

/* camera placement //{ */

// Far enough forward and up that the airframe's own arms and propellers are out of
// frame - on the body they fill the lower third of every shot and dominate depth.
constexpr double kForwardCm = 60.0;
constexpr double kUpCm      = 25.0;

// The event camera is NOT accepted by the rgb config message (ResolveSensorId only lets
// segmentation, instance segmentation and depth through), so it needs its own.
bool MoveEventCameraOffBody(ueds_connector::UedsConnector& Drone, const int SensorId)
{
  auto [Ok, Config] = Drone.GetEventCameraConfig(SensorId);
  if (!Ok) {
    return false;
  }

  Config.offset_ = ueds_connector::Coordinates(kForwardCm, 0.0, kUpCm);
  return Drone.SetEventCameraConfig(Config, SensorId);
}

// Works for every sensor that speaks the rgb config message: rgb, segmentation,
// instance segmentation and depth.
bool MoveRgbLikeOffBody(ueds_connector::UedsConnector& Drone, const int SensorId)
{
  auto [Ok, Config] = Drone.GetRgbCameraConfig(SensorId);
  if (!Ok) {
    return false;
  }

  Config.offset_ = ueds_connector::Coordinates(kForwardCm, 0.0, kUpCm);

  if (!Drone.SetRgbCameraConfig(Config, SensorId)) {
    return false;
  }

  // read it back: a silently ignored offset is otherwise only visible as the airframe
  // sitting in the middle of the frame
  const auto [ReadOk, Applied] = Drone.GetRgbCameraConfig(SensorId);
  if (ReadOk && (std::fabs(Applied.offset_.x - kForwardCm) > 1.0 || std::fabs(Applied.offset_.z - kUpCm) > 1.0)) {
    std::cerr << "  sensor " << SensorId << " kept offset (" << Applied.offset_.x << ", " << Applied.offset_.y << ", " << Applied.offset_.z
              << ") instead of the requested (" << kForwardCm << ", 0, " << kUpCm << ")" << std::endl;
    return false;
  }

  return true;
}

//}

/* motion //{ */

// A small yaw sweep about the current pose. Rotation is what the event camera and the
// first-order rolling-shutter model both respond to, and keeping it small (and in place)
// leaves the scene in frame - a large sweep just moves the camera off the objects and
// changes every pixel at once, which tells you nothing.
void SmallSweep(ueds_connector::UedsConnector& Drone, const int Steps, const int StepMs, const double AmplitudeDeg = 6.0)
{
  const auto [PoseOk, Where]  = Drone.GetLocation();
  const auto [FacingOk, From] = Drone.GetRotation();

  if (!PoseOk || !FacingOk) {
    return;
  }

  for (int Step = 0; Step < Steps; ++Step) {
    const double T = static_cast<double>(Step) / std::max(1, Steps - 1);

    ueds_connector::Rotation Facing = From;
    Facing.yaw                      = From.yaw + AmplitudeDeg * std::sin(T * 2.0 * 3.14159265);

    Drone.SetLocationAndRotationAsync(Where, Facing, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(StepMs));
  }
}

//}

}  // namespace

int main(int argc, char** argv)
{
  if (argc > 1) {
    OutputDir = argv[1];
  }

  fpng::fpng_init();

  ueds_connector::GameModeController GameMode(kAddress, kGameModePort);
  if (!GameMode.ConnectSimple()) {
    std::cerr << "no simulator on " << kAddress << ":" << kGameModePort << std::endl;
    return 1;
  }

  ueds_connector::Coordinates SpawnLocation(0.0, 0.0, 150.0);
  std::string                 Frame = "x500";

  const auto [SpawnOk, Port] = GameMode.SpawnDroneAtLocation(SpawnLocation, Frame);
  if (!SpawnOk) {
    std::cerr << "could not spawn a drone" << std::endl;
    return 1;
  }

  ueds_connector::UedsConnector Drone(kAddress, Port);
  if (!Drone.ConnectSimple()) {
    std::cerr << "could not connect to the drone on " << Port << std::endl;
    return 1;
  }

  const auto Record = [](const std::string& Name, const bool Ok) { Results[Name] = Ok ? "ok" : "FAILED"; };

  // Put known non-Nanite geometry in front of the drone. /Engine/BasicShapes are plain
  // static meshes, unlike the Nanite content a project usually ships - and the instance
  // pass only draws classic mesh batches, so this is what tells the two cases apart.
  std::string Error;
  const std::string SceneYaml = "objects:\n"
                                "  - asset: /Engine/BasicShapes/Cube\n"
                                "    position: [8.0, -2.0, 1.0]\n"
                                "    scale: 2.0\n"
                                "  - asset: /Engine/BasicShapes/Sphere\n"
                                "    position: [9.0, 2.0, 1.0]\n"
                                "    scale: 2.0\n";

  const auto [PlacedOk, PlacedIds] = GameMode.SpawnObjectsFromYaml(SceneYaml, Error);
  std::cout << "placed " << PlacedIds.size() << " reference objects" << (PlacedOk ? "" : (" (" + Error + ")")) << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // RGB is the drone's default camera; the rest have to be added.
  /* rgb //{ */

  std::cout << "rgb" << std::endl;
  {
    if (!MoveRgbLikeOffBody(Drone, -1)) {
      std::cerr << "  could not move the rgb camera off the body" << std::endl;
    }

    const auto [Ok, Image, Stamp, Size] = Drone.GetRgbCameraData();
    Record("rgb", Ok && WriteEncodedFrame("01_rgb", Image));
  }

  //}

  /* segmentation //{ */

  std::cout << "segmentation" << std::endl;
  {
    const auto [AddOk, Id] = Drone.AddSensor(RGB_SEG);
    bool Ok = AddOk;

    if (Ok) {
      MoveRgbLikeOffBody(Drone, Id);

      const auto [DataOk, Raw, Stamp, Size] = Drone.GetRgbSegmented(Id);
      const auto [ConfigOk, Config]         = Drone.GetRgbCameraConfig(Id);

      // this one ships raw BGR, so its dimensions come from the camera config
      if (DataOk && ConfigOk && Raw.size() >= static_cast<size_t>(Config.width_) * Config.height_ * 3) {
        std::vector<uint8_t> Rgb(static_cast<size_t>(Config.width_) * Config.height_ * 3);
        for (size_t i = 0; i < Rgb.size() / 3; ++i) {
          Rgb[i * 3 + 0] = Raw[i * 3 + 2];
          Rgb[i * 3 + 1] = Raw[i * 3 + 1];
          Rgb[i * 3 + 2] = Raw[i * 3 + 0];
        }
        std::cout << "  raw BGR " << Raw.size() << " bytes -> ";
        Ok = WritePng("02_segmentation", Rgb, Config.width_, Config.height_);
      } else {
        std::cerr << "  no segmentation frame (data " << DataOk << ", config " << ConfigOk << ", " << Raw.size() << " bytes)" << std::endl;
        Ok = false;
      }
    }

    Record("segmentation", Ok);
  }

  //}

  /* stereo //{ */

  std::cout << "stereo" << std::endl;
  {
    const auto [AddOk, Id] = Drone.AddSensor(STEREO);
    bool Ok = AddOk;

    if (Ok) {
      auto [ConfigOk, Config] = Drone.GetStereoCameraConfig(Id);
      if (ConfigOk) {
        // keep the eyes' baseline, just move the rig forward and up
        Config.offset_x_left_ += kForwardCm;
        Config.offset_z_left_ += kUpCm;
        Config.offset_x_right_ += kForwardCm;
        Config.offset_z_right_ += kUpCm;
        Drone.SetStereoCameraConfig(Config, Id);
      }

      const auto [DataOk, Left, Right, Stamp] = Drone.GetStereoCameraData(Id);
      Ok = DataOk && WriteEncodedFrame("03_stereo_left", Left) && WriteEncodedFrame("04_stereo_right", Right);
    }

    Record("stereo", Ok);
  }

  //}

  /* fisheye //{ */

  std::cout << "fisheye" << std::endl;
  {
    const auto [AddOk, Id] = Drone.AddSensor(FISHEYE);
    bool Ok = AddOk;

    if (Ok) {
      auto [ConfigOk, Config] = Drone.GetFisheyeCameraConfig(Id);
      if (ConfigOk) {
        Config.offset_ = ueds_connector::Coordinates(kForwardCm, 0.0, kUpCm);
        Drone.SetFisheyeCameraConfig(Config, Id);
      }

      const auto [DataOk, Image, Stamp, Size] = Drone.GetFisheyeCameraData(Id);
      Ok = DataOk && WriteEncodedFrame("05_fisheye", Image);
    }

    Record("fisheye", Ok);
  }

  //}

  /* event //{ */

  std::cout << "event" << std::endl;
  {
    const auto [AddOk, Id] = Drone.AddSensor(EVENT);
    bool Ok = AddOk;

    if (Ok) {
      if (!MoveEventCameraOffBody(Drone, Id)) {
        std::cerr << "  could not move the event camera off the body" << std::endl;
      }

      // Events only fire where brightness changes, so a still camera produces a black
      // frame. Fly an arc first, and drain once before the real read: the first batch
      // covers the interval since the sensor was created, i.e. mostly stillness.
      SmallSweep(Drone, 10, 60);
      Drone.GetEventCameraData(Id);
      SmallSweep(Drone, 10, 60);

      const auto [DataOk, Events] = Drone.GetEventCameraData(Id);
      std::cout << "  " << Events.size() << " events, " << Events.dropped << " dropped" << std::endl;

      const int Width  = Events.width > 0 ? Events.width : 640;
      const int Height = Events.height > 0 ? Events.height : 480;

      // polarity is +1/-1, and -1 is truthy: it has to be compared, not tested
      size_t Positive = 0, Negative = 0;

      std::vector<uint8_t> Rgb(static_cast<size_t>(Width) * Height * 3, 0);
      for (size_t i = 0; i < Events.size(); ++i) {
        const auto Event = Events.at(i);
        if (Event.x < Width && Event.y < Height) {
          const size_t Index = (static_cast<size_t>(Event.y) * Width + Event.x) * 3;

          if (Event.polarity > 0) {
            Rgb[Index + 0] = 255;
            ++Positive;
          } else {
            Rgb[Index + 2] = 255;
            ++Negative;
          }
        }
      }

      std::cout << "  " << Positive << " brighter, " << Negative << " darker" << std::endl;

      Ok = DataOk && WritePng("06_events", Rgb, Width, Height);
    }

    Record("event", Ok);
  }

  //}

  /* instance segmentation //{ */

  std::cout << "instance segmentation" << std::endl;
  {
    const auto [AddOk, Id] = Drone.AddSensor(INSTANCE_SEG);
    bool Ok = AddOk;

    if (Ok) {
      MoveRgbLikeOffBody(Drone, Id);

      std::vector<unsigned char> Encoded;
      double                     Stamp = 0.0;

      if (Drone.GetInstanceSegData(Encoded, Stamp, Id) && !Encoded.empty()) {
        int Width = 0, Height = 0, Channels = 0;
        stbi_uc* Pixels = stbi_load_from_memory(Encoded.data(), static_cast<int>(Encoded.size()), &Width, &Height, &Channels, 3);

        if (Pixels) {
          const std::vector<uint8_t> IdRgb(Pixels, Pixels + static_cast<size_t>(Width) * Height * 3);
          stbi_image_free(Pixels);

          size_t     Distinct = 0;
          const auto Coloured = ColouriseIds(IdRgb, Width, Height, Distinct);
          std::cout << "  " << Distinct << " distinct instance ids" << std::endl;

          Ok = WritePng("07_instance_seg", Coloured, Width, Height);

          const auto [MapOk, Entries] = Drone.GetInstanceSegMap(Id);
          if (MapOk) {
            std::cout << "  id map holds " << Entries.size() << " actors:" << std::endl;

            // the actor paths are what segmentation rules match against, so print them
            for (const auto& Entry : Entries) {
              std::cout << "    " << Entry.id << "  " << Entry.actor << std::endl;
            }
          }
        } else {
          std::cerr << "  could not decode the id image" << std::endl;
          Ok = false;
        }
      } else {
        std::cerr << "  no instance frame" << std::endl;
        Ok = false;
      }
    }

    Record("instance_seg", Ok);
  }

  // Camera realism, one effect at a time against the same viewpoint, so each is legible
  // on its own: 01_rgb is the clean baseline to compare all of these against.

  //}

  /* realistic camera //{ */

  std::cout << "realistic camera" << std::endl;
  {
    const auto [ConfigOk, Baseline] = Drone.GetRgbCameraConfig();
    bool Ok = ConfigOk;

    // A config change takes a few frames to reach the served image: the capture target is
    // resized and the warp LUT rebuilt on the game thread, so frames already in flight
    // still carry the old settings (and the first frame after a resize can be blank).
    // Wait for one whose dimensions match what was asked for.
    const auto Apply = [&](const ueds_connector::RgbCameraConfig& Config, const std::string& Name) {
      if (!Drone.SetRgbCameraConfig(Config)) {
        std::cerr << "  could not apply " << Name << std::endl;
        return false;
      }

      for (int Attempt = 0; Attempt < 12; ++Attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const auto [DataOk, Image, Stamp, Size] = Drone.GetRgbCameraData();
        if (!DataOk || Image.empty()) {
          continue;
        }

        int Width = 0, Height = 0, Channels = 0;
        if (!stbi_info_from_memory(Image.data(), static_cast<int>(Image.size()), &Width, &Height, &Channels)) {
          continue;
        }

        if (Width == Config.width_ && Height == Config.height_ && Attempt >= 3) {
          return WriteEncodedFrame(Name, Image);
        }
      }

      std::cerr << "  " << Name << " never settled to " << Config.width_ << "x" << Config.height_ << std::endl;
      return false;
    };

    if (Ok) {
      // strong barrel, the sort a wide FPV lens shows
      auto Distorted                = Baseline;
      Distorted.distortion_.enable_ = true;
      Distorted.distortion_.k1_     = -0.28;
      Distorted.distortion_.k2_     = 0.07;
      Distorted.distortion_.p1_     = 0.001;
      Distorted.distortion_.p2_     = -0.0005;
      std::cout << "  distortion" << std::endl;
      Ok &= Apply(Distorted, "09_rgb_distortion");

      // exaggerated well past a real sensor so it reads in a single still
      auto Noisy                  = Baseline;
      Noisy.noise_.enable_        = true;
      Noisy.noise_.shot_scale_    = 4.0e-3;
      Noisy.noise_.read_sigma_    = 2.0e-2;
      Noisy.noise_.row_sigma_     = 6.0e-3;
      std::cout << "  noise" << std::endl;
      Ok &= Apply(Noisy, "10_rgb_noise");

      // rolling shutter only bends anything while the camera is turning
      auto Rolling                           = Baseline;
      Rolling.rolling_shutter_.enable_       = true;
      Rolling.rolling_shutter_.readout_time_ = 0.12;  // slow readout to make the skew obvious
      std::cout << "  rolling shutter (while turning)" << std::endl;

      if (Drone.SetRgbCameraConfig(Rolling)) {
        SmallSweep(Drone, 8, 50);
        const auto [DataOk, Image, Stamp, Size] = Drone.GetRgbCameraData();
        Ok &= DataOk && WriteEncodedFrame("11_rgb_rolling_shutter", Image);
      } else {
        Ok = false;
      }

      // everything at once, which is what a client would actually configure
      auto Combined                           = Distorted;
      Combined.noise_.enable_                 = true;
      Combined.noise_.shot_scale_             = 1.0e-3;
      Combined.noise_.read_sigma_             = 8.0e-3;
      Combined.rolling_shutter_.enable_       = true;
      Combined.rolling_shutter_.readout_time_ = 0.06;
      std::cout << "  all combined" << std::endl;

      if (Drone.SetRgbCameraConfig(Combined)) {
        SmallSweep(Drone, 8, 50);
        const auto [DataOk, Image, Stamp, Size] = Drone.GetRgbCameraData();
        Ok &= DataOk && WriteEncodedFrame("12_rgb_all_effects", Image);
      } else {
        Ok = false;
      }

      Drone.SetRgbCameraConfig(Baseline);
    }

    Record("realistic_camera", Ok);
  }

  //}

  /* depth //{ */

  std::cout << "depth" << std::endl;
  {
    const auto [AddOk, Id] = Drone.AddSensor(DEPTH);
    bool Ok = AddOk;

    if (Ok) {
      MoveRgbLikeOffBody(Drone, Id);

      // Park somewhere with geometry actually in view and let the pose settle, then take
      // depth and colour from the same standstill: without the colour reference there is
      // no way to tell "the depth is wrong" from "nothing was in front of the camera".
      const ueds_connector::Coordinates Viewpoint(-400.0, 0.0, 200.0);
      const ueds_connector::Rotation    Facing(0.0, 0.0, 0.0);
      Drone.SetLocationAndRotationAsync(Viewpoint, Facing, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(700));

      for (int i = 0; i < 3; ++i) {
        Drone.GetRgbCameraData();
      }

      const auto [RgbOk, RgbImage, RgbStamp, RgbSize] = Drone.GetRgbCameraData();
      if (RgbOk) {
        WriteEncodedFrame("08a_depth_view_rgb", RgbImage);
      }

      std::vector<uint16_t> Depth;
      int                   Width = 0, Height = 0;
      double                Stamp = 0.0;

      if (Drone.GetDepthCameraData(Depth, Width, Height, Stamp, Id) && Width > 0 && Height > 0) {
        Ok = WritePng("08b_depth", ColouriseDepth(Depth, Width, Height), Width, Height);
      } else {
        std::cerr << "  no depth frame" << std::endl;
        Ok = false;
      }
    }

    Record("depth", Ok);
  }

  //}

  std::cout << "\n--- summary ---" << std::endl;
  int Failures = 0;
  for (const auto& [Name, Status] : Results) {
    std::cout << Name << ": " << Status << std::endl;
    Failures += Status == "ok" ? 0 : 1;
  }

  GameMode.RemoveAllSpawnedObjects();
  GameMode.RemoveDrone(Port);
  Drone.Disconnect();
  GameMode.Disconnect();

  return Failures == 0 ? 0 : 1;
}
