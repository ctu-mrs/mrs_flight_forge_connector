// One frame from every camera, written as PNGs to check the sensors by eye; depth, events and instance ids are colourised.
//   capture_all [output_dir]            (FLIGHTFORGE_ADDRESS and FLIGHTFORGE_PORT override 127.0.0.1:8600)

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <flight_forge_connector/robot_client.h>
#include <flight_forge_connector/simulator_client.h>

#include "common.h"
#include "fpng.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

using namespace flightforge;

namespace
{

std::string Dir = ".";

bool WriteRgb(const std::string& name, const std::vector<uint8_t>& rgb, const uint32_t width, const uint32_t height) {
  const std::string path = Dir + "/" + name + ".png";
  const bool        ok   = fpng::fpng_encode_image_to_file(path.c_str(), rgb.data(), width, height, 3);
  std::cout << (ok ? "wrote " : "could not write ") << path << std::endl;
  return ok;
}

// raw colour frames arrive BGRA8 or BGR8
std::vector<uint8_t> ToRgb(const Image& image) {
  const size_t         channels = image.format == wire::PixelFormat::BGRA8 ? 4 : 3;
  std::vector<uint8_t> rgb(size_t(image.width) * image.height * 3);
  for (size_t i = 0; i < size_t(image.width) * image.height && (i + 1) * channels <= image.data.size(); ++i) {
    rgb[i * 3 + 0] = image.data[i * channels + 2];
    rgb[i * 3 + 1] = image.data[i * channels + 1];
    rgb[i * 3 + 2] = image.data[i * channels + 0];
  }
  return rgb;
}

std::array<uint8_t, 3> Palette(const uint32_t id) {
  if (id == 0) {
    return {0, 0, 0};
  }
  const uint32_t h = id * 2654435761u;
  return {uint8_t(64 + (h & 0x7F)), uint8_t(64 + ((h >> 8) & 0x7F)), uint8_t(64 + ((h >> 16) & 0x7F))};
}

std::unique_ptr<wire::ImageEncodingT> Raw() {
  auto encoding         = std::make_unique<wire::ImageEncodingT>();
  encoding->compression = wire::Compression::NONE;
  return encoding;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1) {
    Dir = argv[1];
  }
  fpng::fpng_init();

  auto            connection = std::make_shared<Connection>(ffcli::Address(), ffcli::Port());
  SimulatorClient sim(connection);
  if (!sim.Connect(5)) {
    std::cerr << "no simulator on " << ffcli::Address() << ":" << ffcli::Port() << std::endl;
    return 1;
  }

  const auto id = sim.SpawnRobot("x500", Pose::At(0.0, 0.0, 2.0), "capture_all");
  if (!id.ok) {
    std::cerr << "spawn failed: " << id.error << std::endl;
    return 1;
  }
  RobotClient robot(connection, *id);

  if (auto config = robot.GetRgbCameraConfig(); config.ok) {
    config->encoding = Raw();
    robot.SetRgbCameraConfig(*config);
    for (int attempt = 0; attempt < 5; ++attempt) {
      robot.GetRgbCameraData();
    }
    if (const auto image = robot.GetRgbCameraData(); image.ok) {
      WriteRgb("rgb", ToRgb(*image), image->width, image->height);
    }
  }

  if (auto config = robot.GetStereoCameraConfig(); config.ok) {
    config->encoding = Raw();
    robot.SetStereoCameraConfig(*config);
    robot.GetStereoCameraData();
    if (const auto pair = robot.GetStereoCameraData(); pair.ok) {
      WriteRgb("stereo_left", ToRgb(pair->left), pair->left.width, pair->left.height);
      WriteRgb("stereo_right", ToRgb(pair->right), pair->right.width, pair->right.height);
    }
  }

  if (auto config = robot.GetFisheyeCameraConfig(); config.ok) {
    config->encoding = Raw();
    robot.SetFisheyeCameraConfig(*config);
    robot.GetFisheyeCameraData();
    if (const auto image = robot.GetFisheyeCameraData(); image.ok) {
      WriteRgb("fisheye", ToRgb(*image), image->width, image->height);
    }
  }

  robot.GetRgbSegCameraData();
  if (const auto image = robot.GetRgbSegCameraData(); image.ok) {
    WriteRgb("segmentation", ToRgb(*image), image->width, image->height);
  }

  robot.GetInstanceSegData();
  if (const auto image = robot.GetInstanceSegData(); image.ok) {
    int      w = 0, h = 0, n = 0;
    uint8_t* rgba = stbi_load_from_memory(image->data.data(), static_cast<int>(image->data.size()), &w, &h, &n, 4);
    if (rgba) {
      std::vector<uint8_t> rgb(size_t(w) * h * 3);
      for (size_t i = 0; i < size_t(w) * h; ++i) {
        const auto colour = Palette(rgba[i * 4] | (rgba[i * 4 + 1] << 8) | (rgba[i * 4 + 2] << 16));
        std::copy(colour.begin(), colour.end(), rgb.begin() + i * 3);
      }
      stbi_image_free(rgba);
      WriteRgb("instances", rgb, w, h);
    }
  }

  robot.GetDepthCameraData();
  if (const auto depth = robot.GetDepthCameraData(); depth.ok) {
    uint16_t lo = 65535, hi = 0;
    for (const uint16_t mm : depth->mm) {
      if (mm > 0 && mm < 65535) {
        lo = std::min(lo, mm);
        hi = std::max(hi, mm);
      }
    }
    std::vector<uint8_t> rgb(depth->mm.size() * 3, 0);
    for (size_t i = 0; i < depth->mm.size(); ++i) {
      if (depth->mm[i] > 0 && depth->mm[i] < 65535 && hi > lo) {
        const double t = double(depth->mm[i] - lo) / double(hi - lo);
        rgb[i * 3 + 0] = uint8_t(255 * (1.0 - t));
        rgb[i * 3 + 1] = uint8_t(255 * (1.0 - std::fabs(2.0 * t - 1.0)));
        rgb[i * 3 + 2] = uint8_t(255 * t);
      }
    }
    WriteRgb("depth", rgb, depth->width, depth->height);
  }

  robot.GetEventCameraData();
  for (int i = 0; i < 10; ++i) {
    robot.SetPose(Pose::At(0.0, 0.0, 2.0, 0.02 * i));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  if (const auto events = robot.GetEventCameraData(); events.ok && events->width > 0) {
    std::vector<uint8_t> rgb(size_t(events->width) * events->height * 3, 0);
    for (size_t i = 0; i < events->size(); ++i) {
      const auto e = events->at(i);
      rgb[(size_t(e.y) * events->width + e.x) * 3 + (e.polarity > 0 ? 0 : 2)] = 255;
    }
    WriteRgb("events", rgb, events->width, events->height);
  }

  sim.RemoveRobot(*id);
  return 0;
}
