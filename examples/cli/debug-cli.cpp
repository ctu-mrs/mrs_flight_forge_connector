// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#include <chrono>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <tuple>
#include <utility>

#include "flight_forge_connector/flight_forge_connector.h"

bool parseDouble(std::string choice, double& _double) {
  double x;
  auto delimiter_position = choice.find(' ');
  if (delimiter_position == std::string::npos) {
    return false;
  }
  // trim '2 '
  choice.erase(0, delimiter_position + 1);

  x = std::stod(choice);
  _double = x;

  return true;
}

bool parseDoubles(std::string choice, double& a, double& b, double& c) {
  double x, y, z;
  auto delimiter_position = choice.find(' ');
  if (delimiter_position == std::string::npos) {
    return false;
  }
  // trim '2 '
  choice.erase(0, delimiter_position + 1);

  delimiter_position = choice.find(' ');
  if (delimiter_position == std::string::npos) {
    return false;
  }
  x = std::stod(choice.substr(0, delimiter_position));
  choice.erase(0, delimiter_position + 1);

  delimiter_position = choice.find(' ');
  if (delimiter_position == std::string::npos) {
    return false;
  }
  y = std::stod(choice.substr(0, delimiter_position));
  choice.erase(0, delimiter_position + 1);

  z = std::stod(choice);

  a = x;
  b = y;
  c = z;
  return true;
}

bool getCoordinates(std::string choice, ueds_connector::Coordinates& coordinates) {
  double a, b, c;
  const auto parseRes = parseDoubles(std::move(choice), a, b, c);

  if (!parseRes) {
    return false;
  }

  coordinates.x = a;
  coordinates.y = b;
  coordinates.z = c;

  return true;
}

bool getRotation(std::string choice, ueds_connector::Rotation& rotation) {
  double a, b, c;
  const auto parseRes = parseDoubles(std::move(choice), a, b, c);

  if (!parseRes) {
    return false;
  }

  rotation.pitch = a;
  rotation.yaw = b;
  rotation.roll = c;

  return true;
}

std::unique_ptr<ueds_connector::UedsConnector> UedsConnector;

void interruptHandler(int s) {
  std::cout << "Got exit signal " << s << std::endl;
  if (UedsConnector != nullptr) {
    // release drone controller ptr and destruct (disconnect)
    UedsConnector.reset();
  }
  exit(1);
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Not enough arguments (missing port)" << std::endl;
    return 1;
  }

  int port = atoi(argv[1]);
  std::cout << "Will connect to port " << port << std::endl;

#ifdef _WIN32
  signal(SIGINT, interruptHandler);
  signal(SIGTERM, interruptHandler);
  signal(SIGABRT, interruptHandler);
#else
  struct sigaction sigIntHandler {};
  sigIntHandler.sa_handler = interruptHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, nullptr);
#endif

  UedsConnector = std::make_unique<ueds_connector::UedsConnector>(LOCALHOST, port);
  auto connect_result = UedsConnector->Connect();
  if (connect_result != 1) {
    std::cout << "Error connecting to drone controller. connect_result was " << connect_result << std::endl;
    return 1;
  }

  while (true) {
    std::cout << "Exit: x" << std::endl;
    std::cout << "Ping: 0" << std::endl;
    std::cout << "Get location: 1" << std::endl;
    std::cout << "Set location: 2 [X] [Y] [Z]" << std::endl;
    std::cout << "Get camera data: 3" << std::endl;
    std::cout << "Get rotation: 4" << std::endl;
    std::cout << "Set rotation: 5" << std::endl;
    std::cout << "Set location and rotation: 6 (fixed 500 500 10000 10 20 30)" << std::endl;
    std::cout << "Get lidar data: 7" << std::endl;
    std::cout << "Get lidar config: 8" << std::endl;
    std::cout << "Set lidar config: 9" << std::endl;
    std::cout << "Get camera config: a" << std::endl;
    std::cout << "Set camera config: b" << std::endl;
    std::cout << "Get rangefinder range data: c" << std::endl;
    std::cout << "Get camera seg data: d" << std::endl;
    std::cout << "Get seg lidar data: e" << std::endl;
    std::cout << "----------------" << std::endl;

    std::string choice;
    std::cout << ": ";
    std::getline(std::cin, choice);

    const auto start = std::chrono::steady_clock::now();

    const auto choice_char = choice[0];
    bool err = false;

    if (choice_char == 'x') {
      break;
    } else if (choice_char == '0') {
      const auto ping_result = UedsConnector->Ping();
      std::cout << "Ping: " << (ping_result ? "true" : "false") << std::endl;
    } else if (choice_char == '1') {
      const auto [res, location] = UedsConnector->GetLocation();
      if (!res) {
        std::cout << "Error getting location" << std::endl;
      } else {
        std::cout << "Location: "
                  << "x: " << location.x << " y: " << location.y << " z: " << location.z << std::endl;
      }
    } else if (choice_char == '2') {
      ueds_connector::Coordinates coordinates{};
      const auto parseRes = getCoordinates(choice, coordinates);

      if (parseRes) {
        const auto [res, teleportedTo, isHit, impactPoint] = UedsConnector->SetLocation(coordinates, true);
        if (res) {
          std::cout << "SetLocation successful: teleported to: (" << teleportedTo.x << ", " << teleportedTo.y << ", "
                    << teleportedTo.z << "), isHit: " << isHit << ", impactPoint: (" << impactPoint.x << ", "
                    << impactPoint.y << ", " << impactPoint.z << ")" << std::endl;
        } else {
          std::cout << "SetLocation error" << std::endl;
        }
      } else {
        std::cout << "SetLocation error" << std::endl;
      }
    } else if (choice_char == '3') {
      const auto [res, camera_data, stamp, size] = UedsConnector->GetRgbCameraData();
      if (res) {
        std::cout
            << "camera_data(ms): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
            << std::endl;
        std::cout << "GetCameraData successful. Size: " << size << std::endl;
        std::cout
            << "Elapsed to get img (ms): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
            << std::endl;

        std::ofstream file;
        file.open("testImage.png", std::ios::binary);
        //        file.write(reinterpret_cast<const char*>(camera_data.get()), size);
        copy(camera_data.cbegin(), camera_data.cend(), std::ostreambuf_iterator<char>(file));
        file.close();
      } else {
        std::cout << "GetCameraData errored, size was 0" << std::endl;
      }
    } else if (choice_char == '4') {
      const auto [res, rotation] = UedsConnector->GetRotation();
      if (!res) {
        std::cout << "Error getting rotation" << std::endl;
      } else {
        std::cout << "Rotation: "
                  << "pitch: " << rotation.pitch << " yaw: " << rotation.yaw << " roll: " << rotation.roll << std::endl;
      }
    } else if (choice_char == '5') {
      ueds_connector::Rotation rotation{};
      auto parseRes = getRotation(choice, rotation);
      if (parseRes) {
        const auto [res, rotatedTo, isHit, impactPoint] = UedsConnector->SetRotation(rotation);
        if (res) {
          std::cout << "SetRotation successful: rotatedTo" << rotatedTo.toString() << ", isHit: " << isHit << ", impactPoint: " << impactPoint.toString() << std::endl;
        } else {
          std::cout << "SetRotation error" << std::endl;
        }
      } else {
        std::cout << "SetRotation parse error" << std::endl;
      }
    } else if (choice_char == '6') {
      ueds_connector::Coordinates location{4000, -12200, 1000};
      ueds_connector::Rotation rotation{10, 20, 30};

      const auto [res, teleportedTo, rotatedTo, isHit, impactPoint] = UedsConnector->SetLocationAndRotation(location, rotation, false);
      if (res) {
        std::cout << "SetLocationAndRotation successful: teleported to: " << teleportedTo.toString() << ", rotatedTo: " << rotatedTo.toString() << ", isHit: " << isHit << ", impactPoint: " << impactPoint.toString() << std::endl;
      } else {
        std::cout << "SetLocationAndRotation error" << std::endl;
      }
    } else if (choice_char == '7') {
      const auto [res, data, start] = UedsConnector->GetLidarData();
      if (res) {
        std::cout << "GetLidarData successful. Start: (" << start.x << ", " << start.y << ", " << start.z
                  << "), Data: " << std::endl;
        for (auto& i : data) {
          std::cout << "(distance: " << i.distance << ", directionX: " << i.directionX
                    << ", directionY: " << i.directionY << ", directionZ: " << i.directionZ << ")" << std::endl;
        }
        std::cout << std::endl;
      } else {
        std::cout << "GetLidarData error" << std::endl;
      }
    } else if (choice_char == '8') {
      const auto [res, config] = UedsConnector->GetLidarConfig();
      if (res) {
        std::cout << "GetLidarConfig successful: (Enable: " << config.Enable
                  << "showBeams: " << config.showBeams
                   << ", beamLength: " << config.beamLength
                  << ", BeamHorRays: " << config.BeamHorRays << ", BeamVertRays: " << config.BeamVertRays 
                  << ", Frequency: " << config.Frequency
                  << ", offset: " << config.offset.toString()
                  << ", rotation: " << config.orientation.toString()
                  << ", FOVHorLeft: " << config.FOVHorLeft << ", FOVHorLeft: " << config.FOVHorLeft
                  << ", FOVVerUp: " << config.FOVVertUp << ", FOVVerDown: " << config.FOVVertDown
                  << ")" << std::endl;
      } else {
        std::cout << "GetLidarConfig error" << std::endl;
      }
    } else if (choice_char == '9') {
      ueds_connector::LidarConfig config{};
      config.Enable = false;
      config.showBeams = true;
      config.BeamHorRays = 64;
      config.BeamVertRays = 20;
      config.beamLength = 300;
      config.Frequency = 10;
      config.offset = ueds_connector::Coordinates(0, 0, 6);
      config.orientation = ueds_connector::Rotation(0, 90, 0);
      config.FOVHorLeft = 180;
      config.FOVHorRight = 180;
      config.FOVVertUp = 52;
      config.FOVVertDown = 7;

      const auto res = UedsConnector->SetLidarConfig(config);

      if (res) {
        std::cout << "SetLidarConfig successful" << std::endl;
      } else {
        std::cout << "SetLidarConfig error" << std::endl;
      }
    } else if (choice_char == 'a') {
      const auto [res, config] = UedsConnector->GetRgbCameraConfig();
      if (res) {
        std::cout << "GetCameraConfig successful: (showDebugCamera: " << config.show_debug_camera_
                   << ", offset: " << config.offset_.toString()
                  << ", rotation: " << config.orientation_.toString() << ", Width: " << config.width_ << ", Height: " << config.height_  << ")" << std::endl;
      } else {
        std::cout << "GetCameraConfig error" << std::endl;
      }
    } else if (choice_char == 'b') {
      ueds_connector::RgbCameraConfig config{};

      config.show_debug_camera_ = true;

      config.fov_ = 120;

      config.offset_ = ueds_connector::Coordinates(0, 0, 0);

      config.orientation_ = ueds_connector::Rotation(0, 0, 0);
      
      config.width_ = 999;//640; <1000

      config.height_ = 999;//480; limit <1000

      config.enable_motion_blur_ = false;

      config.motion_blur_amount_ = 0.5;

      config.motion_blur_distortion_ = 1.0;

      const auto res = UedsConnector->SetRgbCameraConfig(config);

      if (res) {
        std::cout << "SetCameraConfig successful" << std::endl;
      } else {
        std::cout << "SetCameraConfig error" << std::endl;
      }
    }
    else if (choice_char == 'c') {
      const auto [res, range_data] = UedsConnector->GetRangefinderData();
      if (res) {
        std::cout
            << "camera_data(ms): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
            << std::endl;
        std::cout << "GetRangefinderData successful. Range:  " << range_data / 100.0 << " [m]" << std::endl;
        std::cout
            << "Elapsed to get img (ms): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
            << std::endl;
      } else {
        std::cout << "GetRangefinderData errored" << std::endl;
      }
    }
    else if (choice_char == 'd') {
      const auto [res, camera_data, stamp, size] = UedsConnector->GetRgbSegmented();
      if (res) {
        std::cout
            << "camera_data(ms): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
            << std::endl;
        std::cout << "GetCameraData successful. Size: " << size << std::endl;
        std::cout
            << "Elapsed to get img (ms): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
            << std::endl;

        std::ofstream file;
        file.open("SegImage.png", std::ios::binary);
        //        file.write(reinterpret_cast<const char*>(camera_data.get()), size);
        copy(camera_data.cbegin(), camera_data.cend(), std::ostreambuf_iterator<char>(file));
        file.close();
      } else {
        std::cout << "GetCameraData errored, size was 0" << std::endl;
      }
    }
    else if (choice_char == 'e') {
      const auto [res, data, start] = UedsConnector->GetLidarSegData();
      if (res) {
        std::cout << "GetLidarSegData successful. Start: (" << start.x << ", " << start.y << ", " << start.z
                  << "), Data: " << std::endl;
        for (auto& i : data) {
          std::cout << "(distance: " << i.distance << ", directionX: " << i.directionX
                    << ", directionY: " << i.directionY << ", directionZ: " << i.directionZ << ", segmentation: " << i.segmentation << ")" << std::endl;
        }
        std::cout << std::endl;
      } else {
        std::cout << "GetLidarSegData error" << std::endl;
      }
    }
    else {
      err = true;
    }

    if (err) {
      std::cout << "Unknown choice: " << choice << std::endl;
    }
    std::cout << "Elapsed(ms): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()
              << std::endl;
    std::cout << "----------------" << std::endl;
  }

  return 0;
}
