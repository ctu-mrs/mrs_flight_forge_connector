// Interactive robot and sensor ops for one robot.
//   debug_cli <robot_id>            (FLIGHTFORGE_ADDRESS and FLIGHTFORGE_PORT override 127.0.0.1:8600)

#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include <flight_forge_connector/robot_client.h>

#include "common.h"

using namespace flightforge;
using Clock = std::chrono::steady_clock;

namespace
{

void Menu() {
  std::cout << "x exit\n"
            << "1 get state\n"
            << "2 set pose [X Y Z] [YAW deg]\n"
            << "3 rgb frame\n"
            << "4 lidar scan\n"
            << "5 lidar config\n"
            << "6 rgb camera config\n"
            << "7 rangefinder\n"
            << "8 segmentation frame\n"
            << "9 camera info\n"
            << "a list sensors\n"
            << "b set joint velocities [rad/s ...]\n"
            << "----------------" << std::endl;
}

void Report(const Status& status, const Clock::time_point start) {
  const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  std::cout << (status.ok ? "ok" : "error: " + status.error) << " (" << ms << " ms)" << std::endl;
}

std::string Describe(const Pose& p) {
  std::ostringstream out;
  out << "(" << p.position.x << ", " << p.position.y << ", " << p.position.z << ") yaw " << p.orientation.Yaw() * 180.0 / M_PI << " deg";
  return out.str();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "usage: debug_cli <robot_id>" << std::endl;
    return 1;
  }

  RobotClient robot(ffcli::Address(), ffcli::Port(), std::atoi(argv[1]));
  if (!robot.Connect(5)) {
    std::cerr << "no simulator on " << ffcli::Address() << ":" << ffcli::Port() << std::endl;
    return 1;
  }

  std::string line;
  for (Menu(); std::cout << ": " && std::getline(std::cin, line); Menu()) {
    std::istringstream in(line);
    std::string        command;
    in >> command;
    const auto start = Clock::now();

    if (command == "x") {
      break;
    } else if (command == "1") {
      const auto state = robot.GetState();
      Report(state, start);
      if (state.ok && state->state.base) {
        std::cout << "  " << Describe(*state->state.base) << " at " << state->stamp << " s" << std::endl;
      }
    } else if (command == "2") {
      double x = 0, y = 0, z = 2, yaw = 0;
      in >> x >> y >> z >> yaw;
      const auto result = robot.SetPose(Pose::At(x, y, z, yaw * M_PI / 180.0), true);
      Report(result, start);
      if (result.ok) {
        std::cout << "  at " << Describe(result->base) << (result->hit ? " after a hit" : "") << std::endl;
      }
    } else if (command == "3") {
      const auto image = robot.GetRgbCameraData();
      Report(image, start);
      if (image.ok) {
        std::cout << "  " << image->width << "x" << image->height << " " << wire::EnumNameCompression(image->compression) << ", " << image->data.size()
                  << " bytes at " << image->stamp << " s" << std::endl;
      }
    } else if (command == "4") {
      const auto scan = robot.GetLidarScan(true, true);
      Report(scan, start);
      if (scan.ok) {
        size_t hits = 0;
        for (const auto& p : scan->points) {
          hits += std::isfinite(p[0]) ? 1 : 0;
        }
        std::cout << "  " << scan->points.size() << " points, " << hits << " returns, " << scan->rows << "x" << scan->cols << ", sensor at "
                  << Describe(scan->sensor_pose) << std::endl;
      }
    } else if (command == "5") {
      const auto config = robot.GetLidarConfig();
      Report(config, start);
      if (config.ok) {
        std::cout << "  " << config->horizontal_rays << "x" << config->vertical_rays << " rays, " << config->range_m << " m, " << config->frequency
                  << " Hz" << (config->livox ? ", livox " + config->livox_pattern : "") << std::endl;
      }
    } else if (command == "6") {
      const auto config = robot.GetRgbCameraConfig();
      Report(config, start);
      if (config.ok) {
        std::cout << "  " << config->width << "x" << config->height << ", fov " << config->fov << " deg" << std::endl;
      }
    } else if (command == "7") {
      const auto range = robot.GetRangefinderData();
      Report(range, start);
      if (range.ok) {
        std::cout << "  " << range->range << " m" << std::endl;
      }
    } else if (command == "8") {
      const auto image = robot.GetRgbSegCameraData();
      Report(image, start);
      if (image.ok) {
        std::cout << "  " << image->width << "x" << image->height << ", " << image->data.size() << " bytes" << std::endl;
      }
    } else if (command == "9") {
      const auto cameras = robot.GetCameraInfo();
      Report(cameras, start);
      for (const auto& camera : cameras.value) {
        std::cout << "  sensor " << camera.sensor_id << " " << wire::EnumNameSensorType(camera.sensor_type) << " " << camera.width << "x" << camera.height
                  << " fx " << camera.fx << " cx " << camera.cx << std::endl;
      }
    } else if (command == "a") {
      const auto sensors = robot.ListSensors();
      Report(sensors, start);
      for (const auto& sensor : sensors.value) {
        std::cout << "  " << sensor.id << " " << wire::EnumNameSensorType(sensor.type) << " on " << sensor.link << std::endl;
      }
    } else if (command == "b") {
      std::vector<double> speeds;
      for (double speed; in >> speed;) {
        speeds.push_back(speed);
      }
      Report(robot.SetJointVelocities(speeds), start);
    }
  }

  return 0;
}
