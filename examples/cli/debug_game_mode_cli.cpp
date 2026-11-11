// Interactive simulator ops.
//   debug_game_mode_cli            (FLIGHTFORGE_ADDRESS and FLIGHTFORGE_PORT override 127.0.0.1:8600)

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include <flight_forge_connector/simulator_client.h>

#include "common.h"

using namespace flightforge;

namespace
{

void Menu() {
  std::cout << "x exit\n"
            << "0 ping\n"
            << "1 list robots\n"
            << "2 spawn robot [NAME] [X Y Z] [LABEL]\n"
            << "3 remove robot [ID]\n"
            << "4 get capture mode\n"
            << "5 set capture mode [0 all frames, 1 on movement, 2 on demand]\n"
            << "6 fps and time\n"
            << "7 set weather [ID]\n"
            << "8 set daytime [HOUR] [MINUTE]\n"
            << "9 switch world [LEVEL or friendly name]\n"
            << "a set graphics [0 low .. 4 cinematic]\n"
            << "b mutual visibility [0|1]\n"
            << "c spawn objects from yaml [PATH]\n"
            << "d remove spawned object [ID] (negative removes all)\n"
            << "e world origin\n"
            << "----------------" << std::endl;
}

void Report(const Status& status) {
  std::cout << (status.ok ? "ok" : "error: " + status.error) << std::endl;
}

}  // namespace

int main() {
  SimulatorClient sim(ffcli::Address(), ffcli::Port());
  std::cout << "connecting to " << ffcli::Address() << ":" << ffcli::Port() << std::endl;
  if (!sim.Connect(5)) {
    std::cerr << "no simulator there" << std::endl;
    return 1;
  }

  const auto info = sim.GetInfo();
  if (!info.ok || info->major != SimulatorClient::kApiMajor) {
    std::cerr << "the simulator speaks API " << info->major << "." << info->minor << ", this client " << SimulatorClient::kApiMajor << "."
              << SimulatorClient::kApiMinor << std::endl;
    return 1;
  }

  std::string line;
  for (Menu(); std::cout << ": " && std::getline(std::cin, line); Menu()) {
    std::istringstream in(line);
    std::string        command;
    in >> command;

    if (command == "x") {
      break;
    } else if (command == "0") {
      Report(sim.Ping());
    } else if (command == "1") {
      const auto robots = sim.ListRobots();
      Report(robots);
      for (const auto& robot : robots.value) {
        std::cout << "  " << robot.id << " " << robot.name << " '" << robot.label << "' " << wire::EnumNameRobotKind(robot.kind) << std::endl;
      }
    } else if (command == "2") {
      std::string name = "x500", label;
      double      x = 0, y = 0, z = 2;
      in >> name >> x >> y >> z >> label;
      const auto id = sim.SpawnRobot(name, Pose::At(x, y, z), label);
      Report(id);
      if (id.ok) {
        std::cout << "  robot " << *id << std::endl;
      }
    } else if (command == "3") {
      int id = -1;
      in >> id;
      Report(sim.RemoveRobot(id));
    } else if (command == "4") {
      const auto mode = sim.GetCaptureMode();
      Report(mode);
      if (mode.ok) {
        std::cout << "  " << wire::EnumNameCaptureMode(*mode) << std::endl;
      }
    } else if (command == "5") {
      int mode = 0;
      in >> mode;
      Report(sim.SetCaptureMode(static_cast<wire::CaptureMode>(mode)));
    } else if (command == "6") {
      const auto fps  = sim.GetFps();
      const auto time = sim.GetTime();
      if (fps.ok && time.ok) {
        std::cout << "  " << *fps << " fps, sim " << time->sim_time << " s, dilation " << time->time_dilation << std::endl;
      } else {
        Report(fps.ok ? Status(time) : Status(fps));
      }
    } else if (command == "7") {
      int weather = 0;
      in >> weather;
      Report(sim.SetWeather(static_cast<wire::Weather>(weather)));
    } else if (command == "8") {
      int hour = 12, minute = 0;
      in >> hour >> minute;
      Report(sim.SetDaytime(hour, minute));
    } else if (command == "9") {
      std::string level;
      in >> level;
      const auto& names = WorldName::Name2Level();
      Report(sim.SwitchWorld(names.count(level) ? names.at(level) : level));
    } else if (command == "a") {
      int quality = 2;
      in >> quality;
      Report(sim.SetGraphicsSettings(static_cast<wire::GraphicsQuality>(quality)));
    } else if (command == "b") {
      int enabled = 1;
      in >> enabled;
      Report(sim.SetMutualVisibility(enabled != 0));
    } else if (command == "c") {
      std::string path;
      in >> path;
      std::ifstream file(path);
      if (!file) {
        std::cout << "cannot read " << path << std::endl;
        continue;
      }
      const auto ids = sim.SpawnObjectsFromYaml(std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()));
      Report(ids);
      for (const int id : ids.value) {
        std::cout << "  object " << id << std::endl;
      }
    } else if (command == "d") {
      int id = -1;
      in >> id;
      const auto removed = sim.RemoveSpawnedObject(id);
      Report(removed);
      if (removed.ok) {
        std::cout << "  removed " << *removed << std::endl;
      }
    } else if (command == "e") {
      const auto origin = sim.GetWorldOrigin();
      Report(origin);
      if (origin.ok) {
        std::cout << "  ground frame origin at Unreal (" << origin->x << ", " << origin->y << ", " << origin->z << ") cm" << std::endl;
      }
    }
  }

  return 0;
}
