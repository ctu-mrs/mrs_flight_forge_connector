// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#include <chrono>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <optional>

#include "flight_forge_connector/game_mode_controller.h"

bool parseInt(std::string choice, int& _int) {
  int x;
  auto delimiter_position = choice.find(' ');
  if (delimiter_position == std::string::npos) {
    return false;
  }
  // trim '2 '
  choice.erase(0, delimiter_position + 1);

  x = std::stoi(choice);
  _int = x;

  return true;
}

bool parseInts(std::string choice, int& a, int& b) {
  int x, y;
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
  x = std::stoi(choice.substr(0, delimiter_position));
  choice.erase(0, delimiter_position + 1);

  y = std::stoi(choice);

  a = x;
  b = y;
  return true;
}

std::unique_ptr<ueds_connector::GameModeController> gameModeController;

void interruptHandler(int s) {
  std::cout << "Got exit signal " << s << std::endl;
  if (gameModeController != nullptr) {
    // release drone controller ptr and destruct (disconnect)
    gameModeController.reset();
  }
  exit(1);
}

int main() {
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

  gameModeController = std::make_unique<ueds_connector::GameModeController>(LOCALHOST, 8000);
  bool connect_result = gameModeController->Connect();
  if (connect_result != 1) {
    std::cout << "Error connecting to game mode controller. connect_result was " << connect_result << std::endl;
    return 1;
  }

  std::string w = "race_2";
  gameModeController->SwitchWorldLevel(ueds_connector::WorldName::Name2Id().at(w));
  connect_result = gameModeController->Disconnect();
  if (!connect_result) {
     std::cout << "[UnrealSimulator] Disconect was not Disconnected succesfully." << connect_result << std::endl;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  while (true) {
    connect_result = gameModeController->Connect();
    if (connect_result != 1) {
      std::cout << "Error connecting to game mode controller. connect_result was " << connect_result << std::endl;
    } else {
      break;
    }
    // ros::Duration(1.0).sleep();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }


  std::string s = "high";
  gameModeController->SetGraphicsSettings(ueds_connector::GraphicsSettings::Name2Id().at(s));

  while (true) {
    std::cout << "Exit: x" << std::endl;
    std::cout << "Ping: 0" << std::endl;
    std::cout << "Get drones: 1" << std::endl;
    std::cout << "Spawn drone: 2 [FRAME ID]" << std::endl;
    std::cout << "Remove drone: 3 [PORT]" << std::endl;
    std::cout << "Get camera capture mode: 4" << std::endl;
    std::cout << "Set camera capture mode: 5 [MODE] (0 - all frames, 1 - on movement, 2 - on demand)" << std::endl;
    std::cout << "Get FPS: 6" << std::endl;
    std::cout << "Set Weather: 7 [WEATHER ID]" << std::endl;
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
      const auto ping_result = gameModeController->Ping();
      std::cout << "Ping: " << (ping_result ? "true" : "false") << std::endl;
    } else if (choice_char == '1') {
      const auto [res, drones] = gameModeController->GetDrones();
      if (!res) {
        std::cout << "Error getting drones" << std::endl;
      } else {
        std::cout << "Drones: ";
        for (const auto& port : drones) {
          std::cout << port << " ";
        }
        std::cout << std::endl;
      }
    } else if (choice_char == '2') {
      
      ueds_connector::Coordinates spawn_coord = ueds_connector::Coordinates(0.0,0.0,0.0);

      const auto [result, world_origin] = gameModeController->GetWorldOrigin();

      if (!result) {
        std::cout << "GetWorldOrigin error" << std::endl;
      } else {
        spawn_coord = world_origin;
      }

      int uav_type_id = 0; //example: "robofly" has ID=3 ;

      bool parse_res = parseInt(choice, uav_type_id);

      if(!parse_res){
        std::cout << "Parse error!!!" << std::endl;
      }else{

      const auto [res, port] = gameModeController->SpawnDroneAtLocation(spawn_coord, uav_type_id);

      if (res) {
        std::cout << "SpawnDrone successful, port: " << port << std::endl;
      } else {
        std::cout << "SpawnDrone error" << std::endl;
      }

      }
    } else if (choice_char == '3') {
      int port;
      const auto parseRes = parseInt(choice, port);
      if (parseRes) {
        const auto success = gameModeController->RemoveDrone(4000);

        if (success) {
          std::cout << "RemoveDrone successful" << std::endl;
        } else {
          std::cout << "RemoveDrone error" << std::endl;
        }
      } else {
        std::cout << "RemoveDrone parse error" << std::endl;
      }
    } else if (choice_char == '4') {
      const auto [res, cameraCaptureMode] = gameModeController->GetCameraCaptureMode();

      if (res) {
        std::cout << "GetCameraCaptureMode successful. Camera capture mode: " << cameraCaptureMode
                  << " (0 - all frames, 1 - on movement, 2 - on demand)" << std::endl;
      } else {
        std::cout << "RemoveDrone error" << std::endl;
      }
    } else if (choice_char == '5') {
      int mode;
      const auto parseRes = parseInt(choice, mode);
      if (parseRes && mode >= 0 && mode <= 2) {
        ueds_connector::CameraCaptureModeEnum captureMode;
        if (mode == 0)
          captureMode = ueds_connector::CAPTURE_ALL_FRAMES;
        else if (mode == 1)
          captureMode = ueds_connector::CAPTURE_ON_MOVEMENT;
        else
          captureMode = ueds_connector::CAPTURE_ON_DEMAND;

        const auto success = gameModeController->SetCameraCaptureMode(captureMode);

        if (success) {
          std::cout << "SetCameraCaptureMode successful" << std::endl;
        } else {
          std::cout << "SetCameraCaptureMode error" << std::endl;
        }
      } else {
        std::cout << "SetCameraCaptureMode parse error" << std::endl;
      }
    } else if (choice_char == '6') {
      const auto [res, fps] = gameModeController->GetFps();

      if (res) {
        std::cout << "GetFps successful, FPS: " << fps << std::endl;
      } else {
        std::cout << "GetFps error" << std::endl;
      }
    } 
    else if (choice_char == '7') {
      int weather_id;
      bool parse_res = parseInt(choice, weather_id);

      if(!parse_res){
        std::cout << "Parse error!!!" << std::endl;
      }else{

      bool res = gameModeController->SetWeather(weather_id);
      if (res) {
        std::cout << "SetWeather successful." << std::endl;
      } else {
        std::cout << "SetWeather error !!!" << std::endl;
      }

      }
    }
    else if (choice_char == '8') {
      int hour, minute;
      bool parse_res = parseInts(choice, hour, minute);

      if(!parse_res){
        std::cout << "Parse error!!!" << std::endl;
      }else{

      bool res = gameModeController->SetDatetime(hour, minute);
      if (res) {
        std::cout << "SetDatetime successful." << std::endl;
      } else {
        std::cout << "SetDatetime error !!!" << std::endl;
      }

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
