#include <chrono>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include "ueds-connector/drone-controller.h"
#include "ueds-connector/game-mode-controller.h"

using DroneController = ueds_connector::DroneController;

std::unique_ptr<ueds_connector::GameModeController> gameModeController;


double test(std::vector<std::unique_ptr<DroneController>>& droneControllers) {
  int num_images = 50;
  std::vector<int> counters(droneControllers.size(), 0);
  std::vector<std::thread> threads;

  auto getData = [&counters, &num_images, &droneControllers](int index) {
    for (int i = 1; i <= num_images; i++) {
      const auto [res, camera_data, size] = droneControllers[index]->GetCameraData();
      if (res) {
        //std::cout << "drone " << index << "camera captured" << std::endl;
        counters[index]++;
      }
    }
  };

  auto start = std::chrono::system_clock::now();
  for (int i = 0; i < droneControllers.size(); ++i) {
    threads.emplace_back(getData, i);
  }

  for (auto& thread : threads) {
    thread.join();
  }

  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  double elapsed_s = elapsed.count() / 1000000000.0;

  std::vector<double> fps(droneControllers.size());
  for (int i = 0; i < droneControllers.size(); ++i) {
    fps[i] = counters[i] / elapsed_s;
  }

  double avg_fps = std::accumulate(fps.begin(), fps.end(), 0.0) / fps.size();

  return avg_fps;
}



//void interruptHandler(int s) {
//  std::cout << "Got exit signal " << s << std::endl;
//  if (droneController != nullptr) {
//    droneController.reset();
//  }
//  exit(1);
//}

int main(int argc, char* argv[]) {

  std::vector<double> positionX = {0.0, 0.0, 0.0};
  std::vector<double> positionY = {0.0, -1000.0, -2000.0};
  std::vector<double> positionZ = {100.0, 100.0, 100.0};

  std::vector<int> widths = {32, 128, 640, 1280, 1920};
  std::vector<int> heights = {32, 128, 480, 720, 1080};
  //  std::vector<int> widths = {32, 64, 128, 512, 640, 1280, 1920};
  //  std::vector<int> heights = {32, 64, 128, 512, 480, 720, 1080};
  /* std::vector<int> widths = {1920}; */
  /* std::vector<int> heights = {1080}; */
  std::vector<int> fovs = {20, 30, 60, 90, 120};
  std::vector<double> results;

  std::cout << "Spawning drones: " << positionX.size() << std::endl;
//#ifdef _WIN32
//  signal(SIGINT, interruptHandler);
//  signal(SIGTERM, interruptHandler);
//  signal(SIGABRT, interruptHandler);
//#else
//  struct sigaction sigIntHandler {};
//  sigIntHandler.sa_handler = interruptHandler;
//  sigemptyset(&sigIntHandler.sa_mask);
//  sigIntHandler.sa_flags = 0;
//  sigaction(SIGINT, &sigIntHandler, nullptr);
//#endif

  gameModeController = std::make_unique<ueds_connector::GameModeController>(LOCALHOST, 8000);
  auto connect_result = gameModeController->Connect();
  if (connect_result != 1) {
    std::cout << "Error connecting to game mode controller. connect_result was " << connect_result << std::endl;
    return 1;
  }

  ueds_connector::Coordinates world_origin{};
  std::vector<std::unique_ptr<DroneController>> droneControllers;

  for(int i=0; i<positionX.size(); i++){
    bool res;
    int port;
    std::tie(res, port) = gameModeController->SpawnDrone();
    if (res) {
      std::cout << "SpawnDrone successful, port: " << port << std::endl;

      droneControllers.emplace_back(std::make_unique<ueds_connector::DroneController>(LOCALHOST, port));
      connect_result = droneControllers[i]->Connect();
      if (connect_result != 1) {
        std::cout << "Error connecting to drone controller. connect_result was " << connect_result << std::endl;
        return 1;
      }

      if(i==0){
        std::tie(res, world_origin) = droneControllers[i]->GetLocation();
        if (!res) {
          std::cout << "Error getting location of world origin" << std::endl;
        } else {
          std::cout << "Location: "
                    << "x: " << world_origin.x << " y: " << world_origin.y << " z: " << world_origin.z << std::endl;
        }
      }

      ueds_connector::Coordinates coordinates{};
      coordinates.x = positionX[i] + world_origin.x;
      coordinates.y = positionY[i] + world_origin.y;
      coordinates.z = positionZ[i] + world_origin.z;
      ueds_connector::Coordinates teleportedTo{}; ueds_connector::Coordinates impactPoint{}; bool isHit;
//      std::tie(res, teleportedTo, isHit, impactPoint) = droneControllers[i]->SetLocation(coordinates, false);
//      //std::this_thread::sleep_for(std::chrono::seconds(1));
//      while (!res) {
//        std::cout << "SetLocation error" << std::endl;
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//        std::tie(res, teleportedTo, isHit, impactPoint) = droneControllers[i]->SetLocation(coordinates, false);
//      }
//      std::cout << "SetLocation successful: teleported to: (" << teleportedTo.x << ", " << teleportedTo.y << ", "
//                << teleportedTo.z << "), isHit: " << isHit << ", impactPoint: (" << impactPoint.x << ", "
//          << impactPoint.y << ", " << impactPoint.z << ")" << std::endl;

    } else {
      std::cout << "SpawnDrone error" << std::endl;
      return 3;
    }
  }


  for (int i = 0; i < widths.size(); i++) {
    for (int fov : fovs) {
      ueds_connector::CameraConfig config{};
      config.showDebugCamera = false;
      config.angleFOV = fov;
      config.offset = ueds_connector::Coordinates(0, 0, 0);
      config.orientation = ueds_connector::Rotation(0, 0, 0);
      config.Width = widths[i];
      config.Height = heights[i];

      for(const auto & droneController : droneControllers){
        const auto res = droneController->SetCameraConfig(config);
        if (!res) {
          std::cout << "Error setting camera config" << std::endl;
          return 1;
        }else{
          std::cout << "Camera config set" << std::endl;
        }
      }

      double result = 0;
      int num_tests = 5;
      for (int k = 0; k < num_tests; k++) {
        result += test(droneControllers);
      }
      result = result / num_tests;
      results.push_back(result);
      std::cout << "width: " << widths[i] << " height: " << heights[i]
                << " fov: " << fov << " result: " << result << std::endl;
    }
  }
  // save the results to a file
  // format is width, height, fov, fps
  std::ofstream file;
  file.open("camera_results_warehouse_uavs_3_low.txt");
  for (int i = 0; i < widths.size(); i++) {
    for (int j = 0; j < fovs.size(); j++) {
      file << widths[i] << ", " << heights[i] << ", " << fovs[j] << ", "
           << results[i * fovs.size() + j] << std::endl;
    }
  }
  return 1;
}

