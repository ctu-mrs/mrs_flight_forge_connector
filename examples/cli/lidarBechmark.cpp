#include <chrono>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "ueds-connector/drone-controller.h"

std::unique_ptr<ueds_connector::DroneController> droneController;


double test() {
  int num_images = 1000;
  int counter = 0;
  auto start = std::chrono::system_clock::now();
  for (int i = 1; i <= num_images; i++) {
    const auto [res, lidar_data, size] = droneController->GetLidarData();
    if (res) {
      counter++;
    }
  }
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  double elapsed_s = elapsed.count() / 1000000000.0;

  return num_images / elapsed_s;
}



void interruptHandler(int s) {
  std::cout << "Got exit signal " << s << std::endl;
  if (droneController != nullptr) {
    droneController.reset();
  }
  exit(1);
}

int main(int argc, char* argv[]) {
  int port = atoi(argv[1]);

  std::vector<int> vert = {1};
  std::vector<int> hor = {16, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072, 262144};
  std::vector<double> results;
  
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

  droneController = std::make_unique<ueds_connector::DroneController>(LOCALHOST, port);
  auto connect_result = droneController->Connect();
  if (connect_result != 1) {
    std::cout << "Error connecting to drone controller. connect_result was " << connect_result << std::endl;
    return 1;
  }
     
  for (int i = 0; i < vert.size(); i++) {
    for (int j = 0; j < hor.size(); j++) {
      ueds_connector::LidarConfig config{};
      config.Enable = false;
      config.showBeams = false;
      config.BeamHorRays = hor[j];
      config.BeamVertRays = vert[i];
      config.beamLength = 1000;
      config.Frequency = 10;
      config.offset = ueds_connector::Coordinates(0, 0, 6);
      config.orientation = ueds_connector::Rotation(0, 90, 0);
      config.FOVHor = 360;
      config.FOVVert = 50;

      const auto res = droneController->SetLidarConfig(config);

      if (res) {
        std::cout << "SetLidarConfig successful" << std::endl;
      } else {
        std::cout << "SetLidarConfig error" << std::endl;
      }
      double result = 0;
      int num_tests = 5;
      for (int k = 0; k < num_tests; k++) {
        result += test();
      }
      result = result / num_tests;
      results.push_back(result);
      std::cout << " points" << hor[j] << " FPS: " << result << std::endl;
    }
  }
  // save the results to a file
  // format is vert, hor, fps
  std::ofstream file;
  file.open("lidar_results_ueds.txt");
  for (int i = 0; i < vert.size(); i++) {
    for (int j = 0; j < hor.size(); j++) {
      file << vert[i] << " " << hor[j] << " " << results[i * vert.size() + j] << std::endl;
    }
  }
  return 1;
}
