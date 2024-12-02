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
  int num_images = 50;
  int counter = 0;
  auto start = std::chrono::system_clock::now();
  for (int i = 1; i <= num_images; i++) {
    const auto [res, camera_data, size] = droneController->GetCameraData();
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

  std::vector<int> widths = {32, 128, 640, 1280, 1920};
  std::vector<int> heights = {32, 128, 480, 720, 1080};
//  std::vector<int> widths = {32, 64, 128, 512, 640, 1280, 1920};
//  std::vector<int> heights = {32, 64, 128, 512, 480, 720, 1080};
  /* std::vector<int> widths = {1920}; */
  /* std::vector<int> heights = {1080}; */
  std::vector<int> fovs = {20, 30, 60, 90, 120};
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
     
  for (int i = 0; i < widths.size(); i++) {
    for (int j = 0; j < fovs.size(); j++) {
  ueds_connector::CameraConfig config{};
  config.showDebugCamera = false;
  config.angleFOV = fovs[j];
  config.offset = ueds_connector::Coordinates(0, 0, 0);
  config.orientation = ueds_connector::Rotation(0, 0, 0);
  config.Width = widths[i];
  config.Height = heights[i];

  const auto res = droneController->SetCameraConfig(config);
  if (!res) {
    std::cout << "Error setting camera config" << std::endl;
    return 1;
  }else{
    std::cout << "Camera config set" << std::endl;
  }
      double result = 0;
      int num_tests = 5;
      for (int k = 0; k < num_tests; k++) {
        result += test();
      }
      result = result / num_tests;
      results.push_back(result);
      std::cout << "width: " << widths[i] << " height: " << heights[i]
                << " fov: " << fovs[j] << " result: " << result << std::endl;
    }
  }
  // save the results to a file
  // format is width, height, fov, fps
  std::ofstream file;
  file.open("camera_results_warehouse_one_high.txt");
  for (int i = 0; i < widths.size(); i++) {
    for (int j = 0; j < fovs.size(); j++) {
      file << widths[i] << ", " << heights[i] << ", " << fovs[j] << ", "
           << results[i * fovs.size() + j] << std::endl;
    }
  }
  return 1;
}
