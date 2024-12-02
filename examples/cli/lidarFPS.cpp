#include <chrono>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <optional>

#include "ueds-connector/drone-controller.h"

std::unique_ptr<ueds_connector::DroneController> droneController;

void interruptHandler(int s) {
  std::cout << "Got exit signal " << s << std::endl;
  if (droneController != nullptr) {
    droneController.reset();
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

  droneController = std::make_unique<ueds_connector::DroneController>(LOCALHOST, 4000);
  auto connect_result = droneController->Connect();
  if (connect_result != 1) {
    std::cout << "Error connecting to drone controller. connect_result was " << connect_result << std::endl;
    return 1;
  }

  std::ofstream file_stream("lidar_data_log.txt");

  auto start_time = std::chrono::high_resolution_clock::now();
  const int measurement_duration = 2;
  int counter = 0;

  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() <= measurement_duration) {
    auto start = std::chrono::high_resolution_clock::now();
    const auto [res, lidar_data, size] = droneController->GetLidarData();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;

    if(res){
      counter++;
      file_stream << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count() << " " << diff.count() << std::endl;
    }else{
      std::cout << "Fail: get lidar data" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  file_stream.close();

  double fps = static_cast<double>(counter) / measurement_duration;
  std::cout << "Benchmark completed. Results were recorded in lidar_data_log.txt" << std::endl;
  std::cout << "FPS: " << fps << std::endl;

  return 0;
}