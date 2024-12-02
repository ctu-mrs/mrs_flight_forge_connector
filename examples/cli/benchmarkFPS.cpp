

#include <chrono>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <optional>

#include "ueds-connector/game-mode-controller.h"

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
  auto connect_result = gameModeController->Connect();
  if (connect_result != 1) {
    std::cout << "Error connecting to game mode controller. connect_result was " << connect_result << std::endl;
    return 1;
  }


  std::ofstream file_stream("fps_log.txt");


  auto start_time = std::chrono::high_resolution_clock::now();
  const int measurement_duration = 10;

  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() <= measurement_duration) {

    auto [res, fps] = gameModeController->GetFps();

    if(res){
      file_stream << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count() << " " << fps << std::endl;
    }else{
      std::cout << "Fail: get fps" << std::endl;
    }



    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }


  file_stream.close();

  std::cout << "Benchmark dokončen. Výsledky byly zaznamenány do fps_log.txt" << std::endl;


  return 0;
}

