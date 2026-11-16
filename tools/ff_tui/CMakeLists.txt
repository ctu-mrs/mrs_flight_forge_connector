# Standalone build of the FlightForge TUI. It compiles the connector sources
# itself, so the repository's own CMake files stay untouched:
#
#   cmake -S tools/ff_tui -B build/ff_tui && cmake --build build/ff_tui -j
#
cmake_minimum_required(VERSION 3.15)
project(ff_tui CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)
set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FTXUI_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI.git
  GIT_TAG v5.0.0
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(ftxui)

set(FF_REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# the connector library, same sources as src/CMakeLists.txt
add_library(ff_connector STATIC
  ${FF_REPO}/src/socket_client.cpp
  ${FF_REPO}/src/flight_forge_connector.cpp
  ${FF_REPO}/src/game_mode_controller.cpp)
target_include_directories(ff_connector PUBLIC ${FF_REPO}/include)
find_package(Threads REQUIRED)
target_link_libraries(ff_connector PUBLIC Threads::Threads)

# fpng for writing PNGs (the repo already carries it under .gitman)
add_library(ff_fpng STATIC ${FF_REPO}/.gitman/fpng/src/fpng.cpp)
target_include_directories(ff_fpng PUBLIC ${FF_REPO}/.gitman/fpng/src)
target_compile_definitions(ff_fpng PRIVATE FPNG_NO_SSE=1)

add_executable(ff_tui
  main.cpp
  app.cpp
  forms.cpp
  actions_game_mode.cpp
  actions_drone.cpp
  actions_sensors.cpp
  actions_scene.cpp
  drive_mode.cpp
  viz_image.cpp
  viz_lidar.cpp
  pixel_overlay.cpp
  worker.cpp)
target_include_directories(ff_tui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
target_link_libraries(ff_tui PRIVATE ff_connector ff_fpng ftxui::screen ftxui::dom ftxui::component)
