cmake_minimum_required(VERSION 3.15)

#set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-static")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(LIBRARY_NAME flight_forge_connector)
project(${LIBRARY_NAME} VERSION 1.0)

find_package(Threads REQUIRED)

# Add the fpng library from gitman
add_library(fpng STATIC
    .gitman/fpng/src/fpng.cpp
    .gitman/fpng/src/lodepng.cpp
    .gitman/fpng/src/pvpngreader.cpp
)
target_include_directories(fpng PUBLIC .gitman/fpng/src)

option(SSE "Enable SSE4.1" ON)

if (SSE)
  target_compile_definitions(fpng PRIVATE FPNG_NO_SSE=0)
  target_compile_options(fpng PRIVATE -msse4.1 -mpclmul)
else()
  target_compile_definitions(fpng PRIVATE FPNG_NO_SSE=1)
endif()
# End of fpng library

# add_subdirectory(.gitman/fpng)
add_subdirectory(src)

option(BUILD_EXAMPLES "Build examples" ON)
if (BUILD_EXAMPLES)
    add_subdirectory(examples/cli)
endif()

option(BUILD_TESTS "Build the wire-protocol tests" OFF)
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
