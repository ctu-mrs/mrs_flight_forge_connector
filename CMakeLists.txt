cmake_minimum_required(VERSION 3.15)

#set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-static")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(LIBRARY_NAME flight_forge_connector)
project(${LIBRARY_NAME} VERSION 1.0)

if (WIN32)
    link_libraries(ws2_32 wsock32)
endif()

add_subdirectory(src)

option(BUILD_EXAMPLES "Build examples" ON)
if (BUILD_EXAMPLES)
    add_subdirectory(examples/cli)
endif()

# option(BUILD_PYTHON_LIB "Build python lib" ON)
# if (BUILD_PYTHON_LIB)
#     add_subdirectory(extern/pybind11)

#     file (GLOB PYTHON_LIB_SOURCE_FILES "src/*.cpp")

#     include_directories("./include")

#     pybind11_add_module(
#             _uedsGameModeController
#             ${PYTHON_LIB_SOURCE_FILES}
#             src/python-modules/game-mode-controller-module.cpp
#             )
#     target_link_libraries(_uedsGameModeController PUBLIC)

#     pybind11_add_module(
#             _uedsDroneController
#             ${PYTHON_LIB_SOURCE_FILES}
#             src/python-modules/drone-controller-module.cpp
#     )
#     target_link_libraries(_uedsDroneController PUBLIC)
# endif()
