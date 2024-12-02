#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ueds-connector/game-mode-controller.h"

namespace py = pybind11;

using ueds_connector::GameModeController;
using ueds_connector::CameraCaptureModeEnum;

PYBIND11_MODULE(_uedsGameModeController, m) {
  py::enum_<CameraCaptureModeEnum>(m, "CameraCaptureModeEnum")
      .value("CAPTURE_ALL_FRAMES", CameraCaptureModeEnum::CAPTURE_ALL_FRAMES)
      .value("CAPTURE_ON_MOVEMENT", CameraCaptureModeEnum::CAPTURE_ON_MOVEMENT)
      .value("CAPTURE_ON_DEMAND", CameraCaptureModeEnum::CAPTURE_ON_DEMAND);

  py::class_<GameModeController>(m, "GameModeController")
      .def(py::init<const std::string &, uint16_t>())
      .def("ConnectSimple", &GameModeController::ConnectSimple)
      .def("Ping", &GameModeController::Ping)
      .def("GetDrones", &GameModeController::GetDrones)
      .def("SpawnDrone", &GameModeController::SpawnDrone)
      .def("GetCameraCaptureMode", &GameModeController::GetCameraCaptureMode)
      .def("SetCameraCaptureMode", &GameModeController::SetCameraCaptureMode)
      .def("RemoveDrone", &GameModeController::RemoveDrone)
      .def("GetFps", &GameModeController::GetFps)
      .def(py::pickle(
        [](const GameModeController &controller) {
            return py::make_tuple(
                controller.getAddress(),
                controller.getPort()
            );
        },
        [](py::tuple t) {
            if (t.size() != 2)
                throw std::runtime_error("Invalid state!");

            /* Create a new C++ instance */
            auto controller = new GameModeController(t[0].cast<std::string>(), t[1].cast<uint16_t>());
            const auto connectResult = controller->ConnectSimple();
            if (!connectResult) {
              throw std::runtime_error("Unable to connect to the server!");
            }

            return controller;
        }
    ));
}