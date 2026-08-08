#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <flight_forge_connector/game_mode_controller.h>

namespace py = pybind11;

using ueds_connector::CameraCaptureModeEnum;
using ueds_connector::Coordinates;
using ueds_connector::GameModeController;

PYBIND11_MODULE(flight_forge_game_mode, m) {
  m.doc() = "FlightForge game-mode client (API 0.14). Worlds and drone frames are addressed by name; objects spawn from YAML documents.";

  py::enum_<CameraCaptureModeEnum>(m, "CameraCaptureModeEnum")
      .value("CAPTURE_ALL_FRAMES", CameraCaptureModeEnum::CAPTURE_ALL_FRAMES)
      .value("CAPTURE_ON_MOVEMENT", CameraCaptureModeEnum::CAPTURE_ON_MOVEMENT)
      .value("CAPTURE_ON_DEMAND", CameraCaptureModeEnum::CAPTURE_ON_DEMAND);

  py::class_<GameModeController>(m, "GameModeController")
      .def(py::init<const std::string&, uint16_t>())
      .def("ConnectSimple", &GameModeController::ConnectSimple)
      .def("Ping", &GameModeController::Ping)
      .def("Disconnect", &GameModeController::Disconnect)
      .def("GetDrones", &GameModeController::GetDrones)
      .def("SpawnDrone", &GameModeController::SpawnDrone)
      .def("SpawnDroneAtLocation", &GameModeController::SpawnDroneAtLocation, py::arg("location"), py::arg("frame_name"))
      .def("RemoveDrone", &GameModeController::RemoveDrone)
      .def("GetCameraCaptureMode", &GameModeController::GetCameraCaptureMode)
      .def("SetCameraCaptureMode", &GameModeController::SetCameraCaptureMode)
      .def("GetFps", &GameModeController::GetFps)
      .def("GetApiVersion", &GameModeController::GetApiVersion)
      .def("GetTime", &GameModeController::GetTime)
      .def("SetGraphicsSettings", &GameModeController::SetGraphicsSettings)
      .def("SwitchWorldLevel", &GameModeController::SwitchWorldLevel, py::arg("level_name_or_package_path"))
      .def("SetForestDensity", &GameModeController::SetForestDensity)
      .def("SetForestHillyLevel", &GameModeController::SetForestHillyLevel)
      .def("GetWorldOrigin", &GameModeController::GetWorldOrigin)
      .def("SetWeather", &GameModeController::SetWeather)
      .def("SetDatetime", &GameModeController::SetDatetime)
      .def("SetMutualDroneVisibility", &GameModeController::SetMutualDroneVisibility)
      .def(
          "SpawnObjectsFromYaml",
          [](GameModeController& controller, const std::string& yaml) {
            std::string error;
            auto        result = controller.SpawnObjectsFromYaml(yaml, error);
            return py::make_tuple(result.first, result.second, error);
          },
          py::arg("yaml"), "Returns (success, object_ids, error).")
      .def("RemoveSpawnedObject", &GameModeController::RemoveSpawnedObject)
      .def("RemoveAllSpawnedObjects", &GameModeController::RemoveAllSpawnedObjects)
      .def(py::pickle(
          [](const GameModeController& controller) { return py::make_tuple(controller.getAddress(), controller.getPort()); },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");

            auto controller = new GameModeController(t[0].cast<std::string>(), t[1].cast<uint16_t>());
            if (!controller->ConnectSimple()) {
              throw std::runtime_error("Unable to connect to the server!");
            }

            return controller;
          }));
}
