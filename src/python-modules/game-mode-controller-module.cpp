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
      .def("ConnectSimple", &GameModeController::ConnectSimple, py::call_guard<py::gil_scoped_release>())
      .def("Ping", &GameModeController::Ping, py::call_guard<py::gil_scoped_release>())
      .def("Disconnect", &GameModeController::Disconnect, py::call_guard<py::gil_scoped_release>())
      .def("GetDrones", &GameModeController::GetDrones, py::call_guard<py::gil_scoped_release>())
      .def("SpawnDrone", &GameModeController::SpawnDrone, py::call_guard<py::gil_scoped_release>())
      .def("SpawnDroneAtLocation", &GameModeController::SpawnDroneAtLocation, py::call_guard<py::gil_scoped_release>(), py::arg("location"), py::arg("frame_name"))
      .def("RemoveDrone", &GameModeController::RemoveDrone, py::call_guard<py::gil_scoped_release>())
      .def("GetCameraCaptureMode", &GameModeController::GetCameraCaptureMode, py::call_guard<py::gil_scoped_release>())
      .def("SetCameraCaptureMode", &GameModeController::SetCameraCaptureMode, py::call_guard<py::gil_scoped_release>())
      .def("GetFps", &GameModeController::GetFps, py::call_guard<py::gil_scoped_release>())
      .def("GetApiVersion", &GameModeController::GetApiVersion, py::call_guard<py::gil_scoped_release>())
      .def("GetTime", &GameModeController::GetTime, py::call_guard<py::gil_scoped_release>())
      .def("SetGraphicsSettings", &GameModeController::SetGraphicsSettings, py::call_guard<py::gil_scoped_release>())
      .def("SwitchWorldLevel", &GameModeController::SwitchWorldLevel, py::call_guard<py::gil_scoped_release>(), py::arg("level_name_or_package_path"))
      .def("SetForestDensity", &GameModeController::SetForestDensity, py::call_guard<py::gil_scoped_release>())
      .def("SetForestHillyLevel", &GameModeController::SetForestHillyLevel, py::call_guard<py::gil_scoped_release>())
      .def("GetWorldOrigin", &GameModeController::GetWorldOrigin, py::call_guard<py::gil_scoped_release>())
      .def("SetWeather", &GameModeController::SetWeather, py::call_guard<py::gil_scoped_release>())
      .def("SetDatetime", &GameModeController::SetDatetime, py::call_guard<py::gil_scoped_release>())
      .def("SetMutualDroneVisibility", &GameModeController::SetMutualDroneVisibility, py::call_guard<py::gil_scoped_release>())
      .def(
          "SpawnObjectsFromYaml",
          [](GameModeController& controller, const std::string& yaml) {
            std::string                       error;
            std::pair<bool, std::vector<int>> result;
            {
              py::gil_scoped_release release;
              result = controller.SpawnObjectsFromYaml(yaml, error);
            }
            return py::make_tuple(result.first, result.second, error);
          },
          py::arg("yaml"), "Returns (success, object_ids, error).")
      .def("RemoveSpawnedObject", &GameModeController::RemoveSpawnedObject, py::call_guard<py::gil_scoped_release>())
      .def("RemoveAllSpawnedObjects", &GameModeController::RemoveAllSpawnedObjects, py::call_guard<py::gil_scoped_release>())
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
