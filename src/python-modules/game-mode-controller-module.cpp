#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>

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

  py::class_<Serializable::GameMode::SpawnedObjectInfo>(m, "SpawnedObjectInfo")
      .def_readonly("id", &Serializable::GameMode::SpawnedObjectInfo::id)
      .def_readonly("asset", &Serializable::GameMode::SpawnedObjectInfo::asset)
      .def_readonly("stencil", &Serializable::GameMode::SpawnedObjectInfo::stencil)
      .def_readonly("position", &Serializable::GameMode::SpawnedObjectInfo::position)
      .def_readonly("orientation", &Serializable::GameMode::SpawnedObjectInfo::orientation)
      .def_readonly("scale", &Serializable::GameMode::SpawnedObjectInfo::scale);

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
      .def("MoveSpawnedObject", &GameModeController::MoveSpawnedObject, py::call_guard<py::gil_scoped_release>(), py::arg("object_id"), py::arg("position"),
           py::arg("orientation"), py::arg("scale") = std::array<double, 3>{1.0, 1.0, 1.0})
      .def("ListSpawnedObjects", &GameModeController::ListSpawnedObjects, py::call_guard<py::gil_scoped_release>())
      .def("ExportScene", &GameModeController::ExportScene, py::call_guard<py::gil_scoped_release>())
      .def(
          "GetSimTime",
          [](GameModeController& controller) {
            double sim = 0.0, dilation = 1.0;
            bool   ok;
            {
              py::gil_scoped_release release;
              ok = controller.GetSimTime(sim, dilation);
            }
            return py::make_tuple(ok, sim, dilation);
          })
      .def(
          "SetTimeDilation",
          [](GameModeController& controller, double dilation) {
            double applied = 1.0;
            bool   ok;
            {
              py::gil_scoped_release release;
              ok = controller.SetTimeDilation(dilation, applied);
            }
            return py::make_tuple(ok, applied);
          },
          py::arg("dilation"))
      .def(
          "SetFixedTimestep",
          [](GameModeController& controller, double dt, double adaptive_min_dt) {
            double applied = 0.0;
            bool   ok;
            {
              py::gil_scoped_release release;
              ok = controller.SetFixedTimestep(dt, adaptive_min_dt, applied);
            }
            return py::make_tuple(ok, applied);
          },
          py::arg("dt"), py::arg("adaptive_min_dt") = 0.0)
      .def("SetLockstep", &GameModeController::SetLockstep, py::call_guard<py::gil_scoped_release>(), py::arg("enabled") = true)
      .def(
          "Step",
          [](GameModeController& controller, int ticks) {
            double   sim_time   = 0.0;
            uint64_t tick_count = 0;
            bool     ok;
            {
              py::gil_scoped_release release;
              ok = controller.Step(ticks, sim_time, tick_count);
            }
            return py::make_tuple(ok, sim_time, tick_count);
          },
          py::arg("ticks") = 1)
      .def(
          "GetWorldMesh",
          [](GameModeController& controller, const std::array<double, 3>& center, const std::array<double, 3>& extent, uint32_t max_triangles) {
            std::vector<float>    vertices;
            std::vector<uint32_t> indices;
            bool                  truncated = false;
            int32_t               sources   = 0;
            bool                  ok;
            {
              py::gil_scoped_release release;
              ok = controller.GetWorldMesh(center, extent, max_triangles, vertices, indices, truncated, sources);
            }
            // buffer-protocol arrays: a 2M-triangle mesh must not become a python list
            py::array_t<float>    v(static_cast<py::ssize_t>(vertices.size()));
            py::array_t<uint32_t> i(static_cast<py::ssize_t>(indices.size()));
            std::memcpy(v.mutable_data(), vertices.data(), vertices.size() * sizeof(float));
            std::memcpy(i.mutable_data(), indices.data(), indices.size() * sizeof(uint32_t));
            return py::make_tuple(ok, v, i, truncated, sources);
          },
          py::arg("center") = std::array<double, 3>{0.0, 0.0, 0.0}, py::arg("extent") = std::array<double, 3>{0.0, 0.0, 0.0},
          py::arg("max_triangles") = 0)
      .def(
          "GetDistanceBatch",
          [](GameModeController& controller, py::array_t<float, py::array::c_style | py::array::forcecast> points, double max_radius, double tolerance,
             bool terrain_only) {
            std::vector<float> query(points.data(), points.data() + points.size());
            std::vector<float> distances;
            bool               ok;
            {
              py::gil_scoped_release release;
              ok = controller.GetDistanceBatch(query, max_radius, tolerance, terrain_only, distances);
            }
            py::array_t<float> d(static_cast<py::ssize_t>(distances.size()));
            std::memcpy(d.mutable_data(), distances.data(), distances.size() * sizeof(float));
            return py::make_tuple(ok, d);
          },
          py::arg("points"), py::arg("max_radius") = 5.0, py::arg("tolerance") = 0.001, py::arg("terrain_only") = false)
      .def(
          "ListAssets",
          [](GameModeController& controller, const std::string& path_prefix, const std::string& name_filter) {
            bool                                      truncated = false;
            std::pair<bool, std::vector<std::string>> result;
            {
              py::gil_scoped_release release;
              result = controller.ListAssets(path_prefix, name_filter, truncated);
            }
            return py::make_tuple(result.first, result.second, truncated);
          },
          py::arg("path_prefix") = "", py::arg("name_filter") = "", "Returns (success, assets, truncated).")
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
