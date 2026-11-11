#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>

#include <flight_forge_connector/simulator_client.h>

namespace py = pybind11;

using flightforge::CameraCaptureModeEnum;
using flightforge::Coordinates;
using flightforge::SimulatorClient;

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

  py::class_<SimulatorClient>(m, "SimulatorClient")
      .def(py::init<const std::string&, uint16_t>())
      .def("ConnectSimple", &SimulatorClient::ConnectSimple, py::call_guard<py::gil_scoped_release>())
      .def("Ping", &SimulatorClient::Ping, py::call_guard<py::gil_scoped_release>())
      .def(
          "GetMeta",
          [](SimulatorClient& client) {
            bool                                    ok;
            Serializable::Common::GetMeta::Response meta;
            {
              py::gil_scoped_release release;
              std::tie(ok, meta) = client.GetMeta();
            }
            py::list ops;
            for (const auto& op : meta.ops_) {
              py::dict d;
              d["scope"]    = op.scope_;
              d["id"]       = op.id_;
              d["name"]     = op.name_;
              d["request"]  = op.request_;
              d["response"] = op.response_;
              d["hash"]     = op.hash_;
              ops.append(d);
            }
            py::dict out;
            out["api_version"]    = py::make_tuple(meta.api_version_major_, meta.api_version_minor_);
            out["catalogue_hash"] = meta.catalogue_hash_;
            out["ops"]            = ops;
            out["catalogue_json"] = meta.catalogue_json_;
            return py::make_tuple(ok, out);
          })
      .def("CheckCompatibility", &SimulatorClient::CheckCompatibility, py::call_guard<py::gil_scoped_release>(), py::arg("op_names") = std::vector<std::string>{})
      .def("Disconnect", &SimulatorClient::Disconnect, py::call_guard<py::gil_scoped_release>())
      .def("GetDrones", &SimulatorClient::GetDrones, py::call_guard<py::gil_scoped_release>())
      .def("SpawnDrone", &SimulatorClient::SpawnDrone, py::call_guard<py::gil_scoped_release>())
      .def("SpawnDroneAtLocation", &SimulatorClient::SpawnDroneAtLocation, py::call_guard<py::gil_scoped_release>(), py::arg("location"), py::arg("frame_name"))
      .def("RemoveDrone", &SimulatorClient::RemoveDrone, py::call_guard<py::gil_scoped_release>())
      .def("GetCameraCaptureMode", &SimulatorClient::GetCameraCaptureMode, py::call_guard<py::gil_scoped_release>())
      .def("SetCameraCaptureMode", &SimulatorClient::SetCameraCaptureMode, py::call_guard<py::gil_scoped_release>())
      .def("GetFps", &SimulatorClient::GetFps, py::call_guard<py::gil_scoped_release>())
      .def(
          "GetClassMap",
          [](SimulatorClient& controller) {
            bool                                           ok;
            Serializable::GameMode::GetClassMap::Response map;
            {
              py::gil_scoped_release release;
              std::tie(ok, map) = controller.GetClassMap();
            }
            py::list classes;
            for (const auto& c : map.classes_) {
              py::dict d;
              d["id"]        = c.id;
              d["parent_id"] = c.parent_id;
              d["path"]      = c.path;
              classes.append(d);
            }
            return py::make_tuple(ok, classes, map.hash_);
          })
      .def("GetApiVersion", &SimulatorClient::GetApiVersion, py::call_guard<py::gil_scoped_release>())
      .def("GetTime", &SimulatorClient::GetTime, py::call_guard<py::gil_scoped_release>())
      .def("SetGraphicsSettings", &SimulatorClient::SetGraphicsSettings, py::call_guard<py::gil_scoped_release>())
      .def("SwitchWorldLevel", &SimulatorClient::SwitchWorldLevel, py::call_guard<py::gil_scoped_release>(), py::arg("level_name_or_package_path"))
      .def("SetForestDensity", &SimulatorClient::SetForestDensity, py::call_guard<py::gil_scoped_release>())
      .def("SetForestHillyLevel", &SimulatorClient::SetForestHillyLevel, py::call_guard<py::gil_scoped_release>())
      .def("GetWorldOrigin", &SimulatorClient::GetWorldOrigin, py::call_guard<py::gil_scoped_release>())
      .def("SetWeather", &SimulatorClient::SetWeather, py::call_guard<py::gil_scoped_release>())
      .def("SetDatetime", &SimulatorClient::SetDatetime, py::call_guard<py::gil_scoped_release>())
      .def("SetMutualDroneVisibility", &SimulatorClient::SetMutualDroneVisibility, py::call_guard<py::gil_scoped_release>())
      .def(
          "SpawnObjectsFromYaml",
          [](SimulatorClient& controller, const std::string& yaml) {
            std::string                       error;
            std::pair<bool, std::vector<int>> result;
            {
              py::gil_scoped_release release;
              result = controller.SpawnObjectsFromYaml(yaml, error);
            }
            return py::make_tuple(result.first, result.second, error);
          },
          py::arg("yaml"), "Returns (success, object_ids, error).")
      .def("RemoveSpawnedObject", &SimulatorClient::RemoveSpawnedObject, py::call_guard<py::gil_scoped_release>())
      .def("RemoveAllSpawnedObjects", &SimulatorClient::RemoveAllSpawnedObjects, py::call_guard<py::gil_scoped_release>())
      .def("MoveSpawnedObject", &SimulatorClient::MoveSpawnedObject, py::call_guard<py::gil_scoped_release>(), py::arg("object_id"), py::arg("position"),
           py::arg("orientation"), py::arg("scale") = std::array<double, 3>{1.0, 1.0, 1.0})
      .def("ListSpawnedObjects", &SimulatorClient::ListSpawnedObjects, py::call_guard<py::gil_scoped_release>())
      .def("ExportScene", &SimulatorClient::ExportScene, py::call_guard<py::gil_scoped_release>())
      .def(
          "GetSimTime",
          [](SimulatorClient& controller) {
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
          [](SimulatorClient& controller, double dilation) {
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
          [](SimulatorClient& controller, double dt, double adaptive_min_dt) {
            double applied = 0.0;
            bool   ok;
            {
              py::gil_scoped_release release;
              ok = controller.SetFixedTimestep(dt, adaptive_min_dt, applied);
            }
            return py::make_tuple(ok, applied);
          },
          py::arg("dt"), py::arg("adaptive_min_dt") = 0.0)
      .def("SetLockstep", &SimulatorClient::SetLockstep, py::call_guard<py::gil_scoped_release>(), py::arg("enabled") = true)
      .def(
          "Step",
          [](SimulatorClient& controller, int ticks) {
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
          [](SimulatorClient& controller, const std::array<double, 3>& center, const std::array<double, 3>& extent, uint32_t max_triangles) {
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
          [](SimulatorClient& controller, py::array_t<float, py::array::c_style | py::array::forcecast> points, double max_radius, double tolerance,
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
          [](SimulatorClient& controller, const std::string& path_prefix, const std::string& name_filter) {
            bool                                      truncated = false;
            std::pair<bool, std::vector<std::string>> result;
            {
              py::gil_scoped_release release;
              result = controller.ListAssets(path_prefix, name_filter, truncated);
            }
            return py::make_tuple(result.first, result.second, truncated);
          },
          py::arg("path_prefix") = "", py::arg("name_filter") = "", "Returns (success, assets, truncated).")
      .def(
          "LoadLibrary",
          [](SimulatorClient& controller, const std::string& path) {
            std::string          error;
            std::pair<bool, int> result;
            {
              py::gil_scoped_release release;
              result = controller.LoadLibrary(path, error);
            }
            return py::make_tuple(result.first, result.second, error);
          },
          py::arg("path"), "Returns (success, count, error).")
      .def(
          "ListLibrary",
          [](SimulatorClient& controller, const std::string& library, const std::string& cls, const std::string& filter, int offset, int limit) {
            int                                                                 total = 0;
            std::pair<bool, std::vector<Serializable::GameMode::LibraryObject>> result;
            {
              py::gil_scoped_release release;
              result = controller.ListLibrary(library, cls, filter, offset, limit, total);
            }
            py::list objects;
            for (const auto& o : result.second) {
              py::dict d;
              d["library"]     = o.library;
              d["id"]          = o.id;
              d["spawn_name"]  = o.spawn_name;
              d["class"]       = o.cls;
              d["description"] = o.description;
              d["license"]     = o.license;
              d["thumbnail"]   = o.thumbnail;
              d["scale"]       = o.scale;
              d["height_m"]    = o.height_m;
              d["runtime"]     = o.runtime;
              objects.append(d);
            }
            return py::make_tuple(result.first, objects, total);
          },
          py::arg("library") = "", py::arg("cls") = "", py::arg("filter") = "", py::arg("offset") = 0, py::arg("limit") = 200,
          "Returns (success, objects, total).")
      .def(py::pickle(
          [](const SimulatorClient& controller) { return py::make_tuple(controller.getAddress(), controller.getPort()); },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");

            auto controller = new SimulatorClient(t[0].cast<std::string>(), t[1].cast<uint16_t>());
            if (!controller->ConnectSimple()) {
              throw std::runtime_error("Unable to connect to the server!");
            }

            return controller;
          }));
}
