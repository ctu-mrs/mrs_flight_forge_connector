#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ueds-connector/drone-controller.h"

namespace py = pybind11;

using ueds_connector::DroneController;
using ueds_connector::Coordinates;
using ueds_connector::Rotation;
using ueds_connector::LidarData;
using ueds_connector::LidarConfig;
using ueds_connector::CameraConfig;

PYBIND11_MODULE(_uedsDroneController, m) {
  py::class_<Coordinates>(m, "Coordinates")
      .def(py::init<>())
      .def(py::init<double, double, double>())
      .def_readwrite("x", &Coordinates::x)
      .def_readwrite("y", &Coordinates::y)
      .def_readwrite("z", &Coordinates::z)
      .def("__repr__", &Coordinates::toString);

  py::class_<Rotation>(m, "Rotation")
      .def(py::init<>())
      .def(py::init<double, double, double>())
      .def_readwrite("pitch", &Rotation::pitch)
      .def_readwrite("yaw", &Rotation::yaw)
      .def_readwrite("roll", &Rotation::roll)
      .def("__repr__", &Rotation::toString);

  py::class_<LidarData>(m, "LidarData")
      .def(py::init<>())
      .def_readwrite("distance", &LidarData::distance)
      .def_readwrite("directionX", &LidarData::directionX)
      .def_readwrite("directionY", &LidarData::directionY)
      .def_readwrite("directionZ", &LidarData::directionZ)
      .def("__repr__", &LidarData::toString);

  py::class_<LidarConfig>(m, "LidarConfig")
      .def(py::init<>())
      .def(py::init<bool, bool, double, double, double, double, double, double, double, const Coordinates, const Rotation, double, double>())
      .def_readwrite("Enable", &LidarConfig::Enable)
      .def_readwrite("showBeams", &LidarConfig::showBeams)
      .def_readwrite("BeamHorRays", &LidarConfig::BeamHorRays) 
      .def_readwrite("BeamVertRays", &LidarConfig::BeamVertRays) 
      .def_readwrite("beamLength", &LidarConfig::beamLength)
      .def_readwrite("Frequency", &LidarConfig::Frequency)
      .def_readwrite("offset", &LidarConfig::offset)
      .def_readwrite("orientation", &LidarConfig::orientation)
      .def_readwrite("FOVHor", &LidarConfig::FOVHor) 
      .def_readwrite("FOVVert", &LidarConfig::FOVVert) 
      .def("__repr__", &LidarConfig::toString);

  py::class_<CameraConfig>(m, "CameraConfig")
      .def(py::init<>())
      .def(py::init<bool, double, const Coordinates, const Rotation>())
      .def_readwrite("showDebugCamera", &CameraConfig::showDebugCamera)
      .def_readwrite("angleFOV", &CameraConfig::angleFOV)
      .def_readwrite("offset", &CameraConfig::offset)
      .def_readwrite("orientation", &CameraConfig::orientation)
      .def("__repr__", &CameraConfig::toString);

  py::class_<DroneController>(m, "DroneController")
      .def(py::init<const std::string &, uint16_t>())
      .def("ConnectSimple", &DroneController::ConnectSimple)
      .def("Ping", &DroneController::Ping)
      .def("GetLocation", &DroneController::GetLocation)
      .def("SetLocation", &DroneController::SetLocation)
      .def("GetCameraData", &DroneController::GetCameraData)
      .def("GetRotation", &DroneController::GetRotation)
      .def("SetRotation", &DroneController::SetRotation)
      .def("SetLocationAndRotation", &DroneController::SetLocationAndRotation)
      .def("GetLidarData", &DroneController::GetLidarData)
      .def("GetLidarConfig", &DroneController::GetLidarConfig)
      .def("SetLidarConfig", &DroneController::SetLidarConfig)
      .def("GetCameraConfig", &DroneController::GetCameraConfig)
      .def("SetCameraConfig", &DroneController::SetCameraConfig)
      .def("GetMoveLineVisible", &DroneController::GetMoveLineVisible)
      .def("SetMoveLineVisible", &DroneController::SetMoveLineVisible);
}
