#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <flight_forge_connector/flight_forge_connector.h>

namespace py = pybind11;

using ueds_connector::CameraEvent;
using ueds_connector::CameraExposure;
using ueds_connector::CameraIntrinsics;
using ueds_connector::CameraLensEffects;
using ueds_connector::Coordinates;
using ueds_connector::DeviceInfo;
using ueds_connector::EventCameraConfig;
using ueds_connector::FisheyeCameraConfig;
using ueds_connector::LidarConfig;
using ueds_connector::LidarData;
using ueds_connector::LidarIntData;
using ueds_connector::LidarSegData;
using ueds_connector::RgbCameraConfig;
using ueds_connector::Rotation;
using ueds_connector::SensorInfo;
using ueds_connector::UedsConnector;

using StereoCameraConfig = Serializable::Drone::StereoCameraConfig;

PYBIND11_MODULE(flight_forge_drone, m) {
  m.doc() = "FlightForge drone-side client (API 0.14). Sensor calls take sensor_id; -1 addresses the default sensor of that type.";

  /* basic types //{ */

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

  py::class_<SensorInfo>(m, "SensorInfo")
      .def(py::init<>())
      .def_readwrite("id", &SensorInfo::id)
      .def_readwrite("type", &SensorInfo::type);

  py::class_<DeviceInfo>(m, "DeviceInfo")
      .def(py::init<>())
      .def_readwrite("name", &DeviceInfo::name)
      .def_readwrite("description", &DeviceInfo::description);

  py::enum_<ueds_connector::SensorTypeEnum>(m, "SensorType")
      .value("RGB_CAMERA", ueds_connector::SENSOR_RGB_CAMERA)
      .value("LIDAR", ueds_connector::SENSOR_LIDAR)
      .value("RANGEFINDER", ueds_connector::SENSOR_RANGEFINDER)
      .value("LIDAR_LIVOX", ueds_connector::SENSOR_LIDAR_LIVOX)
      .value("RGB_SEG_CAMERA", ueds_connector::SENSOR_RGB_SEG_CAMERA)
      .value("STEREO_CAMERA", ueds_connector::SENSOR_STEREO_CAMERA)
      .value("EVENT_CAMERA", ueds_connector::SENSOR_EVENT_CAMERA)
      .value("FISHEYE_CAMERA", ueds_connector::SENSOR_FISHEYE_CAMERA)
      .export_values();

  //}

  /* lidar types //{ */

  py::class_<LidarData>(m, "LidarData")
      .def(py::init<>())
      .def_readwrite("distance", &LidarData::distance)
      .def_readwrite("directionX", &LidarData::directionX)
      .def_readwrite("directionY", &LidarData::directionY)
      .def_readwrite("directionZ", &LidarData::directionZ)
      .def("__repr__", &LidarData::toString);

  py::class_<LidarSegData>(m, "LidarSegData")
      .def(py::init<>())
      .def_readwrite("distance", &LidarSegData::distance)
      .def_readwrite("directionX", &LidarSegData::directionX)
      .def_readwrite("directionY", &LidarSegData::directionY)
      .def_readwrite("directionZ", &LidarSegData::directionZ)
      .def_readwrite("segmentation", &LidarSegData::segmentation)
      .def("__repr__", &LidarSegData::toString);

  py::class_<LidarIntData>(m, "LidarIntData")
      .def(py::init<>())
      .def_readwrite("distance", &LidarIntData::distance)
      .def_readwrite("directionX", &LidarIntData::directionX)
      .def_readwrite("directionY", &LidarIntData::directionY)
      .def_readwrite("directionZ", &LidarIntData::directionZ)
      .def_readwrite("intensity", &LidarIntData::intensity)
      .def("__repr__", &LidarIntData::toString);

  py::class_<LidarConfig>(m, "LidarConfig")
      .def(py::init<>())
      .def_readwrite("Enable", &LidarConfig::Enable)
      .def_readwrite("showBeams", &LidarConfig::showBeams)
      .def_readwrite("BeamHorRays", &LidarConfig::BeamHorRays)
      .def_readwrite("BeamVertRays", &LidarConfig::BeamVertRays)
      .def_readwrite("beamLength", &LidarConfig::beamLength)
      .def_readwrite("Frequency", &LidarConfig::Frequency)
      .def_readwrite("offset", &LidarConfig::offset)
      .def_readwrite("orientation", &LidarConfig::orientation)
      .def_readwrite("FOVHorLeft", &LidarConfig::FOVHorLeft)
      .def_readwrite("FOVHorRight", &LidarConfig::FOVHorRight)
      .def_readwrite("FOVVertUp", &LidarConfig::FOVVertUp)
      .def_readwrite("FOVVertDown", &LidarConfig::FOVVertDown)
      .def_readwrite("Livox", &LidarConfig::Livox)
      .def_readwrite("LivoxSensor", &LidarConfig::LivoxSensor)
      .def("__repr__", &LidarConfig::toString);

  //}

  /* camera configs //{ */

  py::class_<CameraExposure>(m, "CameraExposure")
      .def(py::init<>())
      .def_readwrite("manual", &CameraExposure::manual_)
      .def_readwrite("shutter_time", &CameraExposure::shutter_time_)
      .def_readwrite("iso", &CameraExposure::iso_)
      .def_readwrite("ev_compensation", &CameraExposure::ev_compensation_);

  py::class_<CameraLensEffects>(m, "CameraLensEffects")
      .def(py::init<>())
      .def_readwrite("fstop", &CameraLensEffects::fstop_)
      .def_readwrite("focal_distance", &CameraLensEffects::focal_distance_)
      .def_readwrite("sensor_width_mm", &CameraLensEffects::sensor_width_mm_)
      .def_readwrite("vignette_intensity", &CameraLensEffects::vignette_intensity_)
      .def_readwrite("chromatic_aberration", &CameraLensEffects::chromatic_aberration_)
      .def_readwrite("bloom_intensity", &CameraLensEffects::bloom_intensity_)
      .def_readwrite("lens_flare_intensity", &CameraLensEffects::lens_flare_intensity_)
      .def_readwrite("white_temp", &CameraLensEffects::white_temp_)
      .def_readwrite("white_tint", &CameraLensEffects::white_tint_)
      .def_readwrite("motion_blur_from_shutter", &CameraLensEffects::motion_blur_from_shutter_);

  py::class_<CameraIntrinsics>(m, "CameraIntrinsics")
      .def(py::init<>())
      .def_readwrite("use_custom", &CameraIntrinsics::use_custom_)
      .def_readwrite("fx", &CameraIntrinsics::fx_)
      .def_readwrite("fy", &CameraIntrinsics::fy_)
      .def_readwrite("cx", &CameraIntrinsics::cx_)
      .def_readwrite("cy", &CameraIntrinsics::cy_);

  py::class_<RgbCameraConfig>(m, "RgbCameraConfig")
      .def(py::init<>())
      .def_readwrite("show_debug_camera", &RgbCameraConfig::show_debug_camera_)
      .def_readwrite("offset", &RgbCameraConfig::offset_)
      .def_readwrite("orientation", &RgbCameraConfig::orientation_)
      .def_readwrite("fov", &RgbCameraConfig::fov_)
      .def_readwrite("width", &RgbCameraConfig::width_)
      .def_readwrite("height", &RgbCameraConfig::height_)
      .def_readwrite("enable_temporal_aa", &RgbCameraConfig::enable_temporal_aa_)
      .def_readwrite("enable_raytracing", &RgbCameraConfig::enable_raytracing_)
      .def_readwrite("enable_hdr", &RgbCameraConfig::enable_hdr_)
      .def_readwrite("enable_motion_blur", &RgbCameraConfig::enable_motion_blur_)
      .def_readwrite("motion_blur_amount", &RgbCameraConfig::motion_blur_amount_)
      .def_readwrite("motion_blur_distortion", &RgbCameraConfig::motion_blur_distortion_)
      .def_readwrite("exposure", &RgbCameraConfig::exposure_)
      .def_readwrite("lens", &RgbCameraConfig::lens_)
      .def_readwrite("intrinsics", &RgbCameraConfig::intrinsics_);

  py::class_<StereoCameraConfig>(m, "StereoCameraConfig")
      .def(py::init<>())
      .def_readwrite("show_debug_camera", &StereoCameraConfig::show_debug_camera_)
      .def_readwrite("offset_x_left", &StereoCameraConfig::offset_x_left_)
      .def_readwrite("offset_y_left", &StereoCameraConfig::offset_y_left_)
      .def_readwrite("offset_z_left", &StereoCameraConfig::offset_z_left_)
      .def_readwrite("offset_x_right", &StereoCameraConfig::offset_x_right_)
      .def_readwrite("offset_y_right", &StereoCameraConfig::offset_y_right_)
      .def_readwrite("offset_z_right", &StereoCameraConfig::offset_z_right_)
      .def_readwrite("rotation_pitch_left", &StereoCameraConfig::rotation_pitch_left_)
      .def_readwrite("rotation_yaw_left", &StereoCameraConfig::rotation_yaw_left_)
      .def_readwrite("rotation_roll_left", &StereoCameraConfig::rotation_roll_left_)
      .def_readwrite("rotation_pitch_right", &StereoCameraConfig::rotation_pitch_right_)
      .def_readwrite("rotation_yaw_right", &StereoCameraConfig::rotation_yaw_right_)
      .def_readwrite("rotation_roll_right", &StereoCameraConfig::rotation_roll_right_)
      .def_readwrite("fov", &StereoCameraConfig::fov_)
      .def_readwrite("width", &StereoCameraConfig::width_)
      .def_readwrite("height", &StereoCameraConfig::height_)
      // the wire struct holds its own intrinsics type; convert so Python sees one CameraIntrinsics class
      .def_property(
          "intrinsics",
          [](const StereoCameraConfig& config) {
            CameraIntrinsics intrinsics;
            intrinsics.use_custom_ = config.intrinsics_.use_custom_;
            intrinsics.fx_         = config.intrinsics_.fx_;
            intrinsics.fy_         = config.intrinsics_.fy_;
            intrinsics.cx_         = config.intrinsics_.cx_;
            intrinsics.cy_         = config.intrinsics_.cy_;
            return intrinsics;
          },
          [](StereoCameraConfig& config, const CameraIntrinsics& intrinsics) {
            config.intrinsics_.use_custom_ = intrinsics.use_custom_;
            config.intrinsics_.fx_         = intrinsics.fx_;
            config.intrinsics_.fy_         = intrinsics.fy_;
            config.intrinsics_.cx_         = intrinsics.cx_;
            config.intrinsics_.cy_         = intrinsics.cy_;
          })
      .def_readwrite("enable_temporal_aa", &StereoCameraConfig::enable_temporal_aa_)
      .def_readwrite("enable_raytracing", &StereoCameraConfig::enable_raytracing_)
      .def_readwrite("enable_hdr", &StereoCameraConfig::enable_hdr_);

  py::class_<EventCameraConfig>(m, "EventCameraConfig")
      .def(py::init<>())
      .def_readwrite("show_debug_camera", &EventCameraConfig::show_debug_camera_)
      .def_readwrite("offset", &EventCameraConfig::offset_)
      .def_readwrite("orientation", &EventCameraConfig::orientation_)
      .def_readwrite("fov", &EventCameraConfig::fov_)
      .def_readwrite("width", &EventCameraConfig::width_)
      .def_readwrite("height", &EventCameraConfig::height_)
      .def_readwrite("contrast_threshold_pos", &EventCameraConfig::contrast_threshold_pos_)
      .def_readwrite("contrast_threshold_neg", &EventCameraConfig::contrast_threshold_neg_)
      .def_readwrite("use_sim_time", &EventCameraConfig::use_sim_time_);

  py::class_<CameraEvent>(m, "CameraEvent")
      .def(py::init<>())
      .def_readwrite("x", &CameraEvent::x)
      .def_readwrite("y", &CameraEvent::y)
      .def_readwrite("polarity", &CameraEvent::polarity)
      .def_readwrite("stamp", &CameraEvent::stamp);

  py::class_<FisheyeCameraConfig>(m, "FisheyeCameraConfig")
      .def(py::init<>())
      .def_readwrite("show_debug_camera", &FisheyeCameraConfig::show_debug_camera_)
      .def_readwrite("offset", &FisheyeCameraConfig::offset_)
      .def_readwrite("orientation", &FisheyeCameraConfig::orientation_)
      .def_readwrite("fov", &FisheyeCameraConfig::fov_)
      .def_readwrite("width", &FisheyeCameraConfig::width_)
      .def_readwrite("height", &FisheyeCameraConfig::height_)
      .def_readwrite("lens_model", &FisheyeCameraConfig::lens_model_);

  //}

  /* connector //{ */

  py::class_<UedsConnector>(m, "DroneController")
      .def(py::init<const std::string&, uint16_t>())
      .def("ConnectSimple", &UedsConnector::ConnectSimple, py::call_guard<py::gil_scoped_release>())
      .def("Ping", &UedsConnector::Ping, py::call_guard<py::gil_scoped_release>())
      .def("Disconnect", &UedsConnector::Disconnect, py::call_guard<py::gil_scoped_release>())
      .def("GetLocation", &UedsConnector::GetLocation, py::call_guard<py::gil_scoped_release>())
      .def("GetCrashState", &UedsConnector::GetCrashState, py::call_guard<py::gil_scoped_release>())
      .def("SetLocation", &UedsConnector::SetLocation, py::call_guard<py::gil_scoped_release>())
      .def("GetRotation", &UedsConnector::GetRotation, py::call_guard<py::gil_scoped_release>())
      .def("SetRotation", &UedsConnector::SetRotation, py::call_guard<py::gil_scoped_release>())
      .def("SetLocationAndRotation", &UedsConnector::SetLocationAndRotation, py::call_guard<py::gil_scoped_release>())
      .def("SetLocationAndRotationAsync", &UedsConnector::SetLocationAndRotationAsync, py::call_guard<py::gil_scoped_release>())
      .def("GetMoveLineVisible", &UedsConnector::GetMoveLineVisible, py::call_guard<py::gil_scoped_release>())
      .def("SetMoveLineVisible", &UedsConnector::SetMoveLineVisible, py::call_guard<py::gil_scoped_release>())
      .def("GetRgbCameraData", &UedsConnector::GetRgbCameraData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetStereoCameraData", &UedsConnector::GetStereoCameraData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetRgbSegmented", &UedsConnector::GetRgbSegmented, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetRgbCameraConfig", &UedsConnector::GetRgbCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("SetRgbCameraConfig", &UedsConnector::SetRgbCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("config"), py::arg("sensor_id") = -1)
      .def("GetStereoCameraConfig", &UedsConnector::GetStereoCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("SetStereoCameraConfig", &UedsConnector::SetStereoCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("config"), py::arg("sensor_id") = -1)
      .def("GetEventCameraData", &UedsConnector::GetEventCameraData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetEventCameraConfig", &UedsConnector::GetEventCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("SetEventCameraConfig", &UedsConnector::SetEventCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("config"), py::arg("sensor_id") = -1)
      .def("GetFisheyeCameraData", &UedsConnector::GetFisheyeCameraData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetFisheyeCameraConfig", &UedsConnector::GetFisheyeCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("SetFisheyeCameraConfig", &UedsConnector::SetFisheyeCameraConfig, py::call_guard<py::gil_scoped_release>(), py::arg("config"), py::arg("sensor_id") = -1)
      .def("GetRangefinderData", &UedsConnector::GetRangefinderData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetLidarData", &UedsConnector::GetLidarData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetLidarSegData", &UedsConnector::GetLidarSegData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetLidarIntData", &UedsConnector::GetLidarIntData, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("GetLidarConfig", &UedsConnector::GetLidarConfig, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("SetLidarConfig", &UedsConnector::SetLidarConfig, py::call_guard<py::gil_scoped_release>(), py::arg("config"), py::arg("sensor_id") = -1)
      .def("AddSensor", &UedsConnector::AddSensor, py::call_guard<py::gil_scoped_release>())
      .def("RemoveSensor", &UedsConnector::RemoveSensor, py::call_guard<py::gil_scoped_release>())
      .def("ListSensors", &UedsConnector::ListSensors, py::call_guard<py::gil_scoped_release>())
      .def("AddDevice", &UedsConnector::AddDevice, py::call_guard<py::gil_scoped_release>(), py::arg("device_name"), py::arg("offset"), py::arg("rotation"), py::arg("show_mesh") = true)
      .def("ListDevices", &UedsConnector::ListDevices, py::call_guard<py::gil_scoped_release>());

  //}
}
