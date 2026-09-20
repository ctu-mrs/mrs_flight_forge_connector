#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>

#include <flight_forge_connector/flight_forge_connector.h>

namespace py = pybind11;

using ueds_connector::CameraEvent;
using ueds_connector::CameraEvents;
using ueds_connector::PoseSample;
using ueds_connector::CameraDistortion;
using ueds_connector::CameraNoise;
using ueds_connector::CameraRollingShutter;
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

using StereoCameraConfig  = Serializable::Drone::StereoCameraConfig;
using InstanceSegMapEntry = Serializable::Drone::InstanceSegMapEntry;
using CameraInfo          = Serializable::Drone::CameraInfo;

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

  py::class_<InstanceSegMapEntry>(m, "InstanceSegMapEntry")
      .def(py::init<>())
      .def_readonly("id", &InstanceSegMapEntry::id)
      .def_readonly("actor", &InstanceSegMapEntry::actor);

  py::class_<CameraInfo>(m, "CameraInfo")
      .def(py::init<>())
      .def_readonly("sensor_id", &CameraInfo::sensor_id_)
      .def_readonly("sensor_type", &CameraInfo::sensor_type_)
      .def_readonly("eye", &CameraInfo::eye_)
      .def_readonly("width", &CameraInfo::width_)
      .def_readonly("height", &CameraInfo::height_)
      .def_readonly("model", &CameraInfo::model_)
      .def_readonly("fx", &CameraInfo::fx_)
      .def_readonly("fy", &CameraInfo::fy_)
      .def_readonly("cx", &CameraInfo::cx_)
      .def_readonly("cy", &CameraInfo::cy_)
      .def_readonly("d", &CameraInfo::d_)
      .def_readonly("fov_deg", &CameraInfo::fov_deg_)
      .def_readonly("offset_x", &CameraInfo::offset_x_)
      .def_readonly("offset_y", &CameraInfo::offset_y_)
      .def_readonly("offset_z", &CameraInfo::offset_z_)
      .def_readonly("rotation_pitch", &CameraInfo::rotation_pitch_)
      .def_readonly("rotation_yaw", &CameraInfo::rotation_yaw_)
      .def_readonly("rotation_roll", &CameraInfo::rotation_roll_)
      .def_readonly("baseline", &CameraInfo::baseline_)
      .def_readonly("readout_time", &CameraInfo::readout_time_);

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
      .value("INSTANCE_SEG_CAMERA", ueds_connector::SENSOR_INSTANCE_SEG_CAMERA)
      .value("DEPTH_CAMERA", ueds_connector::SENSOR_DEPTH_CAMERA)
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
      .def_readwrite("showSensorBody", &LidarConfig::showSensorBody)
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

  py::class_<CameraDistortion>(m, "CameraDistortion")
      .def(py::init<>())
      .def_readwrite("enable", &CameraDistortion::enable_)
      .def_readwrite("k1", &CameraDistortion::k1_)
      .def_readwrite("k2", &CameraDistortion::k2_)
      .def_readwrite("k3", &CameraDistortion::k3_)
      .def_readwrite("p1", &CameraDistortion::p1_)
      .def_readwrite("p2", &CameraDistortion::p2_);

  py::class_<CameraNoise>(m, "CameraNoise")
      .def(py::init<>())
      .def_readwrite("enable", &CameraNoise::enable_)
      .def_readwrite("shot_scale", &CameraNoise::shot_scale_)
      .def_readwrite("read_sigma", &CameraNoise::read_sigma_)
      .def_readwrite("row_sigma", &CameraNoise::row_sigma_);

  py::class_<CameraRollingShutter>(m, "CameraRollingShutter")
      .def(py::init<>())
      .def_readwrite("enable", &CameraRollingShutter::enable_)
      .def_readwrite("readout_time", &CameraRollingShutter::readout_time_);

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
      .def_readwrite("intrinsics", &RgbCameraConfig::intrinsics_)
      .def_readwrite("distortion", &RgbCameraConfig::distortion_)
      .def_readwrite("noise", &RgbCameraConfig::noise_)
      .def_readwrite("rolling_shutter", &RgbCameraConfig::rolling_shutter_);

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
      .def_property(
          "distortion",
          [](const StereoCameraConfig& config) {
            CameraDistortion distortion;
            distortion.enable_ = config.distortion_.enable_;
            distortion.k1_     = config.distortion_.k1_;
            distortion.k2_     = config.distortion_.k2_;
            distortion.k3_     = config.distortion_.k3_;
            distortion.p1_     = config.distortion_.p1_;
            distortion.p2_     = config.distortion_.p2_;
            return distortion;
          },
          [](StereoCameraConfig& config, const CameraDistortion& distortion) {
            config.distortion_.enable_ = distortion.enable_;
            config.distortion_.k1_     = distortion.k1_;
            config.distortion_.k2_     = distortion.k2_;
            config.distortion_.k3_     = distortion.k3_;
            config.distortion_.p1_     = distortion.p1_;
            config.distortion_.p2_     = distortion.p2_;
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
      .def_readwrite("use_sim_time", &EventCameraConfig::use_sim_time_)
      .def_readwrite("threshold_sigma", &EventCameraConfig::threshold_sigma_)
      .def_readwrite("refractory_period", &EventCameraConfig::refractory_period_)
      .def_readwrite("cutoff_hz", &EventCameraConfig::cutoff_hz_)
      .def_readwrite("leak_rate_hz", &EventCameraConfig::leak_rate_hz_)
      .def_readwrite("shot_noise_rate_hz", &EventCameraConfig::shot_noise_rate_hz_)
      .def_readwrite("hot_pixel_fraction", &EventCameraConfig::hot_pixel_fraction_)
      .def_readwrite("max_event_rate_hz", &EventCameraConfig::max_event_rate_hz_)
      .def_readwrite("max_events_per_frame", &EventCameraConfig::max_events_per_frame_)
      .def_readwrite("use_gpu", &EventCameraConfig::use_gpu_)
      .def_readwrite("seed", &EventCameraConfig::seed_);

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
      .def_readwrite("lens_model", &FisheyeCameraConfig::lens_model_)
      .def_readwrite("full_frame", &FisheyeCameraConfig::full_frame_);

  //}

  /* connector //{ */

  py::class_<UedsConnector>(m, "DroneController")
      .def(py::init<const std::string&, uint16_t>())
      .def("ConnectSimple", &UedsConnector::ConnectSimple, py::call_guard<py::gil_scoped_release>())
      .def("Ping", &UedsConnector::Ping, py::call_guard<py::gil_scoped_release>())
      .def(
          "GetMeta",
          [](UedsConnector& client) {
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
      .def("CheckCompatibility", &UedsConnector::CheckCompatibility, py::call_guard<py::gil_scoped_release>(), py::arg("op_names") = std::vector<std::string>{})
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
      .def(
          "GetEventCameraData",
          [](UedsConnector& drone, int sensor_id) {
            bool         ok;
            CameraEvents events;
            {
              py::gil_scoped_release release;
              std::tie(ok, events) = drone.GetEventCameraData(sensor_id);
            }
            py::array_t<uint32_t> xyp(static_cast<py::ssize_t>(events.xyp.size()));
            py::array_t<uint32_t> t_us(static_cast<py::ssize_t>(events.t_us.size()));
            std::memcpy(xyp.mutable_data(), events.xyp.data(), events.xyp.size() * sizeof(uint32_t));
            std::memcpy(t_us.mutable_data(), events.t_us.data(), events.t_us.size() * sizeof(uint32_t));
            return py::make_tuple(ok, xyp, t_us, events.base_stamp, events.stamp, events.dropped, events.width, events.height);
          },
          py::arg("sensor_id") = -1)
      .def(
          "SetPropellerSpeeds",
          [](UedsConnector& drone, const std::vector<double>& speeds) {
            int32_t count = 0;
            bool    ok;
            {
              py::gil_scoped_release release;
              ok = drone.SetPropellerSpeeds(speeds, count);
            }
            return py::make_tuple(ok, count);
          },
          py::arg("speeds"))
      .def(
          "SetPoseTrajectory",
          [](UedsConnector& drone, py::array_t<double, py::array::c_style | py::array::forcecast> samples) {
            if (samples.ndim() != 2 || samples.shape(1) != 7) {
              throw std::invalid_argument("SetPoseTrajectory expects an (N, 7) array of t, x, y, z, pitch, yaw, roll");
            }
            std::vector<PoseSample> out(static_cast<size_t>(samples.shape(0)));
            const double*           row = samples.data();
            for (auto& s : out) {
              s.t           = row[0];
              s.position    = Coordinates{row[1], row[2], row[3]};
              s.orientation = Rotation{row[4], row[5], row[6]};
              row += 7;
            }
            py::gil_scoped_release release;
            return drone.SetPoseTrajectory(out);
          },
          py::arg("samples"))
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
      .def("GetCameraInfo", &UedsConnector::GetCameraInfo, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def("AddDevice", &UedsConnector::AddDevice, py::call_guard<py::gil_scoped_release>(), py::arg("device_name"), py::arg("offset"), py::arg("rotation"), py::arg("show_mesh") = true)
      .def("ListDevices", &UedsConnector::ListDevices, py::call_guard<py::gil_scoped_release>())
      .def(
          "GetInstanceSegData",
          [](UedsConnector& connector, int sensor_id) {
            std::vector<unsigned char> image;
            double                     stamp   = 0.0;
            bool                       success = false;
            {
              py::gil_scoped_release release;
              success = connector.GetInstanceSegData(image, stamp, sensor_id);
            }
            return py::make_tuple(success, py::bytes(reinterpret_cast<const char*>(image.data()), image.size()), stamp);
          },
          py::arg("sensor_id") = -1, "Returns (success, png_bytes, stamp).")
      .def("GetInstanceSegMap", &UedsConnector::GetInstanceSegMap, py::call_guard<py::gil_scoped_release>(), py::arg("sensor_id") = -1)
      .def(
          "GetDepthCameraData",
          [](UedsConnector& connector, int sensor_id) {
            std::vector<uint16_t> image;
            int                   width = 0, height = 0;
            double                stamp   = 0.0;
            bool                  success = false;
            {
              py::gil_scoped_release release;
              success = connector.GetDepthCameraData(image, width, height, stamp, sensor_id);
            }
            return py::make_tuple(success, py::bytes(reinterpret_cast<const char*>(image.data()), image.size() * sizeof(uint16_t)), width, height, stamp);
          },
          py::arg("sensor_id") = -1, "Returns (success, uint16le_mm_bytes, width, height, stamp).");

  //}
}
