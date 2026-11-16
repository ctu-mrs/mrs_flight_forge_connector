#include <cmath>

#include "actions_common.h"
#include "viz.h"

using namespace ueds_connector;
using StereoConfig = Serializable::Drone::StereoCameraConfig;

namespace
{

const Target DR = Target::Drone;

/* config row builders //{ */

using Rows = std::vector<FieldRow>;

template <typename T>
void IntrinsicsRows(Rows& r, T& i) {
  r.push_back(FieldRow::Section("intrinsics"));
  r.push_back(Row("Use custom K", {Field::B(&i.use_custom_)}, "off = derived from fov"));
  r.push_back(Row("fx / fy", {Field::D(&i.fx_, "fx"), Field::D(&i.fy_, "fy")}));
  r.push_back(Row("cx / cy", {Field::D(&i.cx_, "cx"), Field::D(&i.cy_, "cy")}));
}

template <typename T>
void DistortionRows(Rows& r, T& d) {
  r.push_back(FieldRow::Section("distortion (Brown-Conrady)"));
  r.push_back(Row("Enable", {Field::B(&d.enable_)}));
  r.push_back(Row("k1 k2 k3", {Field::D(&d.k1_, "k1"), Field::D(&d.k2_, "k2"), Field::D(&d.k3_, "k3")}));
  r.push_back(Row("p1 p2", {Field::D(&d.p1_, "p1"), Field::D(&d.p2_, "p2")}));
}

template <typename T>
void CompressionRows(Rows& r, T& c) {
  r.push_back(FieldRow::Section("wire compression"));
  r.push_back(Row("Format", {Field::E(&c.format_, {{"PNG", 0}, {"JPEG", 1}})}));
  r.push_back(Row("JPEG quality", {Field::I(&c.quality_)}, "1-100"));
}

Rows RgbRows(RgbCameraConfig& c) {
  Rows r;
  r.push_back(FieldRow::Section("general"));
  r.push_back(Row("Show debug camera", {Field::B(&c.show_debug_camera_)}));
  r.push_back(Row("Offset", Vec3(&c.offset_.x, &c.offset_.y, &c.offset_.z)));
  r.push_back(Row("Orientation", Vec3(&c.orientation_.pitch, &c.orientation_.yaw, &c.orientation_.roll, "pitch", "yaw", "roll")));
  r.push_back(Row("FOV [deg]", {Field::D(&c.fov_)}));
  r.push_back(Row("Resolution", {Field::I(&c.width_, "w"), Field::I(&c.height_, "h")}));
  r.push_back(Row("Temporal AA", {Field::B(&c.enable_temporal_aa_)}));
  r.push_back(Row("Raytracing", {Field::B(&c.enable_raytracing_)}));
  r.push_back(Row("HDR", {Field::B(&c.enable_hdr_)}));
  r.push_back(Row("Motion blur", {Field::B(&c.enable_motion_blur_)}));
  r.push_back(Row("Blur amount / distortion", {Field::D(&c.motion_blur_amount_, "amt"), Field::D(&c.motion_blur_distortion_, "dist")}));
  r.push_back(FieldRow::Section("exposure"));
  r.push_back(Row("Manual", {Field::B(&c.exposure_.manual_)}));
  r.push_back(Row("Shutter [s] / ISO", {Field::D(&c.exposure_.shutter_time_, "t"), Field::D(&c.exposure_.iso_, "iso")}));
  r.push_back(Row("EV compensation", {Field::D(&c.exposure_.ev_compensation_)}));
  r.push_back(FieldRow::Section("lens"));
  r.push_back(Row("f-stop / focus [m]", {Field::D(&c.lens_.fstop_, "f"), Field::D(&c.lens_.focal_distance_, "focus")}, "focus ≤0 = no DoF"));
  r.push_back(Row("Sensor width [mm]", {Field::D(&c.lens_.sensor_width_mm_)}));
  r.push_back(Row("Vignette / chroma", {Field::D(&c.lens_.vignette_intensity_, "vig"), Field::D(&c.lens_.chromatic_aberration_, "ca")}));
  r.push_back(Row("Bloom / flare", {Field::D(&c.lens_.bloom_intensity_, "bloom"), Field::D(&c.lens_.lens_flare_intensity_, "flare")}));
  r.push_back(Row("White temp / tint", {Field::D(&c.lens_.white_temp_, "K"), Field::D(&c.lens_.white_tint_, "tint")}));
  r.push_back(Row("Blur from shutter", {Field::B(&c.lens_.motion_blur_from_shutter_)}));
  IntrinsicsRows(r, c.intrinsics_);
  DistortionRows(r, c.distortion_);
  r.push_back(FieldRow::Section("noise"));
  r.push_back(Row("Enable", {Field::B(&c.noise_.enable_)}));
  r.push_back(Row("Shot / read / row", {Field::D(&c.noise_.shot_scale_, "shot"), Field::D(&c.noise_.read_sigma_, "read"), Field::D(&c.noise_.row_sigma_, "row")}));
  r.push_back(FieldRow::Section("rolling shutter"));
  r.push_back(Row("Enable", {Field::B(&c.rolling_shutter_.enable_)}));
  r.push_back(Row("Readout [s]", {Field::D(&c.rolling_shutter_.readout_time_)}));
  CompressionRows(r, c.compression_);
  return r;
}

Rows StereoRows(StereoConfig& c) {
  Rows r;
  r.push_back(FieldRow::Section("general"));
  r.push_back(Row("Show debug camera", {Field::B(&c.show_debug_camera_)}));
  r.push_back(Row("Left offset", Vec3(&c.offset_x_left_, &c.offset_y_left_, &c.offset_z_left_)));
  r.push_back(Row("Right offset", Vec3(&c.offset_x_right_, &c.offset_y_right_, &c.offset_z_right_)));
  r.push_back(Row("Left rotation", Vec3(&c.rotation_pitch_left_, &c.rotation_yaw_left_, &c.rotation_roll_left_, "pitch", "yaw", "roll")));
  r.push_back(Row("Right rotation", Vec3(&c.rotation_pitch_right_, &c.rotation_yaw_right_, &c.rotation_roll_right_, "pitch", "yaw", "roll")));
  r.push_back(Row("FOV [deg]", {Field::D(&c.fov_)}));
  r.push_back(Row("Resolution", {Field::I(&c.width_, "w"), Field::I(&c.height_, "h")}));
  r.push_back(Row("Temporal AA", {Field::B(&c.enable_temporal_aa_)}));
  r.push_back(Row("Raytracing", {Field::B(&c.enable_raytracing_)}));
  r.push_back(Row("HDR", {Field::B(&c.enable_hdr_)}));
  IntrinsicsRows(r, c.intrinsics_);
  DistortionRows(r, c.distortion_);
  CompressionRows(r, c.compression_);
  return r;
}

Rows FisheyeRows(FisheyeCameraConfig& c) {
  Rows r;
  r.push_back(FieldRow::Section("general"));
  r.push_back(Row("Show debug camera", {Field::B(&c.show_debug_camera_)}));
  r.push_back(Row("Offset", Vec3(&c.offset_.x, &c.offset_.y, &c.offset_.z)));
  r.push_back(Row("Orientation", Vec3(&c.orientation_.pitch, &c.orientation_.yaw, &c.orientation_.roll, "pitch", "yaw", "roll")));
  r.push_back(Row("FOV [deg]", {Field::D(&c.fov_)}, "up to 220"));
  r.push_back(Row("Resolution", {Field::I(&c.width_, "w"), Field::I(&c.height_, "h")}));
  r.push_back(Row("Lens model", {Field::E(&c.lens_model_, {{"equidistant", FISHEYE_EQUIDISTANT},
                                                           {"equisolid", FISHEYE_EQUISOLID},
                                                           {"stereographic", FISHEYE_STEREOGRAPHIC},
                                                           {"kannala-brandt", FISHEYE_KANNALA_BRANDT}})}));
  r.push_back(Row("Full frame", {Field::B(&c.full_frame_)}, "circle fitted to the corner"));
  IntrinsicsRows(r, c.intrinsics_);
  r.push_back(FieldRow::Section("Kannala-Brandt"));
  r.push_back(Row("k1 k2", {Field::D(&c.kb_k1_, "k1"), Field::D(&c.kb_k2_, "k2")}));
  r.push_back(Row("k3 k4", {Field::D(&c.kb_k3_, "k3"), Field::D(&c.kb_k4_, "k4")}));
  return r;
}

Rows EventRows(EventCameraConfig& c) {
  Rows r;
  r.push_back(FieldRow::Section("optics"));
  r.push_back(Row("Show debug camera", {Field::B(&c.show_debug_camera_)}));
  r.push_back(Row("Offset", Vec3(&c.offset_.x, &c.offset_.y, &c.offset_.z)));
  r.push_back(Row("Orientation", Vec3(&c.orientation_.pitch, &c.orientation_.yaw, &c.orientation_.roll, "pitch", "yaw", "roll")));
  r.push_back(Row("FOV [deg]", {Field::D(&c.fov_)}));
  r.push_back(Row("Resolution", {Field::I(&c.width_, "w"), Field::I(&c.height_, "h")}));
  r.push_back(FieldRow::Section("pixel model"));
  r.push_back(Row("Contrast threshold", {Field::D(&c.contrast_threshold_pos_, "+"), Field::D(&c.contrast_threshold_neg_, "-")}));
  r.push_back(Row("Threshold sigma", {Field::D(&c.threshold_sigma_)}));
  r.push_back(Row("Refractory [s]", {Field::D(&c.refractory_period_)}));
  r.push_back(Row("Cutoff [Hz]", {Field::D(&c.cutoff_hz_)}));
  r.push_back(Row("Leak / shot noise [Hz]", {Field::D(&c.leak_rate_hz_, "leak"), Field::D(&c.shot_noise_rate_hz_, "shot")}));
  r.push_back(Row("Hot pixel fraction", {Field::D(&c.hot_pixel_fraction_)}));
  r.push_back(Row("Max event rate [Hz]", {Field::D(&c.max_event_rate_hz_)}, "0 = unlimited"));
  r.push_back(Row("Max events / frame", {Field::I(&c.max_events_per_frame_, "", 10)}));
  r.push_back(Row("Use sim time", {Field::B(&c.use_sim_time_)}));
  r.push_back(Row("Use GPU", {Field::B(&c.use_gpu_)}));
  r.push_back(Row("Seed", {Field::U(&c.seed_)}));
  return r;
}

Rows LidarRows(LidarConfig& c) {
  Rows r;
  r.push_back(Row("Enable", {Field::B(&c.Enable)}));
  r.push_back(Row("Livox", {Field::B(&c.Livox)}));
  r.push_back(Row("Livox pattern", {Field::S(&c.LivoxSensor, "", 16)}, "avia | mid360"));
  r.push_back(Row("Show beams", {Field::B(&c.showBeams)}));
  r.push_back(Row("Show sensor body", {Field::B(&c.showSensorBody)}));
  r.push_back(Row("Beam length", {Field::D(&c.beamLength)}));
  r.push_back(Row("Rays hor / vert", {Field::D(&c.BeamHorRays, "h"), Field::D(&c.BeamVertRays, "v")}));
  r.push_back(Row("Frequency [Hz]", {Field::D(&c.Frequency)}));
  r.push_back(Row("Offset", Vec3(&c.offset.x, &c.offset.y, &c.offset.z)));
  r.push_back(Row("Orientation", Vec3(&c.orientation.pitch, &c.orientation.yaw, &c.orientation.roll, "pitch", "yaw", "roll")));
  r.push_back(Row("FOV hor left / right", {Field::D(&c.FOVHorLeft, "L"), Field::D(&c.FOVHorRight, "R")}));
  r.push_back(Row("FOV vert up / down", {Field::D(&c.FOVVertUp, "U"), Field::D(&c.FOVVertDown, "D")}));
  return r;
}

//}

struct SensorId
{
  int id = -1;
};

FieldRow SensorIdRow(int* id) {
  return Row("Sensor id", {Field::I(id)}, "-1 = default sensor");
}

// a Get<X>Config + Set<X>Config pair sharing one config struct and sensor id
template <typename T>
void ConfigPair(ActionList& list, const std::string& category, const std::string& name, std::function<std::pair<bool, T>(UedsConnector*, int)> get,
                std::function<bool(UedsConnector*, const T&, int)> set, std::function<Rows(T&)> rows_of) {
  struct P
  {
    T   cfg{};
    int id = -1;
  };
  auto p        = std::make_shared<P>();
  auto describe = std::make_shared<Rows>(rows_of(p->cfg));

  auto fetch = [p, describe, get](Ctx& c) {
    auto [ok, cfg] = get(c.drone, p->id);
    if (!ok) {
      return false;
    }
    p->cfg = cfg;
    for (auto& line : DescribeRows(*describe)) {
      c.line(line);
    }
    return true;
  };

  {
    auto& a = list.emplace_back(MakeAction(category, "Get" + name + "Config", DR, "Read the sensor's configuration (also fills the Set form).", fetch));
    Bind(a, p, {SensorIdRow(&p->id)});
    a.params = std::make_shared<std::pair<std::shared_ptr<P>, std::shared_ptr<Rows>>>(p, describe);
  }
  {
    auto& a = list.emplace_back(MakeAction(category, "Set" + name + "Config", DR, "Apply the configuration. Load current first to edit the live values.",
                                           [p, set](Ctx& c) { return set(c.drone, p->cfg, p->id); }));
    a.load    = fetch;
    Rows rows = {SensorIdRow(&p->id)};
    for (auto& row : rows_of(p->cfg)) {
      rows.push_back(std::move(row));
    }
    Bind(a, p, std::move(rows));
  }
}

// one data-fetching action with a sensor id field, eligible for live mode
Action& DataAction(ActionList& list, const std::string& category, const std::string& name, const std::string& help,
                   std::function<bool(Ctx&, int)> run) {
  auto  p = std::make_shared<SensorId>();
  auto& a = list.emplace_back(MakeAction(category, name, DR, help, [p, run](Ctx& c) { return run(c, p->id); }));
  a.sensor = true;
  Bind(a, p, {SensorIdRow(&p->id)});
  return a;
}

std::string StampStr(double stamp) {
  return "stamp " + Num(stamp, 4);
}

bool ShowImage(Ctx& c, const std::vector<unsigned char>& bytes, double stamp, const std::string& title) {
  std::string error;
  auto        image = DecodeImage(bytes, error);
  if (!image) {
    c.line(error);
    return false;
  }
  image->title = title;
  c.line(std::to_string(image->w) + "×" + std::to_string(image->h) + ", " + std::to_string(bytes.size() / 1024) + " KiB encoded, " + StampStr(stamp));
  c.out.preview.images.push_back(image);
  return true;
}

void DescribeCloud(Ctx& c, const LidarCloud& cloud, size_t raw_count, const Coordinates& start, double stamp) {
  float lo = 1e30f, hi = 0;
  for (const auto& p : cloud.points) {
    lo = std::min(lo, p.distance);
    hi = std::max(hi, p.distance);
  }
  c.line(std::to_string(raw_count) + " beams, " + std::to_string(cloud.points.size()) + " returns, " + StampStr(stamp));
  c.line("start " + Fmt(start));
  if (!cloud.points.empty()) {
    c.line("distance min " + Num(lo) + "  max " + Num(hi) + "  p95 " + Num(cloud.auto_range));
  }
}

}  // namespace

ActionList SensorActions(App& app) {
  ActionList list;

  /* Management //{ */

  list.push_back(MakeAction("Management", "ListSensors", DR, "Every sensor on the active drone.", [](Ctx& c) {
    auto [ok, sensors] = c.drone->ListSensors();
    if (!ok) {
      return false;
    }
    c.line(std::to_string(sensors.size()) + " sensor(s)");
    for (const auto& s : sensors) {
      c.line("  id " + std::to_string(s.id) + "  " + SensorTypeName(s.type));
    }
    return true;
  }));

  {
    struct P
    {
      int type = SENSOR_RGB_CAMERA;
    };
    auto  p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Management", "AddSensor", DR, "Create another sensor; use the returned id in the sensor forms.", [p](Ctx& c) {
      auto [ok, id] = c.drone->AddSensor(p->type);
      c.line(SensorTypeName(p->type) + " → id " + std::to_string(id));
      return ok;
    }));
    Bind(a, p, {Row("Type", {Field::E(&p->type, SensorTypeOptions())})});
  }

  {
    auto  p = std::make_shared<SensorId>();
    auto& a = list.emplace_back(MakeAction("Management", "RemoveSensor", DR, "Remove a sensor by id.", [p](Ctx& c) { return c.drone->RemoveSensor(p->id); }));
    Bind(a, p, {Row("Sensor id", {Field::I(&p->id)})});
  }

  list.push_back(MakeAction("Management", "ListDevices", DR, "Preconfigured real-world devices AddDevice can mount.", [](Ctx& c) {
    auto [ok, devices] = c.drone->ListDevices();
    if (!ok) {
      return false;
    }
    std::vector<std::string> names;
    c.line(std::to_string(devices.size()) + " device(s)");
    for (const auto& d : devices) {
      c.line("  " + d.name + "  " + d.description);
      names.push_back(d.name);
    }
    std::lock_guard<std::mutex> lock(c.app.mutex);
    c.app.device_names = names;
    return true;
  }));

  {
    struct P
    {
      std::string name = "realsense_d435i";
      double      x = 0, y = 0, z = 0, pitch = 0, yaw = 0, roll = 0;
      bool        show_mesh = true;
    };
    auto  p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Management", "AddDevice", DR, "Mount a device (may create several sensors). Names come from ListDevices.",
                                           [p](Ctx& c) {
                                             auto [ok, sensors] = c.drone->AddDevice(p->name, Coordinates(p->x, p->y, p->z), Rotation(p->pitch, p->yaw, p->roll),
                                                                                     p->show_mesh);
                                             if (!ok) {
                                               return false;
                                             }
                                             c.line(p->name + " created " + std::to_string(sensors.size()) + " sensor(s)");
                                             for (const auto& s : sensors) {
                                               c.line("  id " + std::to_string(s.id) + "  " + SensorTypeName(s.type));
                                             }
                                             return true;
                                           }));
    Bind(a, p,
         {Row("Device", {Field::S(&p->name, "", 30)}), Row("Offset", Vec3(&p->x, &p->y, &p->z)),
          Row("Rotation", Vec3(&p->pitch, &p->yaw, &p->roll, "pitch", "yaw", "roll")), Row("Show mesh", {Field::B(&p->show_mesh)})});
  }

  {
    auto  p = std::make_shared<SensorId>();
    auto& a = list.emplace_back(MakeAction("Management", "GetCameraInfo", DR, "Effective calibration of every camera (or one id): K, D, mount, baseline.",
                                           [p](Ctx& c) {
                                             auto [ok, infos] = c.drone->GetCameraInfo(p->id);
                                             if (!ok) {
                                               return false;
                                             }
                                             c.line(std::to_string(infos.size()) + " camera eye(s)");
                                             for (const auto& i : infos) {
                                               c.line("── id " + std::to_string(i.sensor_id_) + "  " + SensorTypeName(i.sensor_type_) +
                                                      (i.sensor_type_ == SENSOR_STEREO_CAMERA ? (i.eye_ ? "  right" : "  left") : ""));
                                               c.line("  " + std::to_string(i.width_) + "×" + std::to_string(i.height_) + "  " + i.projection_ + " / " +
                                                      i.distortion_model_ + "  fov " + Num(i.fov_deg_, 1));
                                               c.line("  fx " + Num(i.fx_) + " fy " + Num(i.fy_) + " cx " + Num(i.cx_) + " cy " + Num(i.cy_));
                                               std::string d = "  D";
                                               for (double v : i.d_) {
                                                 d += " " + Num(v, 5);
                                               }
                                               c.line(d);
                                               c.line("  offset (" + Num(i.offset_x_) + ", " + Num(i.offset_y_) + ", " + Num(i.offset_z_) + ")  rot (" +
                                                      Num(i.rotation_pitch_) + ", " + Num(i.rotation_yaw_) + ", " + Num(i.rotation_roll_) + ")  baseline " +
                                                      Num(i.baseline_) + "  readout " + Num(i.readout_time_, 4));
                                             }
                                             return true;
                                           }));
    Bind(a, p, {Row("Sensor id", {Field::I(&p->id)}, "-1 = all cameras")});
  }

  //}

  /* RGB //{ */

  DataAction(list, "RGB camera", "GetRgbCameraData", "One frame from the RGB camera.", [](Ctx& c, int id) {
    auto [ok, data, stamp, size] = c.drone->GetRgbCameraData(id);
    return ok && ShowImage(c, data, stamp, "rgb");
  });
  ConfigPair<RgbCameraConfig>(
      list, "RGB camera", "RgbCamera", [](UedsConnector* d, int id) { return d->GetRgbCameraConfig(id); },
      [](UedsConnector* d, const RgbCameraConfig& cfg, int id) { return d->SetRgbCameraConfig(cfg, id); }, RgbRows);

  //}

  /* segmentation //{ */

  DataAction(list, "Segmentation", "GetRgbSegmented", "Class mask (raw BGR, sized by the RGB camera config).", [](Ctx& c, int id) {
    auto [ok, data, stamp, size] = c.drone->GetRgbSegmented(id);
    if (!ok) {
      return false;
    }
    auto [cfg_ok, cfg] = c.drone->GetRgbCameraConfig(id);
    if (!cfg_ok) {
      c.line("GetRgbCameraConfig failed (needed for the frame size)");
      return false;
    }
    const size_t px = static_cast<size_t>(cfg.width_) * cfg.height_;
    if (px == 0 || data.size() < px * 3) {
      c.line("frame is " + std::to_string(data.size()) + " bytes, expected " + std::to_string(px * 3) + " for " + std::to_string(cfg.width_) + "×" +
             std::to_string(cfg.height_));
      return false;
    }
    std::shared_ptr<Image> image;
    if (data.size() >= px * 4) {
      // BGRA: drop alpha
      std::vector<unsigned char> bgr(px * 3);
      for (size_t i = 0; i < px; ++i) {
        bgr[i * 3]     = data[i * 4];
        bgr[i * 3 + 1] = data[i * 4 + 1];
        bgr[i * 3 + 2] = data[i * 4 + 2];
      }
      image = FromBgr(bgr, cfg.width_, cfg.height_);
    } else {
      image = FromBgr(data, cfg.width_, cfg.height_);
    }
    image->title = "segmentation";
    c.line(std::to_string(cfg.width_) + "×" + std::to_string(cfg.height_) + ", " + std::to_string(data.size()) + " bytes, " + StampStr(stamp));
    c.out.preview.images.push_back(image);
    return true;
  });

  DataAction(list, "Segmentation", "GetInstanceSegData", "Per-actor instance ids (false colour).", [](Ctx& c, int id) {
    std::vector<unsigned char> data;
    double                     stamp = 0;
    if (!c.drone->GetInstanceSegData(data, stamp, id)) {
      return false;
    }
    std::string error;
    size_t      count = 0;
    auto        image = DecodeInstanceImage(data, error, &count);
    if (!image) {
      c.line(error);
      return false;
    }
    image->title = "instances";
    c.line(std::to_string(image->w) + "×" + std::to_string(image->h) + ", " + std::to_string(count) + " instance(s) visible, " + StampStr(stamp));
    c.out.preview.images.push_back(image);
    return true;
  });

  {
    auto  p = std::make_shared<SensorId>();
    auto& a = list.emplace_back(MakeAction("Segmentation", "GetInstanceSegMap", DR, "id → actor map of the last instance frame.", [p](Ctx& c) {
      auto [ok, entries] = c.drone->GetInstanceSegMap(p->id);
      if (!ok) {
        return false;
      }
      c.line(std::to_string(entries.size()) + " instance(s)");
      for (const auto& e : entries) {
        c.line("  " + std::to_string(e.id) + "  class " + std::to_string(e.class_id) + "  " + e.actor);
      }
      return true;
    }));
    Bind(a, p, {SensorIdRow(&p->id)});
  }

  //}

  /* stereo //{ */

  DataAction(list, "Stereo camera", "GetStereoCameraData", "Left and right frames.", [](Ctx& c, int id) {
    auto [ok, left, right, stamp] = c.drone->GetStereoCameraData(id);
    return ok && ShowImage(c, left, stamp, "left") && ShowImage(c, right, stamp, "right");
  });
  ConfigPair<StereoConfig>(
      list, "Stereo camera", "StereoCamera", [](UedsConnector* d, int id) { return d->GetStereoCameraConfig(id); },
      [](UedsConnector* d, const StereoConfig& cfg, int id) { return d->SetStereoCameraConfig(cfg, id); }, StereoRows);

  //}

  /* fisheye //{ */

  DataAction(list, "Fisheye camera", "GetFisheyeCameraData", "One fisheye frame.", [](Ctx& c, int id) {
    auto [ok, data, stamp, size] = c.drone->GetFisheyeCameraData(id);
    return ok && ShowImage(c, data, stamp, "fisheye");
  });
  ConfigPair<FisheyeCameraConfig>(
      list, "Fisheye camera", "FisheyeCamera", [](UedsConnector* d, int id) { return d->GetFisheyeCameraConfig(id); },
      [](UedsConnector* d, const FisheyeCameraConfig& cfg, int id) { return d->SetFisheyeCameraConfig(cfg, id); }, FisheyeRows);

  //}

  /* depth //{ */

  DataAction(list, "Depth camera", "GetDepthCameraData", "uint16 mm depth, turbo-coloured (red near).", [](Ctx& c, int id) {
    std::vector<uint16_t> depth;
    int                   w = 0, h = 0;
    double                stamp = 0;
    if (!c.drone->GetDepthCameraData(depth, w, h, stamp, id)) {
      return false;
    }
    if (w <= 0 || h <= 0 || depth.size() < static_cast<size_t>(w) * h) {
      c.line("bad depth frame " + std::to_string(w) + "×" + std::to_string(h) + " with " + std::to_string(depth.size()) + " values");
      return false;
    }
    double near_m = 0, far_m = 0;
    auto   image  = FromDepth(depth, w, h, near_m, far_m);
    image->title  = "depth";
    const uint16_t centre = depth[static_cast<size_t>(h / 2) * w + w / 2];
    c.line(std::to_string(w) + "×" + std::to_string(h) + ", " + StampStr(stamp));
    c.line("colour range " + Num(near_m) + " – " + Num(far_m) + " m, centre " + (centre == 0 ? "invalid" : Num(centre / 1000.0, 3) + " m"));
    c.out.preview.images.push_back(image);
    return true;
  });

  //}

  /* event //{ */

  DataAction(list, "Event camera", "GetEventCameraData", "Events since the previous call, accumulated (red brighter, blue darker).", [](Ctx& c, int id) {
    auto [ok, events] = c.drone->GetEventCameraData(id);
    if (!ok) {
      return false;
    }
    int w = events.width, h = events.height;
    if (w <= 0 || h <= 0) {
      auto [cfg_ok, cfg] = c.drone->GetEventCameraConfig(id);
      w                  = cfg_ok ? cfg.width_ : 640;
      h                  = cfg_ok ? cfg.height_ : 480;
    }
    auto image   = FromEvents(events, w, h);
    image->title = "events";
    c.line(std::to_string(events.size()) + " event(s), dropped " + std::to_string(events.dropped) + ", base " + Num(events.base_stamp, 4) + " → " +
           StampStr(events.stamp));
    c.out.preview.images.push_back(image);
    return true;
  });
  ConfigPair<EventCameraConfig>(
      list, "Event camera", "EventCamera", [](UedsConnector* d, int id) { return d->GetEventCameraConfig(id); },
      [](UedsConnector* d, const EventCameraConfig& cfg, int id) { return d->SetEventCameraConfig(cfg, id); }, EventRows);

  //}

  /* rangefinder //{ */

  DataAction(list, "Rangefinder", "GetRangefinderData", "Single-beam range.", [](Ctx& c, int id) {
    auto [ok, range] = c.drone->GetRangefinderData(id);
    if (!ok) {
      return false;
    }
    c.line(Num(range / 100.0, 3) + " m  (" + Num(range, 1) + " raw)");
    c.out.preview.range = range;
    return true;
  });

  //}

  /* lidar //{ */

  DataAction(list, "Lidar", "GetLidarData", "One scan; braille views ('v' cycles top/side/range, +/- zoom).", [](Ctx& c, int id) {
    auto [ok, data, start, stamp] = c.drone->GetLidarData(id);
    if (!ok) {
      return false;
    }
    auto cloud = MakeCloud(data, {}, "lidar");
    DescribeCloud(c, *cloud, data.size(), start, stamp);
    c.out.preview.lidar = cloud;
    return true;
  });

  DataAction(list, "Lidar", "GetLidarSegData", "Scan coloured by segmentation class.", [](Ctx& c, int id) {
    auto [ok, data, start, stamp] = c.drone->GetLidarSegData(id);
    if (!ok) {
      return false;
    }
    std::vector<LidarData> points(data.size());
    std::vector<int>       labels(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      points[i].distance   = data[i].distance;
      points[i].directionX = data[i].directionX;
      points[i].directionY = data[i].directionY;
      points[i].directionZ = data[i].directionZ;
      labels[i]            = data[i].segmentation;
    }
    auto cloud = MakeCloud(points, labels, "lidar seg");
    DescribeCloud(c, *cloud, data.size(), start, stamp);
    c.out.preview.lidar = cloud;
    return true;
  });

  DataAction(list, "Lidar", "GetLidarIntData", "Scan coloured by intensity (material class).", [](Ctx& c, int id) {
    auto [ok, data, start, stamp] = c.drone->GetLidarIntData(id);
    if (!ok) {
      return false;
    }
    std::vector<LidarData> points(data.size());
    std::vector<int>       labels(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      points[i].distance   = data[i].distance;
      points[i].directionX = data[i].directionX;
      points[i].directionY = data[i].directionY;
      points[i].directionZ = data[i].directionZ;
      labels[i]            = data[i].intensity;
    }
    auto cloud = MakeCloud(points, labels, "lidar intensity");
    DescribeCloud(c, *cloud, data.size(), start, stamp);
    c.out.preview.lidar = cloud;
    return true;
  });

  {
    struct P
    {
      int  id        = -1;
      bool labels    = true;
      bool intensity = false;
    };
    auto  p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Lidar", "GetLidarScan", DR, "One trace with the channels asked for; coloured by class when labels are on.",
                                           [p](Ctx& c) {
                                             auto [ok, scan] = c.drone->GetLidarScan(p->labels, p->intensity, p->id);
                                             if (!ok) {
                                               return false;
                                             }
                                             const std::vector<int>& labels = !scan.class_id.empty() ? scan.class_id : scan.intensity;
                                             auto cloud = MakeCloud(scan.points, labels, "lidar scan");
                                             DescribeCloud(c, *cloud, scan.points.size(), scan.start, scan.stamp);
                                             c.line("channels: class " + std::to_string(scan.class_id.size()) + ", instance " +
                                                    std::to_string(scan.instance_id.size()) + ", intensity " + std::to_string(scan.intensity.size()));
                                             c.out.preview.lidar = cloud;
                                             return true;
                                           }));
    a.sensor = true;
    Bind(a, p, {SensorIdRow(&p->id), Row("With labels", {Field::B(&p->labels)}), Row("With intensity", {Field::B(&p->intensity)})});
  }

  ConfigPair<LidarConfig>(
      list, "Lidar", "Lidar", [](UedsConnector* d, int id) { return d->GetLidarConfig(id); },
      [](UedsConnector* d, const LidarConfig& cfg, int id) { return d->SetLidarConfig(cfg, id); }, LidarRows);

  //}

  (void)app;
  return list;
}
