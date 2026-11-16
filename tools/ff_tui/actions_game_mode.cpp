#include <set>

#include "actions_common.h"

using namespace ueds_connector;

ActionList GameModeActions(App& app) {
  ActionList list;
  const Target GM = Target::GameMode;

  /* Session //{ */

  list.push_back(MakeAction("Session", "Ping", GM, "Round-trip check on the game-mode socket.", [](Ctx& c) {
    const bool ok = c.gm->Ping();
    c.line(ok ? "pong" : "no answer");
    return ok;
  }));

  list.push_back(MakeAction("Session", "GetApiVersion", GM, "Simulator API version vs. the one this connector was built with.", [](Ctx& c) {
    auto [ok, v] = c.gm->GetApiVersion();
    if (!ok) {
      return false;
    }
    const bool match = v.first == API_VERSION_MAJOR && v.second == API_VERSION_MINOR;
    c.line("simulator v" + std::to_string(v.first) + "." + std::to_string(v.second) + ", connector v" + std::to_string(API_VERSION_MAJOR) + "." +
           std::to_string(API_VERSION_MINOR) + (match ? "  ✓ match" : "  ✗ MISMATCH"));
    return true;
  }));

  list.push_back(MakeAction("Session", "GetMeta", GM, "The server's op catalogue.", [](Ctx& c) { return DescribeMeta(c, c.gm); }));

  list.push_back(MakeAction("Session", "CheckCompatibility", GM, "Ops this connector knows that the server lacks or serializes differently.",
                            [](Ctx& c) { return DescribeCompat(c, c.gm); }));

  {
    struct P
    {
      int ms = 10000;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Session", "SetReceiveTimeoutMs", GM, "Client-side reply timeout of the game-mode socket (a long Step needs more).",
                                           [p](Ctx& c) {
                                             c.gm->SetReceiveTimeoutMs(p->ms);
                                             c.line("timeout " + std::to_string(c.gm->GetReceiveTimeoutMs()) + " ms");
                                             return true;
                                           }));
    a.load = [p](Ctx& c) {
      p->ms = c.gm->GetReceiveTimeoutMs();
      c.line("timeout " + std::to_string(p->ms) + " ms");
      return true;
    };
    Bind(a, p, {Row("Timeout [ms]", {Field::I(&p->ms)})});
  }

  list.push_back(MakeAction("Session", "GetFps", GM, "Simulator frame rate.", [](Ctx& c) {
    auto [ok, fps] = c.gm->GetFps();
    c.line("FPS " + Num(fps, 1));
    return ok;
  }));

  list.push_back(MakeAction("Session", "GetTime", GM, "Simulator wall clock.", [](Ctx& c) {
    auto [ok, t] = c.gm->GetTime();
    c.line("time " + Num(t, 3) + " s");
    return ok;
  }));

  list.push_back(MakeAction("Session", "GetSimTime", GM, "Simulated seconds since level start and the time dilation.", [](Ctx& c) {
    double t = 0, dilation = 0;
    const bool ok = c.gm->GetSimTime(t, dilation);
    c.line("sim time " + Num(t, 3) + " s, dilation " + Num(dilation, 3));
    return ok;
  }));

  {
    struct P
    {
      double dilation = 1.0;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Session", "SetTimeDilation", GM, "Simulated seconds per wall second; the engine clamps it.", [p](Ctx& c) {
      double applied = 0;
      const bool ok  = c.gm->SetTimeDilation(p->dilation, applied);
      c.line("applied dilation " + Num(applied, 3));
      return ok;
    }));
    Bind(a, p, {Row("Dilation", {Field::D(&p->dilation)}, "<1 slow motion, >1 fast")});
  }

  {
    struct P
    {
      double dt = 0.0, adaptive_min_dt = 0.0;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Session", "SetFixedTimestep", GM, "Every tick advances sim time by exactly dt; 0 restores wall-clock steps.",
                                           [p](Ctx& c) {
                                             double applied = 0;
                                             const bool ok  = c.gm->SetFixedTimestep(p->dt, p->adaptive_min_dt, applied);
                                             c.line("applied dt " + Num(applied, 6) + " s");
                                             return ok;
                                           }));
    Bind(a, p, {Row("dt [s]", {Field::D(&p->dt)}, "0 = off"), Row("Adaptive min dt [s]", {Field::D(&p->adaptive_min_dt)}, "event-camera anti-aliasing")});
  }

  {
    struct P
    {
      bool enabled = true;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Session", "SetLockstep", GM, "Pause the world; it then advances only through Step.", [p](Ctx& c) {
      const bool ok = c.gm->SetLockstep(p->enabled);
      c.line(std::string("lockstep ") + (p->enabled ? "on" : "off"));
      return ok;
    }));
    Bind(a, p, {Row("Enabled", {Field::B(&p->enabled)})});
  }

  {
    struct P
    {
      int ticks = 1;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Session", "Step", GM, "Run N ticks (lockstep); returns once every camera has its frame.", [p](Ctx& c) {
      double   sim_time = 0;
      uint64_t ticks    = 0;
      const bool ok     = c.gm->Step(p->ticks, sim_time, ticks);
      c.line("sim time " + Num(sim_time, 4) + " s, tick count " + std::to_string(ticks));
      return ok;
    }));
    Bind(a, p, {Row("Ticks", {Field::I(&p->ticks)})});
  }

  const std::vector<std::pair<std::string, int>> capture_modes = {
      {"all frames", CAPTURE_ALL_FRAMES}, {"on movement", CAPTURE_ON_MOVEMENT}, {"on demand", CAPTURE_ON_DEMAND}};

  list.push_back(MakeAction("Session", "GetCameraCaptureMode", GM, "When cameras render.", [capture_modes](Ctx& c) {
    auto [ok, mode] = c.gm->GetCameraCaptureMode();
    for (auto& [name, id] : capture_modes) {
      if (id == mode) {
        c.line("capture mode: " + name);
        return ok;
      }
    }
    c.line("capture mode " + std::to_string(mode));
    return ok;
  }));

  {
    struct P
    {
      int mode = CAPTURE_ALL_FRAMES;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Session", "SetCameraCaptureMode", GM, "When cameras render.", [p](Ctx& c) {
      return c.gm->SetCameraCaptureMode(static_cast<CameraCaptureModeEnum>(p->mode));
    }));
    a.load = [p](Ctx& c) {
      auto [ok, mode] = c.gm->GetCameraCaptureMode();
      p->mode         = mode;
      return ok;
    };
    Bind(a, p, {Row("Mode", {Field::E(&p->mode, capture_modes)})});
  }

  {
    struct P
    {
      int level = 1;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Session", "SetGraphicsSettings", GM, "Engine scalability preset.",
                                           [p](Ctx& c) { return c.gm->SetGraphicsSettings(p->level); }));
    Bind(a, p, {Row("Level", {Field::E(&p->level, SortedById(GraphicsSettings::Name2Id()))})});
  }

  list.push_back(MakeAction("Session", "GetClassMap", GM, "The segmentation class tree of the current world.", [](Ctx& c) {
    auto [ok, map] = c.gm->GetClassMap();
    if (!ok) {
      return false;
    }
    c.line(std::to_string(map.classes_.size()) + " classes, hash " + std::to_string(map.hash_));
    for (const auto& e : map.classes_) {
      c.line("  " + std::to_string(e.id) + "  (parent " + std::to_string(e.parent_id) + ")  " + e.path);
    }
    return true;
  }));

  //}

  /* Drones //{ */

  list.push_back(MakeAction("Drones", "GetDrones", GM, "Ports of every drone in the world.", [](Ctx& c) {
    auto [ok, ports] = c.gm->GetDrones();
    if (!ok) {
      return false;
    }
    std::string s = std::to_string(ports.size()) + " drone(s):";
    for (int port : ports) {
      s += " " + std::to_string(port);
    }
    c.line(s);
    {
      std::lock_guard<std::mutex> lock(c.app.mutex);
      c.app.drone_ports = ports;
    }
    return true;
  }));

  auto adopt_drone = [](Ctx& c, int port) {
    {
      std::lock_guard<std::mutex> lock(c.app.mutex);
      if (std::find(c.app.drone_ports.begin(), c.app.drone_ports.end(), port) == c.app.drone_ports.end()) {
        c.app.drone_ports.push_back(port);
      }
    }
    c.app.SetActiveDrone(port);
  };

  list.push_back(MakeAction("Drones", "SpawnDrone", GM, "Spawn the default frame at the default spot; it becomes the active drone.", [adopt_drone](Ctx& c) {
    auto [ok, port] = c.gm->SpawnDrone();
    if (ok) {
      c.line("port " + std::to_string(port) + " (now active)");
      adopt_drone(c, port);
    }
    return ok;
  }));

  {
    struct P
    {
      std::string frame = "x500";
      std::string custom;
      bool        at_origin = true;
      double      x = 0, y = 0, z = 200;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction(
        "Drones", "SpawnDroneAtLocation", GM, "Spawn a frame by mesh name; with 'at world origin' the location is an offset from it. Units cm.",
        [p, adopt_drone](Ctx& c) {
          Coordinates location(p->x, p->y, p->z);
          if (p->at_origin) {
            auto [origin_ok, origin] = c.gm->GetWorldOrigin();
            if (!origin_ok) {
              c.line("GetWorldOrigin failed");
              return false;
            }
            location = Coordinates(origin.x + p->x, origin.y + p->y, origin.z + p->z);
          }
          std::string frame = p->custom.empty() ? p->frame : p->custom;
          auto [ok, port]   = c.gm->SpawnDroneAtLocation(location, frame);
          if (!ok) {
            c.line("spawn failed: invalid location or frame '" + frame + "'");
            return false;
          }
          c.line(frame + " on port " + std::to_string(port) + " at " + Fmt(location) + " (now active)");
          adopt_drone(c, port);
          return true;
        }));
    Bind(a, p,
         {Row("Frame", {Field::SE(&p->frame, UavFrameType::KnownNames())}), Row("Custom frame", {Field::S(&p->custom, "", 30)}, "overrides Frame (mesh or .glb)"),
          Row("At world origin", {Field::B(&p->at_origin)}), Row("Location / offset", Vec3(&p->x, &p->y, &p->z))});
  }

  {
    struct P
    {
      int port = -1;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Drones", "RemoveDrone", GM, "Remove a drone by port; -1 means the active drone.", [p](Ctx& c) {
      const int port = p->port >= 0 ? p->port : c.app.ActiveDrone();
      if (port < 0) {
        c.line("no port given and no active drone");
        return false;
      }
      const bool ok = c.gm->RemoveDrone(port);
      if (ok) {
        c.line("removed :" + std::to_string(port));
        {
          std::lock_guard<std::mutex> lock(c.app.mutex);
          auto& ports = c.app.drone_ports;
          ports.erase(std::remove(ports.begin(), ports.end(), port), ports.end());
        }
        if (c.app.ActiveDrone() == port) {
          c.app.drone_worker.Post([&app = c.app] { app.DropDroneConnections(); });
          c.app.SetActiveDrone(-1);
        }
      }
      return ok;
    }));
    Bind(a, p, {Row("Port", {Field::I(&p->port)}, "-1 = active drone")});
  }

  //}

  /* World //{ */

  {
    std::set<std::string> levels;
    for (auto& [name, level] : WorldName::Name2Level()) {
      levels.insert(level);
    }
    struct P
    {
      std::string level = "Valley";
      std::string custom;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("World", "SwitchWorldLevel", GM, "Load another level, then reconnect automatically (drones are gone afterwards).",
                                           [p](Ctx& c) {
                                             const std::string level = p->custom.empty() ? p->level : p->custom;
                                             c.gm->SwitchWorldLevel(level);
                                             c.line("switching to " + level + ", reconnecting...");
                                             c.app.SetActiveDrone(-1);
                                             c.app.ReconnectGameMode(60);
                                             bool connected;
                                             {
                                               std::lock_guard<std::mutex> lock(c.app.mutex);
                                               connected = c.app.gm_connected;
                                             }
                                             c.line(connected ? "reconnected" : "could not reconnect within 60 s");
                                             return connected;
                                           }));
    Bind(a, p,
         {Row("Level", {Field::SE(&p->level, std::vector<std::string>(levels.begin(), levels.end()))}),
          Row("Custom level / path", {Field::S(&p->custom, "", 40)}, "overrides Level")});
  }

  list.push_back(MakeAction("World", "GetWorldOrigin", GM, "Origin of the world's scene frame (cm).", [](Ctx& c) {
    auto [ok, origin] = c.gm->GetWorldOrigin();
    c.line("origin " + Fmt(origin));
    return ok;
  }));

  {
    struct P
    {
      int weather = 0;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("World", "SetWeather", GM, "Weather preset.", [p](Ctx& c) { return c.gm->SetWeather(p->weather); }));
    Bind(a, p, {Row("Weather", {Field::E(&p->weather, SortedById(WeatherType::Type2Id()))})});
  }

  {
    std::vector<std::pair<std::string, int>> times;
    for (int m = 0; m < 24 * 60; m += 30) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "%02d:%02d", m / 60, m % 60);
      times.emplace_back(buf, m);
    }
    struct P
    {
      int  time_of_day = 12 * 60;
      int  hour = -1, minute = 0;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("World", "SetDatetime", GM, "Time of day: ←/→ steps 30 min, Enter applies. Exact hour ≥ 0 overrides.",
                                           [p](Ctx& c) {
                                             const int h = p->hour >= 0 ? p->hour : p->time_of_day / 60;
                                             const int m = p->hour >= 0 ? p->minute : p->time_of_day % 60;
                                             char      buf[8];
                                             std::snprintf(buf, sizeof(buf), "%02d:%02d", h, m);
                                             c.line(std::string("time set to ") + buf);
                                             return c.gm->SetDatetime(h, m);
                                           }));
    Bind(a, p, {Row("Time of day", {Field::E(&p->time_of_day, times)}), Row("Exact hour / minute", {Field::I(&p->hour, "h"), Field::I(&p->minute, "m")}, "-1 = off")});
  }

  {
    struct P
    {
      int level = 1;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("World", "SetForestDensity", GM, "Procedural forest density level.",
                                           [p](Ctx& c) { return c.gm->SetForestDensity(p->level); }));
    Bind(a, p, {Row("Density level", {Field::I(&p->level)})});
  }

  {
    struct P
    {
      int level = 1;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("World", "SetForestHillyLevel", GM, "Procedural forest terrain hilliness.",
                                           [p](Ctx& c) { return c.gm->SetForestHillyLevel(p->level); }));
    Bind(a, p, {Row("Hilly level", {Field::I(&p->level)})});
  }

  {
    struct P
    {
      bool enabled = true;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("World", "SetMutualDroneVisibility", GM, "Whether drones see each other in their sensors.",
                                           [p](Ctx& c) { return c.gm->SetMutualDroneVisibility(p->enabled); }));
    Bind(a, p, {Row("Visible", {Field::B(&p->enabled)})});
  }

  //}

  (void)app;
  return list;
}
