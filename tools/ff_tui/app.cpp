#include "app.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <thread>

using ueds_connector::GameModeController;
using ueds_connector::UedsConnector;

/* helpers //{ */

std::string Num(double v, int precision) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", precision, v);
  return buf;
}

std::string Fmt(const ueds_connector::Coordinates& c) {
  return "x " + Num(c.x) + "  y " + Num(c.y) + "  z " + Num(c.z);
}

std::string Fmt(const ueds_connector::Rotation& r) {
  return "pitch " + Num(r.pitch) + "  yaw " + Num(r.yaw) + "  roll " + Num(r.roll);
}

std::string OkStr(bool ok) {
  return ok ? "ok" : "failed";
}

//}

App::App(std::string address_, int gm_port_) : address(std::move(address_)), gm_port(gm_port_) {
}

App::~App() {
  Shutdown();
}

void App::Shutdown() {
  // workers first: their jobs reference the connections and the actions
  gm_worker.Stop();
  drone_worker.Stop();
  drones_.clear();
  if (gm_) {
    gm_->Disconnect();
    gm_.reset();
  }
}

void App::Refresh() {
  if (refresh) {
    refresh();
  }
}

void App::Log(bool ok, const std::string& text) {
  const auto  now = std::time(nullptr);
  std::tm     tm{};
  localtime_r(&now, &tm);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm);
  {
    std::lock_guard<std::mutex> lock(mutex);
    log.push_back(LogLine{buf, ok, text});
    while (log.size() > 300) {
      log.pop_front();
    }
  }
  Refresh();
}

void App::SetActiveDrone(int port) {
  active_drone_.store(port);
  {
    std::lock_guard<std::mutex> lock(mutex);
    drive.synced = false;
    drive.status = "not synced";
  }
  Log(true, "active drone :" + std::to_string(port));
}

/* Execute //{ */

void App::Execute(Action& action, bool load, bool from_live) {
  if (action.running->load()) {
    if (!from_live) {
      Log(false, action.name + ": the previous call is still running");
    }
    return;
  }

  if (!from_live) {
    std::string error;
    if (!PushAll(action.rows, error)) {
      Log(false, action.name + ": " + error);
      return;
    }
  }

  action.running->store(true);
  Worker& worker = action.target == Target::GameMode ? gm_worker : drone_worker;
  Action* a      = &action;
  worker.Post([this, a, load, from_live] { RunJob(*a, load, from_live); }, from_live ? "live" : "");
}

void App::RunJob(Action& action, bool load, bool from_live) {
  Result r;
  r.action = action.name + (load ? " (load)" : "");

  const auto  start = std::chrono::steady_clock::now();
  bool        ok    = false;
  std::string error;

  GameModeController* gm    = nullptr;
  UedsConnector*      drone = nullptr;

  if (action.target == Target::GameMode) {
    bool connected;
    {
      std::lock_guard<std::mutex> lock(mutex);
      connected = gm_connected;
    }
    if (connected && gm_) {
      gm = gm_.get();
    } else {
      error = "game mode not connected";
    }
  } else {
    drone = DroneConnection(error);
  }

  if (gm || drone) {
    Ctx ctx{*this, gm, drone, r};
    try {
      ok = load ? action.load(ctx) : action.run(ctx);
    }
    catch (const std::exception& e) {
      error = std::string("exception: ") + e.what();
      ok    = false;
    }
  }

  r.ok = ok;
  r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  if (!error.empty()) {
    r.lines.insert(r.lines.begin(), error);
  } else if (r.lines.empty()) {
    r.lines.push_back(ok ? "ok" : "returned false");
  }

  // a failed call may mean the socket died; find out and drop it so it reconnects
  if (!ok && gm && !gm->Ping()) {
    gm->Disconnect();
    std::lock_guard<std::mutex> lock(mutex);
    gm_connected = false;
    gm_status    = "connection lost (retrying)";
  }
  if (!ok && drone && !drone->Ping()) {
    drones_.erase(ActiveDrone());
    std::lock_guard<std::mutex> lock(mutex);
    drone_status = "connection lost";
  }

  if (!from_live || !ok) {
    std::string summary = r.action;
    if (!r.lines.empty()) {
      summary += " → " + r.lines.front();
    }
    summary += "  (" + Num(r.ms, 0) + " ms)";
    Log(ok, summary);
  }

  {
    std::lock_guard<std::mutex> lock(mutex);
    result = std::move(r);
    ++result_seq;
  }
  if (load) {
    action.needs_pull->store(true);
  }
  action.running->store(false);
  Refresh();
}

//}

/* connections //{ */

UedsConnector* App::DroneConnection(std::string& error) {
  const int port = ActiveDrone();
  if (port < 0) {
    error = "no active drone: spawn one or pick one with 'd'";
    return nullptr;
  }
  auto it = drones_.find(port);
  if (it != drones_.end()) {
    return it->second.get();
  }
  auto connection = std::make_unique<UedsConnector>(address, port);
  if (connection->Connect() != kissnet::socket_status::valid) {
    error = "cannot connect to drone at " + address + ":" + std::to_string(port);
    std::lock_guard<std::mutex> lock(mutex);
    drone_status = "unreachable";
    return nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    drone_status = "connected";
  }
  auto* raw = connection.get();
  drones_[port] = std::move(connection);
  return raw;
}

void App::DropDroneConnections() {
  drones_.clear();
  std::lock_guard<std::mutex> lock(mutex);
  drone_status = "";
}

bool App::ConnectGameMode() {
  if (!gm_) {
    gm_ = std::make_unique<GameModeController>(address, gm_port);
  } else {
    gm_->Disconnect();
  }

  if (gm_->Connect() != kissnet::socket_status::valid) {
    std::lock_guard<std::mutex> lock(mutex);
    gm_connected = false;
    gm_status    = "no simulator (retrying)";
    return false;
  }

  const auto [ok, version] = gm_->GetApiVersion();
  bool api_match           = ok && version.first == API_VERSION_MAJOR && version.second == API_VERSION_MINOR;
  {
    std::lock_guard<std::mutex> lock(mutex);
    gm_connected = true;
    gm_status    = "connected";
    api_major    = ok ? version.first : -1;
    api_minor    = ok ? version.second : -1;
    api_ok       = api_match;
  }
  Log(true, "connected to game mode " + address + ":" + std::to_string(gm_port));
  if (!api_match) {
    Log(false, "API mismatch: connector v" + std::to_string(API_VERSION_MAJOR) + "." + std::to_string(API_VERSION_MINOR) + ", simulator v" +
                   std::to_string(version.first) + "." + std::to_string(version.second) + ". Update the connector or the simulator.");
  }
  return true;
}

void App::ReconnectGameMode(int attempts) {
  if (gm_) {
    gm_->Disconnect();
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    gm_connected = false;
    gm_status    = "reconnecting...";
    drone_ports.clear();
  }
  drone_worker.Post([this] { DropDroneConnections(); });
  Refresh();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  for (int i = 0; i < attempts; ++i) {
    if (ConnectGameMode()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      gm_status = "reconnecting " + std::to_string(i + 1) + "/" + std::to_string(attempts);
    }
    Refresh();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

//}

/* Tick //{ */

void App::Tick(int64_t now_ms) {
  static int64_t last_poll_ms = -100000;  // ticker thread only

  if (now_ms - last_poll_ms >= 1000 && !gm_worker.HasKey("poll")) {
    last_poll_ms = now_ms;
    gm_worker.Post(
        [this, now_ms] {
          bool connected;
          {
            std::lock_guard<std::mutex> lock(mutex);
            connected = gm_connected;
          }

          if (!connected) {
            if (now_ms - last_connect_attempt_ms_ >= 2000) {
              last_connect_attempt_ms_ = now_ms;
              ConnectGameMode();
              Refresh();
            }
            return;
          }

          const auto [fps_ok, fps_value] = gm_->GetFps();
          if (!fps_ok && !gm_->Ping()) {
            gm_->Disconnect();
            {
              std::lock_guard<std::mutex> lock(mutex);
              gm_connected = false;
              gm_status    = "connection lost (retrying)";
            }
            Log(false, "game mode connection lost");
            return;
          }

          std::vector<int> ports;
          bool             have_ports = false;
          if (now_ms - last_drones_poll_ms_ >= 3000) {
            last_drones_poll_ms_ = now_ms;
            auto [drones_ok, list] = gm_->GetDrones();
            have_ports             = drones_ok;
            ports                  = list;
          }

          {
            std::lock_guard<std::mutex> lock(mutex);
            if (fps_ok) {
              fps = fps_value;
            }
            if (have_ports) {
              drone_ports = ports;
            }
          }
          if (have_ports && ActiveDrone() < 0 && !ports.empty()) {
            SetActiveDrone(ports.front());
          }
          Refresh();
        },
        "poll");
  }

  Action* live = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex);
    live = live_action;
  }
  if (live && !live->running->load() && !drone_worker.HasKey("live")) {
    Execute(*live, false, true);
  }
}

//}
