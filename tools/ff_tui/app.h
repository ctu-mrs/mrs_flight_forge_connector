#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <flight_forge_connector/flight_forge_connector.h>
#include <flight_forge_connector/game_mode_controller.h>

#include "forms.h"
#include "worker.h"

/* preview data //{ */

struct Image
{
  int                  w = 0;
  int                  h = 0;
  std::vector<uint8_t> rgb;  // w*h*3
  std::string          title;
};

struct LidarPoint
{
  float x, y, z;  // relative to the sensor, simulator axes and units
  float distance;
  int   label;  // segmentation / intensity, -1 none
};

struct LidarCloud
{
  std::vector<LidarPoint> points;
  bool                    has_label = false;
  float                   auto_range = 1.0f;  // 95th percentile distance, used to scale views
  std::string             title;
};

struct Preview
{
  std::vector<std::shared_ptr<const Image>> images;
  std::shared_ptr<const LidarCloud>         lidar;
  std::optional<double>                     range;      // rangefinder read-out
  double                                    range_max = 0.0;

  bool empty() const {
    return images.empty() && !lidar && !range;
  }
};

//}

struct Result
{
  std::string              action;
  bool                     ok = false;
  double                   ms = 0.0;
  std::vector<std::string> lines;
  Preview                  preview;
};

struct LogLine
{
  std::string time;
  bool        ok;
  std::string text;
};

enum class Target
{
  GameMode,
  Drone,
};

class App;

// what an action's run/load gets on its worker thread
struct Ctx
{
  App&                                 app;
  ueds_connector::GameModeController*  gm    = nullptr;
  ueds_connector::UedsConnector*       drone = nullptr;
  Result&                              out;

  void line(std::string s) {
    out.lines.push_back(std::move(s));
  }
};

struct Action
{
  std::string           category;
  std::string           name;
  std::string           help;
  Target                target = Target::GameMode;
  std::vector<FieldRow> rows;
  bool                  sensor = false;  // re-runnable in live mode

  std::function<bool(Ctx&)> run;
  std::function<bool(Ctx&)> load;  // optional "Load current"

  // keeps the parameter struct the rows point into alive
  std::shared_ptr<void> params;

  std::shared_ptr<std::atomic<bool>> running     = std::make_shared<std::atomic<bool>>(false);
  std::shared_ptr<std::atomic<bool>> needs_pull  = std::make_shared<std::atomic<bool>>(false);
};

using ActionList = std::vector<Action>;

ActionList GameModeActions(App& app);
ActionList DroneActions(App& app);
ActionList SensorActions(App& app);
ActionList SceneActions(App& app);

/* drive mode state //{ */

struct DriveState
{
  ueds_connector::Coordinates pos{0, 0, 0};
  ueds_connector::Rotation    rot{0, 0, 0};
  bool                        synced = false;
  int                         synced_port = -1;

  double step_pos = 50.0;  // cm
  double step_rot = 5.0;   // deg

  bool body_frame = true;
  bool collide    = true;
  bool async      = false;
  int  view       = 1;  // 0 none, 1 camera, 2 lidar

  bool                        hit = false;
  ueds_connector::Coordinates impact{0, 0, 0};
  std::string                 status = "not synced";
  double                      last_ms = 0.0;

  Preview preview;
};

//}

class App {
public:
  App(std::string address, int gm_port);
  ~App();

  /* ui-thread entry points //{ */

  // commits the form (unless from_live) and queues the call
  void Execute(Action& action, bool load = false, bool from_live = false);

  void Log(bool ok, const std::string& text);
  void Refresh();

  void SetActiveDrone(int port);
  int  ActiveDrone() const {
    return active_drone_.load();
  }

  // periodic housekeeping (connect, FPS, drone list); called from a ticker thread
  void Tick(int64_t now_ms);

  void Shutdown();

  //}

  /* worker-thread helpers //{ */

  // drone worker only: connection to the active drone, connecting lazily
  ueds_connector::UedsConnector* DroneConnection(std::string& error);
  void                           DropDroneConnections();

  // game-mode worker only
  bool ConnectGameMode();
  void ReconnectGameMode(int attempts);

  //}

  std::string address;
  int         gm_port;

  Worker gm_worker;
  Worker drone_worker;

  std::function<void()> refresh;  // wakes the UI

  // everything below is guarded by mutex
  mutable std::mutex mutex;

  bool        gm_connected = false;
  std::string gm_status    = "connecting...";
  int         api_major    = -1;
  int         api_minor    = -1;
  bool        api_ok       = false;
  float       fps          = 0.0f;
  std::string drone_status;

  std::vector<int> drone_ports;
  std::vector<std::string> device_names;  // last ListDevices

  Result   result;
  uint64_t result_seq = 0;

  std::deque<LogLine> log;

  DriveState drive;

  // set from the UI: the action the live mode re-runs (nullptr: live off)
  Action* live_action = nullptr;

private:
  std::atomic<int> active_drone_{-1};

  // only touched on gm_worker
  std::unique_ptr<ueds_connector::GameModeController> gm_;
  int64_t                                             last_connect_attempt_ms_ = -100000;
  int64_t                                             last_drones_poll_ms_     = -100000;

  // only touched on drone_worker
  std::map<int, std::unique_ptr<ueds_connector::UedsConnector>> drones_;

  void RunJob(Action& action, bool load, bool from_live);
};

/* small formatting helpers shared by the action files //{ */

std::string Num(double v, int precision = 2);
std::string Fmt(const ueds_connector::Coordinates& c);
std::string Fmt(const ueds_connector::Rotation& r);
std::string OkStr(bool ok);

//}
