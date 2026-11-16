#include <fstream>
#include <sstream>

#include "actions_common.h"

using namespace ueds_connector;

namespace
{

struct PoseParams
{
  double x = 0, y = 0, z = 0;
  double pitch = 0, yaw = 0, roll = 0;
  bool   collide = true;
};

bool LoadPose(Ctx& c, PoseParams& p) {
  auto [ok_l, loc] = c.drone->GetLocation();
  auto [ok_r, rot] = c.drone->GetRotation();
  if (ok_l) {
    p.x = loc.x, p.y = loc.y, p.z = loc.z;
  }
  if (ok_r) {
    p.pitch = rot.pitch, p.yaw = rot.yaw, p.roll = rot.roll;
  }
  c.line("location " + Fmt(loc) + " | rotation " + Fmt(rot));
  return ok_l && ok_r;
}

}  // namespace

ActionList DroneActions(App& app) {
  ActionList   list;
  const Target DR = Target::Drone;

  /* Connection //{ */

  list.push_back(MakeAction("Connection", "Ping", DR, "Round-trip check on the active drone's socket.", [](Ctx& c) {
    const bool ok = c.drone->Ping();
    c.line(ok ? "pong" : "no answer");
    return ok;
  }));

  list.push_back(MakeAction("Connection", "GetMeta", DR, "The drone server's op catalogue.", [](Ctx& c) { return DescribeMeta(c, c.drone); }));

  list.push_back(MakeAction("Connection", "CheckCompatibility", DR, "Drone ops this connector knows that the server lacks.",
                            [](Ctx& c) { return DescribeCompat(c, c.drone); }));

  {
    struct P
    {
      int ms = 10000;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Connection", "SetReceiveTimeoutMs", DR, "Client-side reply timeout of the drone socket.", [p](Ctx& c) {
      c.drone->SetReceiveTimeoutMs(p->ms);
      c.line("timeout " + std::to_string(c.drone->GetReceiveTimeoutMs()) + " ms");
      return true;
    }));
    a.load = [p](Ctx& c) {
      p->ms = c.drone->GetReceiveTimeoutMs();
      c.line("timeout " + std::to_string(p->ms) + " ms");
      return true;
    };
    Bind(a, p, {Row("Timeout [ms]", {Field::I(&p->ms)})});
  }

  //}

  /* Pose //{ */

  list.push_back(MakeAction("Pose", "GetLocation", DR, "Drone location (cm, Unreal frame).", [](Ctx& c) {
    auto [ok, loc] = c.drone->GetLocation();
    c.line(Fmt(loc));
    return ok;
  }));

  list.push_back(MakeAction("Pose", "GetRotation", DR, "Drone rotation (deg).", [](Ctx& c) {
    auto [ok, rot] = c.drone->GetRotation();
    c.line(Fmt(rot));
    return ok;
  }));

  list.push_back(MakeAction("Pose", "GetCrashState", DR, "Whether the drone has crashed.", [](Ctx& c) {
    auto [ok, crashed] = c.drone->GetCrashState();
    c.line(crashed ? "CRASHED" : "not crashed");
    return ok;
  }));

  {
    auto p  = std::make_shared<PoseParams>();
    auto& a = list.emplace_back(MakeAction("Pose", "SetLocation", DR, "Teleport; with collisions it stops at the first hit.", [p](Ctx& c) {
      auto [ok, to, hit, impact] = c.drone->SetLocation(Coordinates(p->x, p->y, p->z), p->collide);
      c.line("teleported to " + Fmt(to) + (hit ? "  HIT at " + Fmt(impact) : "  no hit"));
      return ok;
    }));
    a.load = [p](Ctx& c) { return LoadPose(c, *p); };
    Bind(a, p, {Row("Location [cm]", Vec3(&p->x, &p->y, &p->z)), Row("Check collisions", {Field::B(&p->collide)})});
  }

  {
    auto p  = std::make_shared<PoseParams>();
    auto& a = list.emplace_back(MakeAction("Pose", "SetRotation", DR, "Set rotation (deg).", [p](Ctx& c) {
      auto [ok, to, hit, impact] = c.drone->SetRotation(Rotation(p->pitch, p->yaw, p->roll));
      c.line("rotated to " + Fmt(to) + (hit ? "  HIT at " + Fmt(impact) : ""));
      return ok;
    }));
    a.load = [p](Ctx& c) { return LoadPose(c, *p); };
    Bind(a, p, {Row("Rotation [deg]", Vec3(&p->pitch, &p->yaw, &p->roll, "pitch", "yaw", "roll"))});
  }

  {
    auto p  = std::make_shared<PoseParams>();
    auto& a = list.emplace_back(MakeAction("Pose", "SetLocationAndRotation", DR, "Teleport and rotate in one call.", [p](Ctx& c) {
      auto [ok, to, rot, hit, impact] = c.drone->SetLocationAndRotation(Coordinates(p->x, p->y, p->z), Rotation(p->pitch, p->yaw, p->roll), p->collide);
      c.line("at " + Fmt(to) + " | " + Fmt(rot) + (hit ? "  HIT at " + Fmt(impact) : ""));
      return ok;
    }));
    a.load = [p](Ctx& c) { return LoadPose(c, *p); };
    Bind(a, p,
         {Row("Location [cm]", Vec3(&p->x, &p->y, &p->z)), Row("Rotation [deg]", Vec3(&p->pitch, &p->yaw, &p->roll, "pitch", "yaw", "roll")),
          Row("Collide", {Field::B(&p->collide)})});
  }

  {
    auto p  = std::make_shared<PoseParams>();
    auto& a = list.emplace_back(MakeAction("Pose", "SetLocationAndRotationAsync", DR, "Fire-and-forget pose; acknowledged before it is applied.", [p](Ctx& c) {
      auto [ok] = c.drone->SetLocationAndRotationAsync(Coordinates(p->x, p->y, p->z), Rotation(p->pitch, p->yaw, p->roll), p->collide);
      c.line(ok ? "acknowledged" : "not acknowledged");
      return ok;
    }));
    a.load = [p](Ctx& c) { return LoadPose(c, *p); };
    Bind(a, p,
         {Row("Location [cm]", Vec3(&p->x, &p->y, &p->z)), Row("Rotation [deg]", Vec3(&p->pitch, &p->yaw, &p->roll, "pitch", "yaw", "roll")),
          Row("Collide", {Field::B(&p->collide)})});
  }

  {
    struct P
    {
      std::string path = "trajectory.csv";
    };
    auto p  = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Pose", "SetPoseTrajectory", DR,
                                           "CSV rows 't,x,y,z,pitch,yaw,roll' the pawn interpolates along. An empty path detaches.", [p](Ctx& c) {
                                             std::vector<PoseSample> samples;
                                             if (!p->path.empty()) {
                                               std::ifstream file(p->path);
                                               if (!file) {
                                                 c.line("cannot open '" + p->path + "'");
                                                 return false;
                                               }
                                               std::string line;
                                               while (std::getline(file, line)) {
                                                 std::replace(line.begin(), line.end(), ',', ' ');
                                                 std::istringstream in(line);
                                                 PoseSample         s;
                                                 if (in >> s.t >> s.position.x >> s.position.y >> s.position.z >> s.orientation.pitch >> s.orientation.yaw >>
                                                     s.orientation.roll) {
                                                   samples.push_back(s);
                                                 }
                                               }
                                             }
                                             const bool ok = c.drone->SetPoseTrajectory(samples);
                                             c.line(samples.empty() ? "trajectory detached" : std::to_string(samples.size()) + " samples sent");
                                             return ok;
                                           }));
    Bind(a, p, {Row("CSV path", {Field::S(&p->path, "", 50)}, "header lines are skipped")});
  }

  {
    struct P
    {
      std::string speeds = "500 -500 -500 500";
    };
    auto p  = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Pose", "SetPropellerSpeeds", DR, "Per-propeller rad/s (RL RR FL FR ...); empty hands them back to default spin.",
                                           [p](Ctx& c) {
                                             std::vector<double> speeds;
                                             std::string         normalized = p->speeds;
                                             std::replace(normalized.begin(), normalized.end(), ',', ' ');
                                             std::istringstream in(normalized);
                                             double             v;
                                             while (in >> v) {
                                               speeds.push_back(v);
                                             }
                                             int32_t count = 0;
                                             const bool ok = c.drone->SetPropellerSpeeds(speeds, count);
                                             c.line(std::to_string(speeds.size()) + " speed(s) sent, frame has " + std::to_string(count) + " propeller(s)");
                                             return ok;
                                           }));
    Bind(a, p, {Row("Speeds [rad/s]", {Field::S(&p->speeds, "", 40)})});
  }

  list.push_back(MakeAction("Pose", "GetMoveLineVisible", DR, "Whether the debug move line is drawn.", [](Ctx& c) {
    auto [ok, visible] = c.drone->GetMoveLineVisible();
    c.line(visible ? "visible" : "hidden");
    return ok;
  }));

  {
    struct P
    {
      bool visible = true;
    };
    auto p  = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Pose", "SetMoveLineVisible", DR, "Show or hide the debug move line.",
                                           [p](Ctx& c) { return c.drone->SetMoveLineVisible(p->visible); }));
    a.load = [p](Ctx& c) {
      auto [ok, visible] = c.drone->GetMoveLineVisible();
      p->visible         = visible;
      return ok;
    };
    Bind(a, p, {Row("Visible", {Field::B(&p->visible)})});
  }

  //}

  (void)app;
  return list;
}
