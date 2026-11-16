#include "drive_mode.h"

#include <chrono>
#include <cmath>

#include <ftxui/component/event.hpp>
#include <ftxui/dom/elements.hpp>

using namespace ftxui;
using ueds_connector::Coordinates;
using ueds_connector::Rotation;

namespace
{

void FetchPreview(App& app, ueds_connector::UedsConnector* drone, int view) {
  Preview preview;
  if (view == 1) {
    auto [ok, data, stamp, size] = drone->GetRgbCameraData(-1);
    std::string error;
    if (ok) {
      if (auto image = DecodeImage(data, error)) {
        image->title = "rgb";
        preview.images.push_back(image);
      }
    }
  } else if (view == 2) {
    auto [ok, data, start, stamp] = drone->GetLidarData(-1);
    if (ok) {
      preview.lidar = MakeCloud(data, {}, "lidar");
    }
  }
  std::lock_guard<std::mutex> lock(app.mutex);
  app.drive.preview = std::move(preview);
}

void PostSync(App& app) {
  app.drone_worker.Post(
      [&app] {
        std::string error;
        auto*       drone = app.DroneConnection(error);
        if (!drone) {
          std::lock_guard<std::mutex> lock(app.mutex);
          app.drive.status = error;
          app.Refresh();
          return;
        }
        auto [ok_l, loc] = drone->GetLocation();
        auto [ok_r, rot] = drone->GetRotation();
        int view;
        {
          std::lock_guard<std::mutex> lock(app.mutex);
          if (ok_l && ok_r) {
            app.drive.pos         = loc;
            app.drive.rot         = rot;
            app.drive.synced      = true;
            app.drive.synced_port = app.ActiveDrone();
            app.drive.status      = "synced from simulator";
          } else {
            app.drive.status = "sync failed";
          }
          view = app.drive.view;
        }
        FetchPreview(app, drone, view);
        app.Refresh();
      },
      "drive_sync");
}

void PostMove(App& app) {
  // coalesced: while one move is in flight, further presses only update the
  // target and a single follow-up sends the newest pose
  app.drone_worker.Post(
      [&app] {
        Coordinates pos;
        Rotation    rot;
        bool        collide, async;
        int         view;
        {
          std::lock_guard<std::mutex> lock(app.mutex);
          pos     = app.drive.pos;
          rot     = app.drive.rot;
          collide = app.drive.collide;
          async   = app.drive.async;
          view    = app.drive.view;
        }
        std::string error;
        auto*       drone = app.DroneConnection(error);
        if (!drone) {
          std::lock_guard<std::mutex> lock(app.mutex);
          app.drive.status = error;
          app.Refresh();
          return;
        }

        const auto start = std::chrono::steady_clock::now();
        if (async) {
          auto [ok] = drone->SetLocationAndRotationAsync(pos, rot, collide);
          std::lock_guard<std::mutex> lock(app.mutex);
          app.drive.hit    = false;
          app.drive.status = ok ? "sent (async)" : "async send failed";
        } else {
          auto [ok, to, rotated, hit, impact] = drone->SetLocationAndRotation(pos, rot, collide);
          std::lock_guard<std::mutex> lock(app.mutex);
          app.drive.hit    = ok && hit;
          app.drive.impact = impact;
          if (ok && hit) {
            // blocked: continue from where the simulator stopped us
            app.drive.pos = to;
          }
          app.drive.status = ok ? (hit ? "blocked by collision" : "moved") : "SetLocationAndRotation failed";
        }
        {
          std::lock_guard<std::mutex> lock(app.mutex);
          app.drive.last_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        }
        FetchPreview(app, drone, view);
        app.Refresh();
      },
      "drive_move");
}

Element KeyHelp(const std::string& keys, const std::string& what) {
  return hbox({text(keys) | bold | color(Color::Cyan), text(" " + what + "   ")});
}

}  // namespace

void DriveOnEnter(App& app) {
  bool need_sync;
  {
    std::lock_guard<std::mutex> lock(app.mutex);
    need_sync = !app.drive.synced || app.drive.synced_port != app.ActiveDrone();
  }
  if (need_sync && app.ActiveDrone() >= 0) {
    PostSync(app);
  }
}

Component DriveComponent(App& app, ViewSettings* view) {
  auto renderer = Renderer([&app, view](bool focused) {
    DriveState d;
    {
      std::lock_guard<std::mutex> lock(app.mutex);
      d = app.drive;
    }
    const int   port  = app.ActiveDrone();
    std::string title = "Drive ─ drone " + (port >= 0 ? ":" + std::to_string(port) : std::string("none (press d)")) + " ─ step pos " + Num(d.step_pos, 1) +
                        " cm ─ step rot " + Num(d.step_rot, 1) + "° ─ frame " + (d.body_frame ? "BODY" : "WORLD") + " ─ collide " +
                        (d.collide ? "on" : "off") + " ─ async " + (d.async ? "on" : "off") + " ─ view " +
                        (d.view == 0 ? "none" : d.view == 1 ? "camera" : "lidar");

    auto help = vbox({
        hbox({KeyHelp("W/S ↑/↓", "fwd/back"), KeyHelp("A/D", "left/right"), KeyHelp("R/F PgUp/PgDn", "up/down"), KeyHelp("Q/E ←/→", "yaw"),
              KeyHelp("I/K", "pitch"), KeyHelp("J/L", "roll"), KeyHelp("0", "level")}),
        hbox({KeyHelp("+/-", "pos step"), KeyHelp("[/]", "rot step"), KeyHelp("B", "body/world"), KeyHelp("C", "collide"), KeyHelp("Y", "async"),
              KeyHelp("V", "view"), KeyHelp("Z/X", "lidar zoom"), KeyHelp("Space", "sync + refresh"), KeyHelp("M", "image mode"), KeyHelp("F11", "maximize"), KeyHelp("Esc", "back")}),
    });

    auto pose = hbox({
        text("pos ") | dim,
        text(Fmt(d.pos)) | bold,
        text("    rot ") | dim,
        text(Fmt(d.rot)) | bold,
        text("    "),
        d.hit ? text("HIT at " + Fmt(d.impact)) | color(Color::Red) | bold : text("no hit") | color(Color::Green),
        text("    " + d.status + (d.last_ms > 0 ? "  " + Num(d.last_ms, 0) + " ms" : "")) | dim,
    });

    Element preview = d.preview.empty() ? text(d.view == 0 ? "preview off (V)" : "no preview yet, press Space") | dim | center
                                        : PreviewElement(d.preview, LidarView::Top, view->zoom, d.rot.yaw, view->image_mode);

    auto body = view->maximized ? vbox({pose, separator(), preview | flex})
                                : vbox({text(title) | bold, separator(), help, separator(), pose, separator(), preview | flex});
    if (focused) {
      body = body | focus;
    }
    return body;
  });

  return CatchEvent(renderer, [&app, view](Event e) {
    if (!e.is_character() && e != Event::ArrowUp && e != Event::ArrowDown && e != Event::ArrowLeft && e != Event::ArrowRight && e != Event::PageUp &&
        e != Event::PageDown) {
      return false;
    }

    double fwd = 0, right = 0, up = 0, dyaw = 0, dpitch = 0, droll = 0;
    bool   move = false, sync = false;
    {
      std::lock_guard<std::mutex> lock(app.mutex);
      auto&                       d = app.drive;
      const std::string           c = e.is_character() ? e.character() : "";

      if (c == "w" || e == Event::ArrowUp) {
        fwd = 1, move = true;
      } else if (c == "s" || e == Event::ArrowDown) {
        fwd = -1, move = true;
      } else if (c == "a") {
        right = -1, move = true;
      } else if (c == "d") {
        right = 1, move = true;
      } else if (c == "r" || e == Event::PageUp) {
        up = 1, move = true;
      } else if (c == "f" || e == Event::PageDown) {
        up = -1, move = true;
      } else if (c == "q" || e == Event::ArrowLeft) {
        dyaw = -1, move = true;
      } else if (c == "e" || e == Event::ArrowRight) {
        dyaw = 1, move = true;
      } else if (c == "i") {
        dpitch = 1, move = true;
      } else if (c == "k") {
        dpitch = -1, move = true;
      } else if (c == "j") {
        droll = -1, move = true;
      } else if (c == "l") {
        droll = 1, move = true;
      } else if (c == "0") {
        d.rot.pitch = 0, d.rot.roll = 0, move = true;
      } else if (c == "+" || c == "=") {
        d.step_pos = std::min(d.step_pos * 2.0, 100000.0);
      } else if (c == "-") {
        d.step_pos = std::max(d.step_pos / 2.0, 0.1);
      } else if (c == "]") {
        d.step_rot = std::min(d.step_rot * 2.0, 180.0);
      } else if (c == "[") {
        d.step_rot = std::max(d.step_rot / 2.0, 0.1);
      } else if (c == "b") {
        d.body_frame = !d.body_frame;
      } else if (c == "c") {
        d.collide = !d.collide;
      } else if (c == "y") {
        d.async = !d.async;
      } else if (c == "v") {
        d.view = (d.view + 1) % 3;
        sync   = true;
      } else if (c == "z") {
        view->zoom = std::min(view->zoom * 1.5, 50.0);
      } else if (c == "x") {
        view->zoom = std::max(view->zoom / 1.5, 0.05);
      } else if (c == " ") {
        sync = true;
      } else {
        return false;
      }

      if (move && (!d.synced || d.synced_port != app.ActiveDrone())) {
        // never drive from a stale pose
        d.status = "syncing first...";
        move     = false;
        sync     = true;
      }

      if (move) {
        const double yaw = d.rot.yaw * M_PI / 180.0;
        const double f   = fwd * d.step_pos;
        const double r   = right * d.step_pos;
        if (d.body_frame) {
          // Unreal: X forward, Y right, Z up, yaw clockwise seen from above
          d.pos.x += f * std::cos(yaw) - r * std::sin(yaw);
          d.pos.y += f * std::sin(yaw) + r * std::cos(yaw);
        } else {
          d.pos.x += f;
          d.pos.y += r;
        }
        d.pos.z += up * d.step_pos;
        d.rot.yaw   = std::remainder(d.rot.yaw + dyaw * d.step_rot, 360.0);
        d.rot.pitch = std::clamp(d.rot.pitch + dpitch * d.step_rot, -89.9, 89.9);
        d.rot.roll  = std::remainder(d.rot.roll + droll * d.step_rot, 360.0);
      }
    }

    if (move) {
      PostMove(app);
    } else if (sync) {
      PostSync(app);
    }
    app.Refresh();
    return true;
  });
}
