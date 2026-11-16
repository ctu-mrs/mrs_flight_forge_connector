#include <algorithm>
#include <cmath>
#include <fstream>

#include <ftxui/dom/canvas.hpp>

#include "viz.h"

using namespace ftxui;

std::shared_ptr<LidarCloud> MakeCloud(const std::vector<ueds_connector::LidarData>& data, const std::vector<int>& labels, std::string title) {
  auto cloud       = std::make_shared<LidarCloud>();
  cloud->title     = std::move(title);
  cloud->has_label = !labels.empty();
  cloud->points.reserve(data.size());

  std::vector<float> distances;
  distances.reserve(data.size());
  for (size_t i = 0; i < data.size(); ++i) {
    const auto& p = data[i];
    if (!(p.distance > 0.0) || !std::isfinite(p.distance)) {
      continue;
    }
    const int label = i < labels.size() ? labels[i] : -1;
    cloud->points.push_back(LidarPoint{static_cast<float>(p.directionX * p.distance), static_cast<float>(p.directionY * p.distance),
                                       static_cast<float>(p.directionZ * p.distance), static_cast<float>(p.distance), label});
    distances.push_back(static_cast<float>(p.distance));
  }
  if (!distances.empty()) {
    const size_t k = std::min(distances.size() - 1, distances.size() * 95 / 100);
    std::nth_element(distances.begin(), distances.begin() + k, distances.end());
    cloud->auto_range = std::max(1e-3f, distances[k]);
  }
  return cloud;
}

bool SavePly(const LidarCloud& cloud, const std::string& path) {
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "ply\nformat ascii 1.0\nelement vertex " << cloud.points.size() << "\nproperty float x\nproperty float y\nproperty float z\n"
      << "property float distance\nproperty int label\nend_header\n";
  for (const auto& p : cloud.points) {
    out << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.distance << ' ' << p.label << '\n';
  }
  return static_cast<bool>(out);
}

const char* LidarViewName(LidarView view) {
  switch (view) {
    case LidarView::Top:
      return "top (X↑ Y→)";
    case LidarView::Side:
      return "side (X→ Z↑)";
    case LidarView::Range:
      return "range image (azimuth × elevation)";
  }
  return "";
}

Element LidarElement(std::shared_ptr<const LidarCloud> cloud, LidarView view, double zoom, double heading_deg) {
  return canvas([cloud, view, zoom, heading_deg](Canvas& c) {
    const int W = c.width();
    const int H = c.height();
    if (W <= 0 || H <= 0 || !cloud) {
      return;
    }

    const double range = cloud->auto_range / std::max(zoom, 1e-3);
    const double cx    = W / 2.0;
    const double cy    = H / 2.0;
    // braille dots are about twice as tall as wide on screen: 2x4 dots in a 1x2 cell
    const double scale = std::min(W, H) / 2.0 / range;

    auto color_of = [&](const LidarPoint& p) {
      uint8_t r, g, b;
      if (cloud->has_label && p.label >= 0) {
        LabelColor(static_cast<uint32_t>(p.label) + 1, r, g, b);
      } else {
        TurboColor(1.0 - std::min(1.0, p.distance / range), r, g, b);
      }
      return Color::RGB(r, g, b);
    };

    if (view == LidarView::Range) {
      double el_min = 1e9, el_max = -1e9;
      for (const auto& p : cloud->points) {
        const double el = std::atan2(p.z, std::hypot(p.x, p.y));
        el_min          = std::min(el_min, el);
        el_max          = std::max(el_max, el);
      }
      if (el_max - el_min < 1e-3) {
        el_max = el_min + 1e-3;
      }
      for (const auto& p : cloud->points) {
        const double az = std::atan2(p.y, p.x);
        const double el = std::atan2(p.z, std::hypot(p.x, p.y));
        const int    x  = static_cast<int>((az + M_PI) / (2 * M_PI) * (W - 1));
        const int    y  = static_cast<int>((el_max - el) / (el_max - el_min) * (H - 1));
        c.DrawPoint(x, y, true, color_of(p));
      }
      c.DrawText(0, 0, "-180°", Color::GrayDark);
      c.DrawText(std::max(0, W - 8), 0, "+180°", Color::GrayDark);
      return;
    }

    // range rings at 25/50/100 % of the view range
    for (double ring : {0.25, 0.5, 1.0}) {
      const int r = static_cast<int>(ring * range * scale);
      if (r > 2) {
        c.DrawPointCircle(static_cast<int>(cx), static_cast<int>(cy), r, Color::GrayDark);
      }
    }

    for (const auto& p : cloud->points) {
      double sx, sy;
      if (view == LidarView::Top) {
        sx = cx + p.y * scale;
        sy = cy - p.x * scale;
      } else {
        sx = cx + p.x * scale;
        sy = cy - p.z * scale;
      }
      if (sx < 0 || sy < 0 || sx >= W || sy >= H) {
        continue;
      }
      c.DrawPoint(static_cast<int>(sx), static_cast<int>(sy), true, color_of(p));
    }

    // the vehicle
    c.DrawPointCircleFilled(static_cast<int>(cx), static_cast<int>(cy), 1, Color::White);
    if (view == LidarView::Top && std::isfinite(heading_deg)) {
      const double yaw = heading_deg * M_PI / 180.0;
      const double len = std::min(W, H) / 8.0;
      c.DrawPointLine(static_cast<int>(cx), static_cast<int>(cy), static_cast<int>(cx + std::sin(yaw) * len), static_cast<int>(cy - std::cos(yaw) * len),
                      Color::White);
    }
  });
}
