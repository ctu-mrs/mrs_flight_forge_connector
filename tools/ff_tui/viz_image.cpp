#include <algorithm>
#include <cmath>
#include <list>
#include <mutex>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#include <stb_image.h>

#include <fpng.h>

#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>

#include "pixel_overlay.h"
#include "viz.h"

using namespace ftxui;

/* decoding //{ */

std::shared_ptr<Image> DecodeImage(const std::vector<unsigned char>& bytes, std::string& error) {
  if (bytes.empty()) {
    error = "empty frame";
    return nullptr;
  }
  int  w = 0, h = 0, channels = 0;
  auto pixels = stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &w, &h, &channels, 3);
  if (!pixels) {
    error = std::string("decode failed: ") + stbi_failure_reason();
    return nullptr;
  }
  auto image = std::make_shared<Image>();
  image->w   = w;
  image->h   = h;
  image->rgb.assign(pixels, pixels + static_cast<size_t>(w) * h * 3);
  stbi_image_free(pixels);
  return image;
}

std::shared_ptr<Image> DecodeInstanceImage(const std::vector<unsigned char>& bytes, std::string& error, size_t* instance_count) {
  if (bytes.empty()) {
    error = "empty frame";
    return nullptr;
  }
  int  w = 0, h = 0, channels = 0;
  auto pixels = stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &w, &h, &channels, 4);
  if (!pixels) {
    error = std::string("decode failed: ") + stbi_failure_reason();
    return nullptr;
  }
  auto image = std::make_shared<Image>();
  image->w   = w;
  image->h   = h;
  image->rgb.resize(static_cast<size_t>(w) * h * 3);
  std::vector<uint32_t> seen;
  for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i) {
    const uint32_t id = pixels[i * 4] | (pixels[i * 4 + 1] << 8) | (pixels[i * 4 + 2] << 16);
    if (id == 0) {
      image->rgb[i * 3] = image->rgb[i * 3 + 1] = image->rgb[i * 3 + 2] = 0;
      continue;
    }
    LabelColor(id, image->rgb[i * 3], image->rgb[i * 3 + 1], image->rgb[i * 3 + 2]);
    if (instance_count && seen.size() < 100000) {
      seen.push_back(id);
    }
  }
  stbi_image_free(pixels);
  if (instance_count) {
    std::sort(seen.begin(), seen.end());
    *instance_count = std::unique(seen.begin(), seen.end()) - seen.begin();
  }
  return image;
}

std::shared_ptr<Image> FromBgr(const std::vector<unsigned char>& bgr, int w, int h) {
  auto image = std::make_shared<Image>();
  image->w   = w;
  image->h   = h;
  image->rgb.resize(static_cast<size_t>(w) * h * 3);
  for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i) {
    image->rgb[i * 3 + 0] = bgr[i * 3 + 2];
    image->rgb[i * 3 + 1] = bgr[i * 3 + 1];
    image->rgb[i * 3 + 2] = bgr[i * 3 + 0];
  }
  return image;
}

std::shared_ptr<Image> FromDepth(const std::vector<uint16_t>& depth, int w, int h, double& near_m, double& far_m) {
  std::vector<uint16_t> valid;
  valid.reserve(depth.size());
  for (auto d : depth) {
    if (d != 0 && d != 65535) {
      valid.push_back(d);
    }
  }
  double lo = 0.0, hi = 1.0;
  if (!valid.empty()) {
    std::sort(valid.begin(), valid.end());
    lo = valid[valid.size() / 100];
    hi = valid[std::min(valid.size() - 1, valid.size() * 98 / 100)];
    if (hi <= lo) {
      hi = lo + 1.0;
    }
  }
  near_m = lo / 1000.0;
  far_m  = hi / 1000.0;

  auto image = std::make_shared<Image>();
  image->w   = w;
  image->h   = h;
  image->rgb.resize(static_cast<size_t>(w) * h * 3);
  for (size_t i = 0; i < static_cast<size_t>(w) * h && i < depth.size(); ++i) {
    const uint16_t d = depth[i];
    if (d == 0) {
      image->rgb[i * 3] = image->rgb[i * 3 + 1] = image->rgb[i * 3 + 2] = 0;
      continue;
    }
    if (d == 65535) {
      image->rgb[i * 3] = image->rgb[i * 3 + 1] = image->rgb[i * 3 + 2] = 40;
      continue;
    }
    const double t = std::clamp((d - lo) / (hi - lo), 0.0, 1.0);
    TurboColor(1.0 - t, image->rgb[i * 3], image->rgb[i * 3 + 1], image->rgb[i * 3 + 2]);  // near = red
  }
  return image;
}

std::shared_ptr<Image> FromEvents(const ueds_connector::CameraEvents& events, int w, int h) {
  auto image = std::make_shared<Image>();
  image->w   = std::max(1, w);
  image->h   = std::max(1, h);
  std::vector<int> pos(static_cast<size_t>(image->w) * image->h, 0), neg(pos.size(), 0);
  for (size_t i = 0; i < events.size(); ++i) {
    const uint32_t xyp = events.xyp[i];
    const int      x   = xyp & 0xFFFF;
    const int      y   = (xyp >> 16) & 0x7FFF;
    if (x >= image->w || y >= image->h) {
      continue;
    }
    ((xyp & 0x80000000u) ? pos : neg)[static_cast<size_t>(y) * image->w + x]++;
  }
  image->rgb.resize(pos.size() * 3);
  for (size_t i = 0; i < pos.size(); ++i) {
    const int p = std::min(pos[i] * 80, 255);
    const int n = std::min(neg[i] * 80, 255);
    image->rgb[i * 3 + 0] = static_cast<uint8_t>(p);
    image->rgb[i * 3 + 1] = static_cast<uint8_t>(std::min(255, (p + n) / 4));
    image->rgb[i * 3 + 2] = static_cast<uint8_t>(n);
  }
  return image;
}

// Google's Turbo colormap, polynomial fit (t in [0,1])
void TurboColor(double t, uint8_t& r, uint8_t& g, uint8_t& b) {
  t              = std::clamp(t, 0.0, 1.0);
  const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
  const double rr = 0.13572138 + 4.61539260 * t - 42.66032258 * t2 + 132.13108234 * t3 - 152.94239396 * t4 + 59.28637943 * t5;
  const double gg = 0.09140261 + 2.19418839 * t + 4.84296658 * t2 - 14.18503333 * t3 + 4.27729857 * t4 + 2.82956604 * t5;
  const double bb = 0.10667330 + 12.64194608 * t - 60.58204836 * t2 + 110.36276771 * t3 - 89.90310912 * t4 + 27.34824973 * t5;
  r               = static_cast<uint8_t>(std::clamp(rr, 0.0, 1.0) * 255.0);
  g               = static_cast<uint8_t>(std::clamp(gg, 0.0, 1.0) * 255.0);
  b               = static_cast<uint8_t>(std::clamp(bb, 0.0, 1.0) * 255.0);
}

// stable, well-separated colour per id (golden-ratio hue walk)
void LabelColor(uint32_t id, uint8_t& r, uint8_t& g, uint8_t& b) {
  const double h = std::fmod(id * 0.618033988749895, 1.0) * 6.0;
  const double s = 0.65 + 0.35 * ((id * 2654435761u >> 16) % 2);
  const double v = 0.75 + 0.25 * ((id * 40503u >> 8) % 2);
  const int    i = static_cast<int>(h);
  const double f = h - i;
  const double p = v * (1 - s), q = v * (1 - s * f), u = v * (1 - s * (1 - f));
  double       rr, gg, bb;
  switch (i % 6) {
    case 0: rr = v, gg = u, bb = p; break;
    case 1: rr = q, gg = v, bb = p; break;
    case 2: rr = p, gg = v, bb = u; break;
    case 3: rr = p, gg = q, bb = v; break;
    case 4: rr = u, gg = p, bb = v; break;
    default: rr = v, gg = p, bb = q; break;
  }
  r = static_cast<uint8_t>(rr * 255);
  g = static_cast<uint8_t>(gg * 255);
  b = static_cast<uint8_t>(bb * 255);
}

bool SavePng(const Image& image, const std::string& path) {
  static std::once_flag once;
  std::call_once(once, [] { fpng::fpng_init(); });
  return fpng::fpng_encode_image_to_file(path.c_str(), image.rgb.data(), image.w, image.h, 3);
}

//}

/* half-block image element //{ */

namespace
{

struct Scaled
{
  std::shared_ptr<const Image> source;  // held so its address cannot be reused by a new frame
  int                  w = 0, h = 0;
  std::vector<uint8_t> rgb;
};

// box-filter downscale, cached: FTXUI redraws on every event and a frame is
// usually drawn many times at the same size
const Scaled& ScaleCached(const std::shared_ptr<const Image>& image_ptr, int w, int h) {
  const Image& image = *image_ptr;
  static std::list<Scaled> cache;  // UI thread only
  for (auto it = cache.begin(); it != cache.end(); ++it) {
    if (it->source == image_ptr && it->w == w && it->h == h) {
      cache.splice(cache.begin(), cache, it);
      return cache.front();
    }
  }

  Scaled s;
  s.source = image_ptr;
  s.w      = w;
  s.h      = h;
  s.rgb.resize(static_cast<size_t>(w) * h * 3);
  for (int oy = 0; oy < h; ++oy) {
    const int sy0 = oy * image.h / h;
    const int sy1 = std::max(sy0 + 1, (oy + 1) * image.h / h);
    for (int ox = 0; ox < w; ++ox) {
      const int sx0 = ox * image.w / w;
      const int sx1 = std::max(sx0 + 1, (ox + 1) * image.w / w);
      unsigned  acc[3] = {0, 0, 0};
      unsigned  n      = 0;
      // sparse sampling of large blocks keeps the cost bounded
      const int stepy = std::max(1, (sy1 - sy0) / 4);
      const int stepx = std::max(1, (sx1 - sx0) / 4);
      for (int sy = sy0; sy < sy1; sy += stepy) {
        for (int sx = sx0; sx < sx1; sx += stepx) {
          const uint8_t* p = &image.rgb[(static_cast<size_t>(sy) * image.w + sx) * 3];
          acc[0] += p[0];
          acc[1] += p[1];
          acc[2] += p[2];
          ++n;
        }
      }
      uint8_t* o = &s.rgb[(static_cast<size_t>(oy) * w + ox) * 3];
      o[0]       = static_cast<uint8_t>(acc[0] / n);
      o[1]       = static_cast<uint8_t>(acc[1] / n);
      o[2]       = static_cast<uint8_t>(acc[2] / n);
    }
  }

  cache.push_front(std::move(s));
  if (cache.size() > 6) {
    cache.pop_back();
  }
  return cache.front();
}

// Unicode quadrant blocks indexed by which 2x2 sub-pixels are foreground:
// bit 0 top-left, 1 top-right, 2 bottom-left, 3 bottom-right
const char* const kQuadrants[16] = {" ", "▘", "▝", "▀", "▖", "▌", "▞", "▛", "▗", "▚", "▐", "▜", "▄", "▙", "▟", "█"};

class BlockImage : public Node {
public:
  BlockImage(std::shared_ptr<const Image> image, ImageMode mode) : image_(std::move(image)), mode_(mode) {
  }

  void ComputeRequirement() override {
    requirement_.min_x         = 2;
    requirement_.min_y         = 1;
    requirement_.flex_grow_x   = 1;
    requirement_.flex_grow_y   = 1;
    requirement_.flex_shrink_x = 1;
    requirement_.flex_shrink_y = 1;
  }

  void Render(Screen& screen) override {
    if (!image_ || image_->w <= 0 || image_->h <= 0 || image_->rgb.size() < static_cast<size_t>(image_->w) * image_->h * 3) {
      return;
    }
    const int cells_w = box_.x_max - box_.x_min + 1;
    const int cells_h = box_.y_max - box_.y_min + 1;
    if (cells_w <= 0 || cells_h <= 0) {
      return;
    }
    // Fit in units of "square pixels": a cell is about 1 wide and 2 tall, so
    // it holds 1x2 of them. Half-block draws exactly that; quadrant splits
    // each column in two again (sub-pixels half as wide as they are tall).
    const double scale = std::min(static_cast<double>(cells_w) / image_->w, static_cast<double>(cells_h * 2) / image_->h);
    const int    w     = std::max(1, static_cast<int>(image_->w * scale));  // cells across
    const int    h     = std::max(1, static_cast<int>(image_->h * scale));  // square pixels down
    const int    rows  = (h + 1) / 2;
    const int    x0    = box_.x_min + (cells_w - w) / 2;
    const int    y0    = box_.y_min + (cells_h - rows) / 2;

    if (mode_ == ImageMode::Pixels && PixelOverlay::Get().Available()) {
      // leave the cells blank; w3mimgdisplay paints the real image here after the frame is printed
      PixelOverlay::Get().Add(box_, image_);
      return;
    }

    if (mode_ == ImageMode::HalfBlock) {
      const Scaled& s = ScaleCached(image_, w, h);
      for (int cy = 0; cy < rows; ++cy) {
        for (int cx = 0; cx < w; ++cx) {
          Pixel&         pixel   = screen.PixelAt(x0 + cx, y0 + cy);
          const uint8_t* top     = &s.rgb[(static_cast<size_t>(cy * 2) * w + cx) * 3];
          pixel.character        = "▀";
          pixel.foreground_color = Color::RGB(top[0], top[1], top[2]);
          if (cy * 2 + 1 < h) {
            const uint8_t* bottom  = &s.rgb[(static_cast<size_t>(cy * 2 + 1) * w + cx) * 3];
            pixel.background_color = Color::RGB(bottom[0], bottom[1], bottom[2]);
          }
        }
      }
      return;
    }

    // quadrant: 2x2 sub-pixels per cell, two colours per cell. Try every
    // split (patterns containing the top-left bit cover all up to swapping
    // fg/bg) and keep the one with the least squared error.
    const int     sw = w * 2;
    const Scaled& s  = ScaleCached(image_, sw, h);
    for (int cy = 0; cy < rows; ++cy) {
      for (int cx = 0; cx < w; ++cx) {
        int px[4][3];
        for (int q = 0; q < 4; ++q) {
          const int sx = cx * 2 + (q & 1);
          const int sy = std::min(h - 1, cy * 2 + (q >> 1));
          const uint8_t* p = &s.rgb[(static_cast<size_t>(sy) * sw + sx) * 3];
          px[q][0] = p[0], px[q][1] = p[1], px[q][2] = p[2];
        }

        int  best_pattern = 15;
        long best_error   = -1;
        int  best_fg[3] = {0, 0, 0}, best_bg[3] = {0, 0, 0};
        for (int pattern = 1; pattern < 16; pattern += 2) {
          int sum_fg[3] = {0, 0, 0}, sum_bg[3] = {0, 0, 0}, n_fg = 0, n_bg = 0;
          for (int q = 0; q < 4; ++q) {
            int* sum = (pattern >> q & 1) ? sum_fg : sum_bg;
            ((pattern >> q & 1) ? n_fg : n_bg)++;
            sum[0] += px[q][0], sum[1] += px[q][1], sum[2] += px[q][2];
          }
          int fg[3], bg[3];
          for (int c = 0; c < 3; ++c) {
            fg[c] = sum_fg[c] / n_fg;
            bg[c] = n_bg ? sum_bg[c] / n_bg : fg[c];
          }
          long error = 0;
          for (int q = 0; q < 4; ++q) {
            const int* ref = (pattern >> q & 1) ? fg : bg;
            for (int c = 0; c < 3; ++c) {
              const int d = px[q][c] - ref[c];
              error += d * d;
            }
          }
          if (best_error < 0 || error < best_error) {
            best_error   = error;
            best_pattern = pattern;
            std::copy(fg, fg + 3, best_fg);
            std::copy(bg, bg + 3, best_bg);
          }
        }

        Pixel& pixel           = screen.PixelAt(x0 + cx, y0 + cy);
        pixel.character        = kQuadrants[best_pattern];
        pixel.foreground_color = Color::RGB(best_fg[0], best_fg[1], best_fg[2]);
        pixel.background_color = Color::RGB(best_bg[0], best_bg[1], best_bg[2]);
      }
    }
  }

private:
  std::shared_ptr<const Image> image_;
  ImageMode                    mode_;
};

}  // namespace

const char* ImageModeName(ImageMode mode) {
  switch (mode) {
    case ImageMode::Pixels:
      return PixelOverlay::Get().Available() ? "pixels (w3m)" : "quadrant 2×2 (w3m unavailable)";
    case ImageMode::Quadrant:
      return "quadrant 2×2";
    case ImageMode::HalfBlock:
      return "half-block 1×2";
  }
  return "";
}

namespace
{

class OverlaySentinel : public Node {
public:
  explicit OverlaySentinel(bool suppressed) : suppressed_(suppressed) {
  }
  void ComputeRequirement() override {
    requirement_ = Requirement{};
  }
  void Render(Screen& /*screen*/) override {
    PixelOverlay::Get().ClearStale(suppressed_);
  }

private:
  bool suppressed_;
};

}  // namespace

Element PixelOverlaySentinel(bool suppressed) {
  return std::make_shared<OverlaySentinel>(suppressed);
}

Element ImageElement(std::shared_ptr<const Image> image, ImageMode mode) {
  return std::make_shared<BlockImage>(std::move(image), mode);
}

//}

Element PreviewElement(const Preview& preview, LidarView view, double zoom, double heading_deg, ImageMode mode) {
  Elements parts;

  if (!preview.images.empty()) {
    Elements images;
    for (const auto& image : preview.images) {
      const std::string title = image->title + "  " + std::to_string(image->w) + "×" + std::to_string(image->h) + "  " + ImageModeName(mode) + " [m]";
      images.push_back(vbox({text(title) | dim, ImageElement(image, mode) | flex}) | flex);
    }
    parts.push_back(hbox(images) | flex);
  }

  if (preview.lidar) {
    const auto& c     = *preview.lidar;
    std::string title = c.title + "  " + std::to_string(c.points.size()) + " pts  view: " + LidarViewName(view) + " [v]  zoom ×" + Num(zoom, 2) +
                        " [+/-]  range " + Num(c.auto_range / zoom, 1);
    parts.push_back(vbox({text(title) | dim, LidarElement(preview.lidar, view, zoom, heading_deg) | flex}) | flex);
  }

  if (preview.range) {
    const double max   = preview.range_max > 0 ? preview.range_max : std::max(1.0, *preview.range * 1.5);
    const float  ratio = static_cast<float>(std::clamp(*preview.range / max, 0.0, 1.0));
    parts.push_back(vbox({text("range " + Num(*preview.range / 100.0, 2) + " m  (" + Num(*preview.range, 1) + " raw)") | bold,
                          hbox({gauge(ratio) | color(Color::Green) | flex, text(" " + Num(max / 100.0, 1) + " m") | dim})}));
  }

  return vbox(parts);
}
