#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ftxui/dom/elements.hpp>

#include "app.h"

/* decoding (viz_image.cpp) //{ */

// JPEG/PNG bytes -> RGB; nullptr and an error on failure
std::shared_ptr<Image> DecodeImage(const std::vector<unsigned char>& bytes, std::string& error);

// instance-seg PNG (r + g<<8 + b<<16 = instance id) -> false-colour ids
std::shared_ptr<Image> DecodeInstanceImage(const std::vector<unsigned char>& bytes, std::string& error, size_t* instance_count = nullptr);

// raw BGR (the segmentation camera), w*h*3 bytes
std::shared_ptr<Image> FromBgr(const std::vector<unsigned char>& bgr, int w, int h);

// uint16 millimetres, turbo-coloured between the nearest and far percentile
std::shared_ptr<Image> FromDepth(const std::vector<uint16_t>& depth, int w, int h, double& near_m, double& far_m);

// accumulated events: red brighter, blue darker
std::shared_ptr<Image> FromEvents(const ueds_connector::CameraEvents& events, int w, int h);

void TurboColor(double t, uint8_t& r, uint8_t& g, uint8_t& b);
void LabelColor(uint32_t id, uint8_t& r, uint8_t& g, uint8_t& b);

bool SavePng(const Image& image, const std::string& path);

//}

/* lidar (viz_lidar.cpp) //{ */

std::shared_ptr<LidarCloud> MakeCloud(const std::vector<ueds_connector::LidarData>& data, const std::vector<int>& labels, std::string title);

bool SavePly(const LidarCloud& cloud, const std::string& path);

enum class LidarView
{
  Top,    // X up, Y right
  Side,   // X right, Z up
  Range,  // azimuth x elevation
};

const char* LidarViewName(LidarView view);

// heading_deg: draw the vehicle's heading arrow (top view), NaN to skip
ftxui::Element LidarElement(std::shared_ptr<const LidarCloud> cloud, LidarView view, double zoom, double heading_deg);

//}

// how an image is mapped onto terminal cells
enum class ImageMode
{
  HalfBlock,  // "▀": 1x2 pixels per cell, smooth colour
  Quadrant,   // quadrant blocks: 2x2 sub-pixels per cell, two colours each; twice the horizontal detail
  Pixels,     // real pixels painted by w3mimgdisplay (X11 terminals); falls back to Quadrant
};

const char* ImageModeName(ImageMode mode);

// truecolor block image that fills its box, keeping aspect
ftxui::Element ImageElement(std::shared_ptr<const Image> image, ImageMode mode = ImageMode::Quadrant);

// display options shared by the pages and the drive tab
struct ViewSettings
{
  LidarView lidar_view = LidarView::Top;
  double    zoom       = 1.0;
  ImageMode image_mode = ImageMode::Quadrant;  // main() switches to Pixels when w3m is usable
  bool      maximized  = false;  // preview takes the whole screen
};

// rendered last in the frame: erases pixel-mode images that are gone, before the frame is printed
ftxui::Element PixelOverlaySentinel(bool suppressed);

// the whole preview block for a result
ftxui::Element PreviewElement(const Preview& preview, LidarView view, double zoom, double heading_deg, ImageMode mode = ImageMode::Quadrant);
