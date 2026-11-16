#include "pixel_overlay.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <filesystem>

#include "viz.h"

namespace
{

std::string Env(const char* name) {
  const char* v = std::getenv(name);
  return v ? v : "";
}

long NowMs() {
  return static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string Run(const std::string& cmd) {
  std::string out;
  FILE*       p = popen(cmd.c_str(), "r");
  if (!p) {
    return out;
  }
  char buf[512];
  while (fgets(buf, sizeof(buf), p)) {
    out += buf;
  }
  pclose(p);
  return out;
}

bool SameBox(const ftxui::Box& a, const ftxui::Box& b) {
  return a.x_min == b.x_min && a.x_max == b.x_max && a.y_min == b.y_min && a.y_max == b.y_max;
}

}  // namespace

PixelOverlay& PixelOverlay::Get() {
  static PixelOverlay overlay;
  return overlay;
}

PixelOverlay::~PixelOverlay() {
  Shutdown();
}

bool PixelOverlay::Available() {
  if (checked_ >= 0) {
    return checked_ == 1;
  }
  checked_ = 0;

  const std::string force = Env("FF_TUI_PIXELS");
  if (force == "0") {
    reason_ = "disabled by FF_TUI_PIXELS=0";
    return false;
  }

  std::vector<std::string> candidates;
  if (!Env("FF_TUI_W3MIMGDISPLAY").empty()) {
    candidates.push_back(Env("FF_TUI_W3MIMGDISPLAY"));
  }
  for (const char* p : {"/usr/lib/w3m/w3mimgdisplay", "/usr/libexec/w3m/w3mimgdisplay", "/usr/lib64/w3m/w3mimgdisplay", "/usr/local/libexec/w3m/w3mimgdisplay"}) {
    candidates.push_back(p);
  }
  for (const auto& c : candidates) {
    if (access(c.c_str(), X_OK) == 0) {
      binary_ = c;
      break;
    }
  }
  if (binary_.empty()) {
    reason_ = "w3mimgdisplay not found (install w3m-img)";
    return false;
  }

  if (force != "1") {
    if (Env("DISPLAY").empty()) {
      reason_ = "no X display";
      return false;
    }
    if (!Env("STY").empty()) {
      reason_ = "inside GNU screen (FF_TUI_PIXELS=1 to try anyway)";
      return false;
    }
    if (!Env("SSH_CONNECTION").empty()) {
      reason_ = "over ssh (FF_TUI_PIXELS=1 to try anyway)";
      return false;
    }
  }

  in_tmux_ = !Env("TMUX").empty();
  if (in_tmux_) {
    // no client attached yet is fine: the pane just counts as not visible
    RefreshTmux(true);
  }

  checked_ = 1;
  return true;
}

// one tmux query for everything the overlay needs; cached for a moment
void PixelOverlay::RefreshTmux(bool force) {
  const long now = NowMs();
  if (!force && now - tmux_checked_ms_ < 200) {
    return;
  }
  tmux_checked_ms_ = now;

  const std::string pane   = Env("TMUX_PANE");
  const std::string target = pane.empty() ? "" : " -t '" + pane + "'";
  const std::string out    = Run("tmux display -p" + target +
                                 " '#{window_active} #{session_attached} #{pane_left} #{pane_top} #{client_width} #{client_height} #{status} "
                                 "#{status-position} #{client_cell_width} #{client_cell_height} #{client_tty}' 2>/dev/null;"
                                 " tmux show-environment" + (pane.empty() ? std::string() : " -t '" + pane + "'") + " WINDOWID 2>/dev/null");

  std::istringstream in(out);
  std::string        line;
  if (!std::getline(in, line)) {
    tmux_.visible = false;
    return;
  }
  std::istringstream f(line);
  int                window_active = 0, attached = 0, pane_left = 0, pane_top = 0;
  std::string        status, position;
  TmuxInfo           t;
  f >> window_active >> attached >> pane_left >> pane_top >> t.client_cols >> t.client_rows >> status >> position >> t.cell_w >> t.cell_h >> t.client_tty;
  const int status_lines = status == "off" ? 0 : status == "on" ? 1 : std::max(0, std::atoi(status.c_str()));
  t.visible              = window_active == 1 && attached > 0 && t.client_cols > 0;
  t.origin_col           = pane_left;
  t.origin_row           = pane_top + (position == "top" ? status_lines : 0);

  bool windowid_known = false;
  if (std::getline(in, line) && line.rfind("WINDOWID=", 0) == 0) {
    t.windowid     = line.substr(9);
    windowid_known = true;
  }

  // the attached terminal can change (detach + attach elsewhere): w3mimgdisplay
  // must draw into the current one, and its cell size may differ
  if (windowid_known != have_windowid_ || t.windowid != child_windowid_) {
    have_windowid_  = windowid_known;
    child_windowid_ = t.windowid;
    if (windowid_known) {
      setenv("WINDOWID", t.windowid.c_str(), 1);
    } else {
      unsetenv("WINDOWID");  // a stale id from the shell that started tmux would be worse
    }
    cols_ = rows_ = 0;
    if (pid_ > 0) {
      StopChild("");
    }
  }
  if (t.client_cols != tmux_.client_cols || t.client_rows != tmux_.client_rows || t.cell_w != tmux_.cell_w || t.cell_h != tmux_.cell_h) {
    cols_ = rows_ = 0;
  }
  tmux_ = t;
}

// tell the real terminal to repaint itself from its own buffer: reverse video
// on and off again in one write changes nothing visible but redraws every
// cell, which wipes pixels w3mimgdisplay painted over text
void PixelOverlay::RepaintTerminal(const std::string& tty) {
  if (Env("FF_TUI_REPAINT") == "0" || tty.empty()) {
    return;
  }
  const int fd = open(tty.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return;
  }
  static const char seq[] = "\033[?5h\033[?5l";
  if (write(fd, seq, sizeof(seq) - 1) < 0) {
    // best effort
  }
  close(fd);
}

void PixelOverlay::HideForTmux() {
  hidden_ = hidden_ || !shown_.empty() || !pending_.empty();
  if (shown_.empty()) {
    return;
  }
  // our pane is gone from the screen; erasing with w3mimgdisplay would hit
  // the other window's text, so let the terminal repaint instead
  shown_.clear();
  RepaintTerminal(tmux_.client_tty);
}

bool PixelOverlay::Poll() {
  if (!in_tmux_ || checked_ != 1 || (shown_.empty() && !hidden_)) {
    return false;
  }
  RefreshTmux(false);
  if (!tmux_.visible) {
    HideForTmux();
    return false;
  }
  if (hidden_) {
    hidden_ = false;
    return true;
  }
  return false;
}

bool PixelOverlay::Start() {
  if (pid_ > 0) {
    return true;
  }
  if (!Available()) {
    return false;
  }

  // frames of instances that were killed (no chance to clean up) are named by their pid
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator("/tmp", ec)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind("ff_tui_pixels.", 0) != 0) {
      continue;
    }
    const int owner = std::atoi(name.c_str() + std::strlen("ff_tui_pixels."));
    if (owner <= 0 || (kill(owner, 0) != 0 && errno == ESRCH)) {
      std::filesystem::remove_all(entry.path(), ec);
    }
  }

  if (dir_.empty()) {
    std::string       tmpl_str = "/tmp/ff_tui_pixels." + std::to_string(getpid()) + ".XXXXXX";
    std::vector<char> tmpl(tmpl_str.begin(), tmpl_str.end());
    tmpl.push_back('\0');
    const char* dir = mkdtemp(tmpl.data());
    if (!dir) {
      reason_  = "cannot create a temp dir";
      checked_ = 0;
      return false;
    }
    dir_ = dir;
  }

  int to[2], from[2];
  if (pipe(to) != 0 || pipe(from) != 0) {
    reason_  = "pipe failed";
    checked_ = 0;
    return false;
  }
  signal(SIGPIPE, SIG_IGN);  // a dead child must not kill the UI

  pid_ = fork();
  if (pid_ == 0) {
    dup2(to[0], STDIN_FILENO);
    dup2(from[1], STDOUT_FILENO);
    const int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      dup2(devnull, STDERR_FILENO);
    }
    close(to[1]);
    close(from[0]);
    execl(binary_.c_str(), binary_.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  close(to[0]);
  close(from[1]);
  if (pid_ < 0) {
    reason_  = "fork failed";
    checked_ = 0;
    return false;
  }
  to_child_   = fdopen(to[1], "w");
  from_child_ = from[0];
  return true;
}

void PixelOverlay::StopChild(const std::string& why) {
  if (pid_ > 0) {
    fclose(to_child_);
    close(from_child_);
    to_child_   = nullptr;
    from_child_ = -1;
    kill(pid_, SIGTERM);
    waitpid(pid_, nullptr, 0);
    pid_ = -1;
  }
  if (!why.empty()) {
    checked_ = 0;  // fall back to block characters from now on
    reason_  = why;
  }
}

void PixelOverlay::Shutdown() {
  if (pid_ > 0) {
    int cw, ch;
    if (CellSize(cw, ch)) {
      for (const auto& item : shown_) {
        Clear(item.box, cw, ch);
      }
    }
    StopChild("");
  }
  shown_.clear();
  for (const auto& f : files_) {
    unlink(f.path.c_str());
  }
  files_.clear();
  if (!dir_.empty()) {
    rmdir(dir_.c_str());
    dir_.clear();
  }
}

// send one command block and wait for the sync reply ("4;\n3;\n" answers with a newline)
bool PixelOverlay::Command(const std::string& cmd) {
  if (pid_ <= 0 || !to_child_) {
    return false;
  }
  if (fputs(cmd.c_str(), to_child_) < 0 || fflush(to_child_) != 0) {
    StopChild("w3mimgdisplay exited");
    return false;
  }
  char c;
  while (true) {
    pollfd p{from_child_, POLLIN, 0};
    if (poll(&p, 1, 500) <= 0) {
      StopChild("w3mimgdisplay stopped answering");
      return false;
    }
    if (read(from_child_, &c, 1) != 1) {
      StopChild("w3mimgdisplay exited");
      return false;
    }
    if (c == '\n') {
      return true;
    }
  }
}

// pixel size of one terminal cell, like ranger: the tty's pixel size if the
// terminal reports it, else the window size from "w3mimgdisplay -test"
bool PixelOverlay::CellSize(int& cell_w, int& cell_h) {
  winsize ws{};
  if (in_tmux_) {
    // our tty is the pane; the geometry that matters is the client terminal's
    if (tmux_.cell_w > 0 && tmux_.cell_h > 0) {
      cell_w = tmux_.cell_w;
      cell_h = tmux_.cell_h;
      return true;
    }
    ws.ws_col = static_cast<unsigned short>(tmux_.client_cols);
    ws.ws_row = static_cast<unsigned short>(tmux_.client_rows);
    if (ws.ws_col == 0 || ws.ws_row == 0) {
      return false;
    }
  } else if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) != 0 || ws.ws_col == 0 || ws.ws_row == 0) {
    return false;
  }
  if (ws.ws_col == cols_ && ws.ws_row == rows_ && cell_w_ > 0) {
    cell_w = cell_w_;
    cell_h = cell_h_;
    return true;
  }

  int xpixels = ws.ws_xpixel, ypixels = ws.ws_ypixel;
  if (xpixels == 0 || ypixels == 0) {
    FILE* p = popen((binary_ + " -test 2>/dev/null").c_str(), "r");
    if (!p) {
      return false;
    }
    if (fscanf(p, "%d %d", &xpixels, &ypixels) != 2) {
      xpixels = ypixels = 0;
    }
    pclose(p);
    xpixels += 2;  // the same border correction ranger applies
    ypixels += 2;
  }
  if (xpixels <= 0 || ypixels <= 0) {
    return false;
  }

  cols_   = ws.ws_col;
  rows_   = ws.ws_row;
  cell_w_ = xpixels / cols_;
  cell_h_ = ypixels / rows_;
  cell_w  = cell_w_;
  cell_h  = cell_h_;
  return cell_w > 0 && cell_h > 0;
}

std::string PixelOverlay::FileFor(const std::shared_ptr<const Image>& image) {
  for (auto it = files_.begin(); it != files_.end(); ++it) {
    if (it->image == image) {
      return it->path;
    }
  }
  const std::string path = dir_ + "/frame_" + std::to_string(file_counter_++) + ".png";
  if (!SavePng(*image, path)) {
    return "";
  }
  files_.insert(files_.begin(), CachedFile{image, path});
  while (files_.size() > 8) {
    unlink(files_.back().path.c_str());
    files_.pop_back();
  }
  return path;
}

void PixelOverlay::Draw(const Item& item, int cell_w, int cell_h) {
  const Image& image = *item.image;
  if (image.w <= 0 || image.h <= 0) {
    return;
  }
  const std::string path = FileFor(item.image);
  if (path.empty()) {
    return;
  }

  const int    offset = std::atoi(Env("FF_TUI_W3M_OFFSET").c_str());
  const int    max_w  = (item.box.x_max - item.box.x_min + 1) * cell_w;
  const int    max_h  = (item.box.y_max - item.box.y_min + 1) * cell_h - 2;
  const double scale  = std::min(static_cast<double>(max_w) / image.w, static_cast<double>(max_h) / image.h);
  const int    w      = std::max(1, static_cast<int>(image.w * scale));
  const int    h      = std::max(1, static_cast<int>(image.h * scale));
  const int    x      = (item.box.x_min + tmux_.origin_col) * cell_w + (max_w - w) / 2 + offset;
  const int    y      = (item.box.y_min + tmux_.origin_row) * cell_h + (max_h - h) / 2 + offset;

  Command("0;1;" + std::to_string(x) + ";" + std::to_string(y) + ";" + std::to_string(w) + ";" + std::to_string(h) + ";;;;;" + path + "\n4;\n3;\n");
}

void PixelOverlay::Clear(const ftxui::Box& box, int cell_w, int cell_h) {
  const int offset = std::atoi(Env("FF_TUI_W3M_OFFSET").c_str());
  const int x      = (box.x_min + tmux_.origin_col) * cell_w + offset;
  const int y      = (box.y_min + tmux_.origin_row) * cell_h + offset;
  const int w      = (box.x_max - box.x_min + 1) * cell_w;
  const int h      = (box.y_max - box.y_min + 1) * cell_h;
  Command("6;" + std::to_string(x) + ";" + std::to_string(y) + ";" + std::to_string(w) + ";" + std::to_string(h) + "\n4;\n3;\n");
}

void PixelOverlay::BeginFrame() {
  pending_.clear();
}

void PixelOverlay::Add(const ftxui::Box& box, std::shared_ptr<const Image> image) {
  pending_.push_back(Item{box, std::move(image)});
}

void PixelOverlay::ClearStale(bool suppressed) {
  if (shown_.empty() || pid_ <= 0) {
    return;
  }
  if (in_tmux_ && !tmux_.visible) {
    return;  // the screen shows another window; Poll/Flush already handed it back
  }
  // pane zoomed / terminal resized: the layout moved and the terminal or tmux
  // has already redrawn text over the old spots, so erasing them now could wipe
  // that text. Drop them and, in tmux, have the terminal repaint itself.
  winsize ws{};
  if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && (ws.ws_col != pane_cols_ || ws.ws_row != pane_rows_) && pane_cols_ != 0) {
    shown_.clear();
    if (in_tmux_) {
      RepaintTerminal(tmux_.client_tty);
    }
    return;
  }
  int cw, ch;
  if (!CellSize(cw, ch)) {
    return;
  }
  std::vector<Item> keep;
  for (const auto& item : shown_) {
    const bool still_used = !suppressed && std::any_of(pending_.begin(), pending_.end(), [&](const Item& p) { return SameBox(p.box, item.box); });
    if (still_used) {
      keep.push_back(item);
    } else {
      Clear(item.box, cw, ch);
    }
  }
  shown_ = std::move(keep);
}

void PixelOverlay::Flush(bool suppressed) {
  if (in_tmux_ && Available()) {
    // a zoomed or resized pane moves on screen: never paint with stale geometry
    winsize ws{};
    bool    resized = false;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && (ws.ws_col != pane_cols_ || ws.ws_row != pane_rows_)) {
      pane_cols_ = ws.ws_col;
      pane_rows_ = ws.ws_row;
      resized    = true;
    }
    RefreshTmux(resized);
    if (!tmux_.visible) {
      HideForTmux();
      return;  // painting now would land on whatever window is shown instead
    }
  }
  if (!in_tmux_) {
    winsize ws{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) {
      pane_cols_ = ws.ws_col;
      pane_rows_ = ws.ws_row;
    }
  }
  if (suppressed || pending_.empty() || !Start()) {
    return;
  }
  int cw, ch;
  if (!CellSize(cw, ch)) {
    return;
  }
  for (const auto& item : pending_) {
    Draw(item, cw, ch);
    if (pid_ <= 0) {
      return;  // the helper died; the next frame falls back to blocks
    }
  }
  shown_  = pending_;
  hidden_ = false;
}
