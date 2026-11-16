#pragma once

#include <sys/types.h>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <ftxui/screen/box.hpp>

#include "app.h"

// Full-resolution images through w3mimgdisplay, the way ranger's "w3m"
// preview method does it: the image area is left blank in the text UI and
// w3mimgdisplay paints the picture straight onto the terminal's X11 window
// right after each frame has been written. Works in urxvt, xterm and similar
// X terminals, also inside tmux (pane offset, the attached client's window
// and cell size come from tmux; nothing is painted while the pane is hidden).
// Not over ssh, in GNU screen, or on Wayland-only terminals.
//
// UI thread only.
class PixelOverlay {
public:
  static PixelOverlay& Get();

  // binary present, X display, no multiplexer; decided once (FF_TUI_PIXELS=0/1 overrides)
  bool Available();

  // why it is unavailable, for the log
  const std::string& Reason() const {
    return reason_;
  }

  // start of a frame: forget last frame's image areas
  void BeginFrame();

  // a pixel-mode image element painted at this cell box
  void Add(const ftxui::Box& box, std::shared_ptr<const Image> image);

  // after all elements are painted but before the frame is printed: erase
  // image areas that are no longer used (everything when suppressed, i.e. a
  // dialog is open). Erasing before printing lets new text land on top.
  void ClearStale(bool suppressed);

  // after the frame is on screen: paint this frame's images
  void Flush(bool suppressed);

  // periodic (a few Hz, UI thread): inside tmux, take the images away as soon
  // as the pane stops being visible (another window selected, detached).
  // Returns true when the pane is back and the images should be repainted.
  bool Poll();

  void Shutdown();

private:
  PixelOverlay() = default;
  ~PixelOverlay();

  struct Item
  {
    ftxui::Box                   box;
    std::shared_ptr<const Image> image;
  };

  struct CachedFile
  {
    std::shared_ptr<const Image> image;
    std::string                  path;
  };

  struct TmuxInfo
  {
    bool        visible    = true;
    int         origin_col = 0;  // pane position on the client terminal, in cells
    int         origin_row = 0;
    int         client_cols = 0, client_rows = 0;
    int         cell_w = 0, cell_h = 0;  // 0 when tmux does not know the client's cell size
    std::string client_tty;
    std::string windowid;  // the attached terminal's X window, from the session environment
  };

  void        RefreshTmux(bool force);
  void        HideForTmux();
  void        RepaintTerminal(const std::string& tty);
  bool        Start();
  void        StopChild(const std::string& why);
  bool        Command(const std::string& cmd);
  bool        CellSize(int& cell_w, int& cell_h);
  std::string FileFor(const std::shared_ptr<const Image>& image);
  void        Draw(const Item& item, int cell_w, int cell_h);
  void        Clear(const ftxui::Box& box, int cell_w, int cell_h);

  int         checked_ = -1;  // -1 not yet, 0 no, 1 yes
  std::string reason_;
  std::string binary_;

  pid_t pid_      = -1;
  FILE* to_child_ = nullptr;
  int   from_child_ = -1;

  // cell size cache, keyed by the terminal size it was measured for
  int cols_ = 0, rows_ = 0, cell_w_ = 0, cell_h_ = 0;

  std::string             dir_;
  int                     file_counter_ = 0;
  std::vector<CachedFile> files_;  // most recent first

  bool     in_tmux_         = false;
  bool     hidden_          = false;  // images withheld because the pane is not on screen
  int      pane_cols_ = 0, pane_rows_ = 0;  // our own tty size, to notice zoom/resize at once
  TmuxInfo tmux_;
  long     tmux_checked_ms_ = -1000000;
  bool     have_windowid_   = false;
  std::string child_windowid_;

  std::vector<Item> pending_;
  std::vector<Item> shown_;
};
