// FlightForge TUI: one interactive terminal front-end for the whole connector
// API (game mode + drone), with in-terminal camera and lidar previews and a
// teleport drive mode. See README.md for the key map.

#include <atomic>
#include <cmath>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/terminal.hpp>

#include "app.h"
#include "drive_mode.h"
#include "pixel_overlay.h"
#include "viz.h"

using namespace ftxui;

namespace
{

struct Page
{
  std::string              name;
  ActionList               actions;
  std::vector<std::string> entries;
  std::vector<int>         entry_action;  // -1 = category header
  int                      selected      = 1;
  int                      focused_entry = 1;
  int                      previous      = 1;
  Component                menu;
  Component                form_holder;
  Component                root;

  Action* Current() {
    if (selected < 0 || selected >= static_cast<int>(entry_action.size()) || entry_action[selected] < 0) {
      return nullptr;
    }
    return &actions[entry_action[selected]];
  }
};

struct Ui
{
  int       tab          = 0;
  int       previous_tab = 0;
  bool      show_help    = false;
  bool      show_picker  = false;
  int       scroll       = 0;
  uint64_t  seen_seq     = 0;
  ViewSettings view;
  Action*   last_action  = nullptr;

  std::vector<std::string> picker_entries;
  std::vector<int>         picker_ports;
  int                      picker_selected = 0;
  std::string              picker_port;
};

void BuildEntries(Page& page) {
  std::vector<std::string> categories;
  for (const auto& a : page.actions) {
    if (std::find(categories.begin(), categories.end(), a.category) == categories.end()) {
      categories.push_back(a.category);
    }
  }
  for (const auto& category : categories) {
    page.entries.push_back("▾ " + category);
    page.entry_action.push_back(-1);
    for (size_t i = 0; i < page.actions.size(); ++i) {
      if (page.actions[i].category == category) {
        page.entries.push_back("  " + page.actions[i].name);
        page.entry_action.push_back(static_cast<int>(i));
      }
    }
  }
}

std::string Timestamp() {
  const auto now = std::time(nullptr);
  std::tm    tm{};
  localtime_r(&now, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
  return buf;
}

void SavePreview(App& app, const Preview& preview, const std::string& what) {
  if (preview.empty()) {
    app.Log(false, "save: nothing to save, run a sensor call first");
    return;
  }
  std::string base = "ff_" + what + "_" + Timestamp();
  for (auto& ch : base) {
    if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_' && ch != '-') {
      ch = '_';
    }
  }
  for (size_t i = 0; i < preview.images.size(); ++i) {
    const std::string path = base + (preview.images.size() > 1 ? "_" + preview.images[i]->title : "") + ".png";
    const bool        ok   = SavePng(*preview.images[i], path);
    app.Log(ok, ok ? "saved " + path : "could not write " + path);
  }
  if (preview.lidar) {
    const std::string path = base + ".ply";
    const bool        ok   = SavePly(*preview.lidar, path);
    app.Log(ok, ok ? "saved " + path + " (" + std::to_string(preview.lidar->points.size()) + " points)" : "could not write " + path);
  }
  if (preview.range && preview.images.empty() && !preview.lidar) {
    app.Log(true, "range " + Num(*preview.range, 2) + " (nothing to write for a single value)");
  }
}

Element HelpBox() {
  auto key = [](const std::string& k, const std::string& what) {
    return hbox({text(k) | bold | color(Color::Cyan) | size(WIDTH, EQUAL, 20), text(what)});
  };
  return window(text(" FlightForge TUI — keys ") | bold,
                vbox({
                    text("Navigation") | bold | color(Color::Yellow),
                    key("F2–F6 / 1–5", "switch tab (1–5 only while the menu has focus)"),
                    key("↑ ↓  Enter", "pick an action; Enter runs it or jumps into its form"),
                    key("→  Esc", "into the form / back to the menu"),
                    key("Tab ↑ ↓", "move between form fields; ←/→ cycle ◂choices▸"),
                    key("Enter (in a field)", "execute the action"),
                    key("PgUp / PgDn", "scroll the result text"),
                    text(""),
                    text("Actions (menu focused)  / anywhere") | bold | color(Color::Yellow),
                    key("r", "repeat the last executed call"),
                    key("l  / F9", "live mode: re-run the selected sensor call continuously"),
                    key("s  / F8", "save the shown image(s) as PNG, lidar as PLY (cwd)"),
                    key("d  / F7", "pick the active drone"),
                    key("v", "lidar view: top / side / range image"),
                    key("+ / -", "lidar zoom"),
                    key("f  / F11", "maximize the preview (Esc or f again to restore)"),
                    key("m  / F12", "image mode: pixels (w3m, full res) / quadrant 2×2 / half-block 1×2"),
                    key("?  / F1", "this help"),
                    key("q  / F10", "quit"),
                    text(""),
                    text("Drive tab (F6)") | bold | color(Color::Yellow),
                    text("W/S fwd/back  A/D left/right  R/F up/down  Q/E yaw  I/K pitch  J/L roll  0 level"),
                    text("+/- pos step  [/] rot step  B body/world  C collisions  Y async  V view  Z/X zoom  M image mode  F11 max  Space sync"),
                    text(""),
                    text("Any key closes this window.") | dim,
                })) |
         size(WIDTH, LESS_THAN, 100) | clear_under | center;
}

Element RenderLog(App& app, int lines) {
  std::vector<LogLine> tail;
  {
    std::lock_guard<std::mutex> lock(app.mutex);
    const int start = std::max(0, static_cast<int>(app.log.size()) - lines);
    tail.assign(app.log.begin() + start, app.log.end());
  }
  Elements rows;
  for (const auto& l : tail) {
    rows.push_back(hbox({text(l.time + " ") | dim, text(l.ok ? "✓ " : "✗ ") | color(l.ok ? Color::Green : Color::Red), text(l.text)}));
  }
  while (static_cast<int>(rows.size()) < lines) {
    rows.insert(rows.begin(), text(""));
  }
  return vbox(rows);
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string address    = LOCALHOST;
  int         gm_port    = 8551;
  int         drone_port = -1;
  std::string log_path   = "/tmp/ff_tui.log";

  for (int i = 1; i < argc; ++i) {
    const std::string arg  = argv[i];
    auto              next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << what << std::endl;
        std::exit(1);
      }
      return argv[++i];
    };
    if (arg == "--address" || arg == "-a") {
      address = next("--address");
    } else if (arg == "--gm-port" || arg == "-p") {
      gm_port = std::stoi(next("--gm-port"));
    } else if (arg == "--drone-port" || arg == "-d") {
      drone_port = std::stoi(next("--drone-port"));
    } else if (arg == "--log") {
      log_path = next("--log");
    } else {
      std::cout << "Usage: ff_tui [--address IP] [--gm-port 8551] [--drone-port PORT] [--log /tmp/ff_tui.log]\n";
      return arg == "--help" || arg == "-h" ? 0 : 1;
    }
  }

  // the library reports serialization errors on std::cerr, which would tear the UI
  std::ofstream log_file(log_path, std::ios::app);
  auto*         old_cerr = std::cerr.rdbuf();
  if (log_file) {
    std::cerr.rdbuf(log_file.rdbuf());
  }

  auto screen = ScreenInteractive::Fullscreen();

  App app(address, gm_port);
  app.refresh = [&screen] { screen.PostEvent(Event::Custom); };
  if (drone_port >= 0) {
    app.SetActiveDrone(drone_port);
  }

  Ui ui;
  if (PixelOverlay::Get().Available()) {
    ui.view.image_mode = ImageMode::Pixels;
    app.Log(true, "camera images in full resolution via w3mimgdisplay ('m' switches to block characters)");
  } else {
    app.Log(true, "full-resolution images unavailable: " + PixelOverlay::Get().Reason() + "; using block characters");
  }

  std::vector<Page> pages(4);
  pages[0].name    = "Game Mode";
  pages[0].actions = GameModeActions(app);
  pages[1].name    = "Drone";
  pages[1].actions = DroneActions(app);
  pages[2].name    = "Sensors";
  pages[2].actions = SensorActions(app);
  pages[3].name    = "Scene";
  pages[3].actions = SceneActions(app);

  auto rebuild_form = [&](Page& page) {
    page.form_holder->DetachAllChildren();
    Action* a = page.Current();
    if (!a) {
      page.form_holder->Add(Renderer([] { return text(""); }));
      return;
    }
    PullAll(a->rows);
    FormCallbacks callbacks;
    callbacks.execute = [&app, &ui, a] {
      ui.last_action = a;
      app.Execute(*a);
    };
    if (a->load) {
      callbacks.load = [&app, a] { app.Execute(*a, true); };
    }
    callbacks.back = [&page] { page.menu->TakeFocus(); };
    page.form_holder->Add(BuildForm(a->rows, callbacks));
  };

  for (auto& page : pages) {
    BuildEntries(page);
    Page* pg = &page;

    MenuOption option              = MenuOption::Vertical();
    option.entries                 = &page.entries;
    option.selected                = &page.selected;
    option.focused_entry           = &page.focused_entry;
    option.entries_option.transform = [](const EntryState& s) {
      Element e = text(s.label);
      if (s.label.rfind("▾", 0) == 0) {
        return e | bold | color(Color::Yellow);
      }
      if (s.focused) {
        return e | inverted;
      }
      if (s.active) {
        return e | bold | color(Color::Cyan);
      }
      return e;
    };
    option.on_change = [pg, &rebuild_form] {
      // headers are not selectable: keep moving in the direction of travel
      const int n   = static_cast<int>(pg->entries.size());
      const int dir = pg->selected >= pg->previous ? 1 : -1;
      while (pg->selected >= 0 && pg->selected < n && pg->entry_action[pg->selected] < 0) {
        pg->selected += dir;
      }
      if (pg->selected < 0 || pg->selected >= n) {
        pg->selected = pg->previous;
      }
      pg->focused_entry = pg->selected;
      pg->previous      = pg->selected;
      rebuild_form(*pg);
    };
    option.on_enter = [pg, &app, &ui] {
      Action* a = pg->Current();
      if (!a) {
        return;
      }
      if (a->rows.empty()) {
        ui.last_action = a;
        app.Execute(*a);
      } else {
        pg->form_holder->TakeFocus();
      }
    };

    page.menu        = Menu(option);
    page.form_holder = Container::Vertical({});
    page.root        = Container::Horizontal({page.menu, page.form_holder});
    rebuild_form(page);
  }

  auto drive = DriveComponent(app, &ui.view);

  Components tabs;
  for (auto& page : pages) {
    tabs.push_back(page.root);
  }
  tabs.push_back(drive);
  auto tab_container = Container::Tab(tabs, &ui.tab);

  const std::vector<std::string> tab_names = {"Game Mode", "Drone", "Sensors", "Scene", "Drive"};

  auto switch_tab = [&](int tab) {
    if (tab == ui.tab) {
      return;
    }
    ui.previous_tab = ui.tab;
    ui.tab          = tab;
    if (tab == 4) {
      drive->TakeFocus();
      DriveOnEnter(app);
    } else {
      pages[tab].menu->TakeFocus();
    }
  };

  auto toggle_live = [&] {
    std::lock_guard<std::mutex> lock(app.mutex);
    if (app.live_action) {
      app.live_action = nullptr;
      return;
    }
    if (ui.tab < 4) {
      Action* a = pages[ui.tab].Current();
      if (a && a->sensor) {
        app.live_action = a;
      }
    }
  };

  Component picker_menu;  // created with the modals below
  auto      open_picker = [&] {
    ui.picker_entries.clear();
    ui.picker_ports.clear();
    {
      std::lock_guard<std::mutex> lock(app.mutex);
      for (int port : app.drone_ports) {
        ui.picker_entries.push_back(":" + std::to_string(port) + (port == app.ActiveDrone() ? "  (active)" : ""));
        ui.picker_ports.push_back(port);
      }
    }
    if (ui.picker_entries.empty()) {
      ui.picker_entries.push_back("(no drones reported — type a port below)");
    }
    ui.picker_selected = 0;
    ui.picker_port.clear();
    ui.show_picker = true;
    if (picker_menu) {
      picker_menu->TakeFocus();  // start on the list, not on the port field used last time
    }
  };

  auto save_current = [&] {
    Preview     preview;
    std::string what;
    {
      std::lock_guard<std::mutex> lock(app.mutex);
      if (ui.tab == 4) {
        preview = app.drive.preview;
        what    = "drive";
      } else {
        preview = app.result.preview;
        what    = app.result.action;
      }
    }
    SavePreview(app, preview, what);
  };

  // maximize only takes effect when there is something to maximize
  auto maximized_active = [&] {
    if (!ui.view.maximized) {
      return false;
    }
    if (ui.tab == 4) {
      return true;
    }
    std::lock_guard<std::mutex> lock(app.mutex);
    return !app.result.preview.empty();
  };

  /* main renderer //{ */

  auto render_page = [&](Page& page) {
    Element menu_el = page.menu->Render() | vscroll_indicator | frame | size(WIDTH, EQUAL, 34);

    Action* a = page.Current();
    Element right;
    if (!a) {
      right = text("select an action") | dim | center;
    } else {
      if (a->needs_pull->exchange(false)) {
        PullAll(a->rows);
      }
      Action* live;
      {
        std::lock_guard<std::mutex> lock(app.mutex);
        live = app.live_action;
      }
      const std::string target = a->target == Target::GameMode ? "game mode" : "drone " + (app.ActiveDrone() >= 0 ? ":" + std::to_string(app.ActiveDrone()) : "(none)");
      Elements title_parts = {text(a->name) | bold, text("  → " + target) | dim};
      if (a->running->load()) {
        title_parts.push_back(text("  ⟳ running") | color(Color::Yellow));
      }
      if (live == a) {
        title_parts.push_back(text("  ● LIVE") | color(Color::Red) | bold);
      } else if (a->sensor) {
        title_parts.push_back(text("  [l] live") | dim);
      }

      int needed = 2;  // spacer + buttons
      for (const auto& row : a->rows) {
        needed += 1;
        (void)row;
      }
      const int max_h  = std::max(5, Terminal::Size().dimy * 2 / 5);
      Element   form   = page.form_holder->Render() | vscroll_indicator | frame | size(HEIGHT, EQUAL, std::min(needed, max_h));

      // result
      Result result;
      {
        std::lock_guard<std::mutex> lock(app.mutex);
        result = app.result;
        if (app.result_seq != ui.seen_seq) {
          ui.seen_seq = app.result_seq;
          ui.scroll   = 0;
        }
      }
      Element result_el;
      if (result.action.empty()) {
        result_el = text("no call yet") | dim;
      } else {
        Elements lines;
        lines.push_back(hbox({text(result.ok ? "✓ " : "✗ ") | color(result.ok ? Color::Green : Color::Red) | bold, text(result.action) | bold,
                              text("  " + Num(result.ms, 1) + " ms") | dim}));
        ui.scroll = std::clamp(ui.scroll, 0, std::max(0, static_cast<int>(result.lines.size()) - 1));
        for (size_t i = ui.scroll; i < result.lines.size(); ++i) {
          lines.push_back(text(result.lines[i]));
        }
        Element text_block = vbox(lines);
        if (ui.view.maximized && !result.preview.empty()) {
          return vbox({lines.front(), PreviewElement(result.preview, ui.view.lidar_view, ui.view.zoom, std::nan(""), ui.view.image_mode) | flex});
        }
        if (result.preview.empty()) {
          result_el = text_block;
        } else {
          result_el = hbox({text_block | size(WIDTH, LESS_THAN, 48), separator(), PreviewElement(result.preview, ui.view.lidar_view, ui.view.zoom, std::nan(""), ui.view.image_mode) | flex});
        }
      }

      right = vbox({hbox(title_parts), paragraph(a->help) | dim, separator(), form, separator(), result_el | flex});
    }
    return hbox({menu_el, separator(), right | flex});
  };

  auto main_renderer = Renderer(tab_container, [&] {
    PixelOverlay::Get().BeginFrame();
    // header
    bool        connected, api_ok;
    std::string status, drone_status;
    int         major, minor;
    float       fps;
    size_t      drone_count;
    Action*     live;
    {
      std::lock_guard<std::mutex> lock(app.mutex);
      connected    = app.gm_connected;
      api_ok       = app.api_ok;
      status       = app.gm_status;
      major        = app.api_major;
      minor        = app.api_minor;
      fps          = app.fps;
      drone_count  = app.drone_ports.size();
      drone_status = app.drone_status;
      live         = app.live_action;
    }
    const int active = app.ActiveDrone();

    Elements header = {
        text(" FlightForge TUI ") | bold | inverted,
        text(" GM " + app.address + ":" + std::to_string(app.gm_port) + " "),
        text(connected ? "● " : "○ ") | color(connected ? Color::Green : Color::Red),
        text(status) | (connected ? nothing : color(Color::Red)),
        text(" │ "),
        connected ? (text("API v" + std::to_string(major) + "." + std::to_string(minor) + (api_ok ? " ✓" : " ✗ mismatch")) |
                     color(api_ok ? Color::Green : Color::Red))
                  : text("API ?") | dim,
        text(" │ drone "),
        text(active >= 0 ? ":" + std::to_string(active) : "none") | bold | color(active >= 0 ? Color::Cyan : Color::GrayLight),
        text(" (" + std::to_string(drone_count) + " in world" + (drone_status.empty() ? "" : ", " + drone_status) + ") [d]") | dim,
        text(" │ FPS " + (connected ? Num(fps, 0) : std::string("-"))),
    };
    if (live) {
      header.push_back(text(" │ ") );
      header.push_back(text("● LIVE " + live->name) | color(Color::Red) | bold);
    }
    header.push_back(filler());
    header.push_back(text("F1 help ") | dim);

    Elements tab_bar;
    for (int i = 0; i < static_cast<int>(tab_names.size()); ++i) {
      auto t = text(" F" + std::to_string(i + 2) + " " + tab_names[i] + " ");
      tab_bar.push_back(i == ui.tab ? t | inverted | bold : t | dim);
      tab_bar.push_back(text(" "));
    }

    Element body = ui.tab < 4 ? render_page(pages[ui.tab]) : drive->Render();

    Element document;
    if (maximized_active()) {
      document = vbox({hbox(header), body | flex});
    } else {
      document = vbox({
          hbox(header),
          hbox(tab_bar),
          separator(),
          body | flex,
          separator(),
          RenderLog(app, 6),
      });
    }

    // pixel images: erase stale ones before this frame is printed (sentinel
    // renders last), paint the current ones right after it is on screen
    const bool dialog_open = ui.show_help || ui.show_picker;
    screen.Post([dialog_open] { PixelOverlay::Get().Flush(dialog_open); });
    return dbox({document, PixelOverlaySentinel(dialog_open)});
  });

  //}

  /* global keys //{ */

  auto with_keys = CatchEvent(main_renderer, [&](Event e) {
    if (e == Event::Custom) {
      return false;
    }
    const bool in_drive   = ui.tab == 4;
    const bool maximized_page = !in_drive && maximized_active();
    const bool menu_focus     = !in_drive && (pages[ui.tab].menu->Focused() || maximized_page);

    if (e == Event::F11 || (menu_focus && e == Event::Character('f')) || (maximized_active() && e == Event::Escape)) {
      ui.view.maximized = !ui.view.maximized;
      return true;
    }
    if (e == Event::F12 || ((menu_focus || in_drive) && e == Event::Character('m'))) {
      // pixels -> quadrant -> half-block -> pixels (pixels only when w3m works)
      switch (ui.view.image_mode) {
        case ImageMode::Pixels:
          ui.view.image_mode = ImageMode::Quadrant;
          break;
        case ImageMode::Quadrant:
          ui.view.image_mode = ImageMode::HalfBlock;
          break;
        case ImageMode::HalfBlock:
          ui.view.image_mode = PixelOverlay::Get().Available() ? ImageMode::Pixels : ImageMode::Quadrant;
          break;
      }
      return true;
    }

    if (e == Event::F1 || (menu_focus && e == Event::Character('?'))) {
      ui.show_help = true;
      return true;
    }
    const Event fkeys[] = {Event::F2, Event::F3, Event::F4, Event::F5, Event::F6};
    for (int i = 0; i < 5; ++i) {
      if (e == fkeys[i] || (menu_focus && e == Event::Character(static_cast<char>('1' + i)))) {
        switch_tab(i);
        return true;
      }
    }
    if (e == Event::F7 || (menu_focus && e == Event::Character('d'))) {
      open_picker();
      return true;
    }
    if (e == Event::F8 || (menu_focus && e == Event::Character('s'))) {
      save_current();
      return true;
    }
    if (e == Event::F9 || (menu_focus && e == Event::Character('l'))) {
      toggle_live();
      return true;
    }
    if (e == Event::F10 || (menu_focus && e == Event::Character('q'))) {
      screen.Exit();
      return true;
    }
    if (in_drive) {
      if (e == Event::Escape) {
        switch_tab(ui.previous_tab == 4 ? 0 : ui.previous_tab);
        return true;
      }
      return false;
    }
    if (e == Event::PageUp) {
      ui.scroll = std::max(0, ui.scroll - 10);
      return true;
    }
    if (e == Event::PageDown) {
      ui.scroll += 10;
      return true;
    }
    if (menu_focus) {
      if (e == Event::Character('r')) {
        if (ui.last_action) {
          app.Execute(*ui.last_action);
        } else {
          app.Log(false, "nothing to repeat yet");
        }
        return true;
      }
      if (e == Event::Character('v')) {
        ui.view.lidar_view = ui.view.lidar_view == LidarView::Top ? LidarView::Side : ui.view.lidar_view == LidarView::Side ? LidarView::Range : LidarView::Top;
        return true;
      }
      if (e == Event::Character('+') || e == Event::Character('=')) {
        ui.view.zoom = std::min(ui.view.zoom * 1.5, 50.0);
        return true;
      }
      if (e == Event::Character('-')) {
        ui.view.zoom = std::max(ui.view.zoom / 1.5, 0.05);
        return true;
      }
    }
    // the menu and form are hidden: do not let keys change them unseen
    return maximized_page;
  });

  //}

  /* modals //{ */

  MenuOption picker_option = MenuOption::Vertical();
  picker_option.on_enter   = [&] {
    if (ui.picker_selected < static_cast<int>(ui.picker_ports.size())) {
      app.SetActiveDrone(ui.picker_ports[ui.picker_selected]);
      ui.show_picker = false;
    }
  };
  picker_menu       = Menu(&ui.picker_entries, &ui.picker_selected, picker_option);

  InputOption port_option;
  port_option.multiline = false;
  port_option.on_enter  = [&] {
    try {
      const int port = std::stoi(ui.picker_port);
      app.SetActiveDrone(port);
      ui.show_picker = false;
    }
    catch (const std::exception&) {
      app.Log(false, "'" + ui.picker_port + "' is not a port");
    }
  };
  auto picker_input = Input(&ui.picker_port, "type a port, Enter", port_option);
  auto picker       = Renderer(Container::Vertical({picker_menu, picker_input}), [&] {
    return window(text(" Active drone ") | bold, vbox({picker_menu->Render() | vscroll_indicator | frame | size(HEIGHT, LESS_THAN, 12), separator(),
                                                       hbox({text("port: "), picker_input->Render() | size(WIDTH, EQUAL, 26)}),
                                                       text("Enter select · Esc close") | dim})) |
           size(WIDTH, EQUAL, 46);
  });
  picker = CatchEvent(picker, [&](Event e) {
    if (e == Event::Escape) {
      ui.show_picker = false;
      return true;
    }
    return false;
  });

  auto help = CatchEvent(Renderer([](bool /*focused*/) { return HelpBox(); }), [&](Event e) {
    if (e == Event::Custom || e.is_mouse()) {
      return false;
    }
    ui.show_help = false;
    return true;
  });

  auto root = Modal(with_keys, picker, &ui.show_picker);
  root      = Modal(root, help, &ui.show_help);

  //}

  std::atomic<bool> running{true};
  std::thread       ticker([&] {
    const auto start = std::chrono::steady_clock::now();
    while (running.load()) {
      const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
      app.Tick(now);
      // inside tmux: drop the images promptly when the pane is switched away,
      // and bring them back when it returns
      screen.Post([&app] {
        if (PixelOverlay::Get().Poll()) {
          app.Refresh();
        }
      });
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  pages[0].menu->TakeFocus();
  screen.Loop(root);

  running.store(false);
  ticker.join();
  {
    std::lock_guard<std::mutex> lock(app.mutex);
    app.live_action = nullptr;
  }
  app.refresh = nullptr;
  app.Shutdown();
  PixelOverlay::Get().Shutdown();

  std::cerr.rdbuf(old_cerr);
  return 0;
}
