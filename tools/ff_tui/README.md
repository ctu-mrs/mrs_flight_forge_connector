# ff_tui: FlightForge terminal test bench

`ff_tui` replaces `debug_cli` and `debug_game_mode_cli` with one interactive terminal app. It can:
- call every public method of `GameModeController` and `UedsConnector` through arrow-key menus and forms;
- preview camera, depth, segmentation, event and lidar data inside the terminal;
- teleport-drive the drone from the keyboard.

It lives entirely in this folder and compiles the connector sources itself, so nothing outside `tools/ff_tui/` is modified.

## Build and run

```bash
./build_tui.sh                              # build or rebuild (clean: wipe first)
./build_tui.sh run                          # rebuild and start with truecolor
# or by hand: cmake -S tools/ff_tui -B build/ff_tui && cmake --build build/ff_tui -j
./build/ff_tui/ff_tui                       # game mode on 127.0.0.1:8551
./build/ff_tui/ff_tui -a 10.0.0.5 -p 8551   # remote simulator
./build/ff_tui/ff_tui -d 4000               # start with drone :4000 active
```

FTXUI v5.0.0 is downloaded by CMake on the first configure. `stb_image.h` is vendored in `third_party/`.

The app connects and reconnects to the game mode by itself and checks the API version. It then picks the first drone it finds as the active one. The library's stderr output goes to `/tmp/ff_tui.log` (change it with `--log`) so it doesn't break the screen.

**Colours:** images use 24-bit colour when the terminal says it supports it (`COLORTERM=truecolor`). Otherwise FTXUI falls back to 256 colours, which look noticeably worse. If your terminal handles truecolor but doesn't set the variable (common inside tmux), start it with `COLORTERM=truecolor ./build/ff_tui/ff_tui`.

## Layout

```
┌ header: connection ● · API version check · active drone [d] · FPS · LIVE indicator ───────┐
│ F2 Game Mode  F3 Drone  F4 Sensors  F5 Scene  F6 Drive                                    │
├────────────────────┬──────────────────────────────────────────────────────────────────────┤
│ ▾ Category         │ Action name → target      description                                │
│   Action           │ parameter form (pre-filled with the struct defaults)                 │
│   Action           │ [ Execute ⏎ ]  [ Load current ]                                      │
│ ▾ Category         ├───────────────────────────┬──────────────────────────────────────────┤
│   ...              │ result text (PgUp/PgDn)   │ image / lidar preview                    │
├────────────────────┴───────────────────────────┴──────────────────────────────────────────┤
│ log: every call with ✓/✗, first result line and latency                                   │
└───────────────────────────────────────────────────────────────────────────────────────────┘
```

## Keys

| Where | Key | Action |
|---|---|---|
| anywhere | F2–F6 | switch tab |
| anywhere | F1 | help |
| anywhere | F7 | choose the active drone (listed from `GetDrones`, or type a port) |
| anywhere | F8 | save the current preview: PNG for images, PLY for lidar, into the working directory |
| anywhere | F9 | live mode on/off |
| anywhere | F10 | quit |
| anywhere | F11 | maximise the preview to the whole screen; F11, `f` or Esc restores it |
| anywhere | F12 | image mode: pixels (w3m) → quadrant 2×2 → half-block 1×2 |
| menu | ↑ ↓ | choose an action (category headers are skipped) |
| menu | Enter | run the action if it takes no parameters, otherwise jump into its form |
| menu | → | jump into the form |
| menu | 1–5 | switch tab |
| menu | `d` `s` `l` `q` `?` `f` `m` | same as F7 / F8 / F9 / F10 / F1 / F11 / F12 |
| menu | `r` | repeat the last executed call |
| menu | `v` | lidar view: top (X↑ Y→), side (X→ Z↑), range image |
| menu | `+` / `-` | lidar zoom |
| form | Tab / ↑ ↓ | move between fields |
| form | ←/→ on `◂ choice ▸` | cycle the options |
| form | Space or Enter on `[x]` | toggle |
| form | Enter in a text field | execute |
| form | Esc | back to the menu |
| results | PgUp / PgDn | scroll long output (ExportScene, ListAssets, GetClassMap, ...) |

**Live mode:** select a sensor "Get…Data" action and press `l` (or F9). The call then repeats as fast as the drone answers. The header shows `● LIVE <action>`. Press `l` again to stop.

**"Load current":** forms that set something (pose, sensor configs, capture mode, …) have this button. It reads the current value from the simulator into the form, so you can change one field and send the rest back unchanged. The `Get…Config` actions fill the matching `Set…Config` form too.

## Drive tab (F6)

Teleport control with no physics. On entry the app reads the drone's pose once. After that it keeps the pose locally, and each key press adds a fixed step and sends it with `SetLocationAndRotation`. If key presses arrive faster than the simulator answers, only the newest pose is sent.

| Key | Action |
|---|---|
| W/S or ↑/↓ | forward / back |
| A/D | left / right |
| R/F or PgUp/PgDn | up / down |
| Q/E or ←/→ | yaw |
| I/K | pitch |
| J/L | roll |
| 0 | level (pitch and roll back to 0) |
| `+`/`-` | position step ×2 / ÷2 (cm) |
| `[`/`]` | rotation step ÷2 / ×2 (deg) |
| B | BODY frame (steps follow the yaw) or WORLD frame (steps follow the axes) |
| C | collision check on/off |
| Y | send with `SetLocationAndRotationAsync` |
| V | preview after each move: camera, lidar (top view with a heading arrow) or off |
| M / F11 | image mode / maximise the preview |
| Z/X | lidar zoom |
| Space | re-sync the pose from the simulator and refresh the preview |
| Esc | back to the previous tab |

Units are the simulator's: cm and degrees, Unreal axes (X forward, Y right, Z up, yaw clockwise seen from above). If a move hits something, the drone continues from where the simulator stopped it.

## Visualisation notes

- **Full-resolution images (pixels mode, the default when available):** this works like ranger's `w3m` preview method. The image area is left blank, the frame is written to a temporary PNG, and `/usr/lib/w3m/w3mimgdisplay` paints it at full resolution straight onto the terminal's X11 window after every frame. It needs X11, the `w3m-img` package and a terminal such as urxvt or xterm. It turns itself off over ssh, in GNU screen, or when the helper is missing; the log says why, and the images fall back to block characters.
  - **Inside tmux** it works too. The pane's position, the status bar, the attached client's cell size and its X window (`WINDOWID` from the tmux session environment) are read from tmux, so split, zoomed and re-attached panes line up. While the pane isn't on screen (another tmux window selected, or the session detached), nothing is painted. The image is taken away within about 0.2 s, with a terminal repaint (reverse video on and off, sent to the client tty) so it doesn't stay over the other window, and it comes back when you return. `FF_TUI_REPAINT=0` turns the repaint off.
  - Like ranger, it takes the cell size from the terminal (`TIOCGWINSZ`), or from `w3mimgdisplay -test` when the terminal doesn't report one.
  - If the picture sits a few pixels off, set `FF_TUI_W3M_OFFSET=<px>` (the equivalent of ranger's `w3m_offset`). `FF_TUI_PIXELS=0` turns the mode off, `FF_TUI_PIXELS=1` forces it on, and `FF_TUI_W3MIMGDISPLAY=<path>` points to a different helper binary.
  - The image is hidden while the help or drone picker is open, because it would cover them. Temporary frames go to `/tmp/ff_tui_pixels.<pid>.*` and are deleted on exit.
- **Block-character images** are box-filtered from the frame, keep their aspect ratio, and work in any terminal, including over ssh and in tmux. There are two modes; `m` switches between them:
  - **quadrant** (default): each cell is split into 2×2 sub-pixels using the quadrant block characters (`▘▝▖▗▚▞▌▐▀▄▙▛▜▟█`). For each cell the renderer chooses the split and the two colours that best match the four sub-pixels. This doubles the horizontal resolution and sharpens edges and thin lines. The trade-off is that each cell can only show two colours.
  - **half-block**: `▀` with the top pixel as the foreground and the bottom pixel as the background. That gives 1×2 pixels per cell with smoother colour and less detail.
- **More resolution:** press `f` (F11) to give the preview the whole screen. A smaller terminal font gives more cells too (often Ctrl+- in the terminal). Use F8 when you need the full-resolution frame.
- **Depth:** turbo colour map with red as near. The range runs from the 1st to the 98th percentile of valid pixels; black means invalid, grey means clamped.
- **Instance segmentation:** every id gets its own stable false colour. **Event camera:** red means brighter, blue means darker, accumulated since the last call.
- **Lidar:** drawn as braille dots (2×4 per cell). Points are coloured by distance, or by class/intensity in the Seg/Int/Scan variants. The view scales to the 95th-percentile range, with rings at ¼, ½ and the full range.

## Files

| File | Contents |
|---|---|
| `main.cpp` | layout, tabs, global keys, modals |
| `app.h/.cpp` | connections, worker dispatch, shared state, log, polling |
| `worker.h/.cpp` | one background thread per socket, so blocking calls never freeze the UI |
| `forms.h/.cpp` | form cells bound to fields of the parameter structs |
| `actions_*.cpp` | the action tables (one entry per API call) |
| `drive_mode.cpp` | the teleport drive tab |
| `viz_image.cpp`, `viz_lidar.cpp` | decoding (stb_image), half-block images, braille lidar, PNG/PLY export |

To add an API call, add one `MakeAction(...)` entry to the matching `actions_*.cpp`. If it takes parameters, `Bind` them to a small struct of fields.
