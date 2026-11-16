#!/bin/bash
# Build (or rebuild) the FlightForge TUI into ./build/ff_tui.
#
#   ./build_tui.sh          incremental rebuild
#   ./build_tui.sh clean    wipe the build dir first (refetches FTXUI)
#   ./build_tui.sh run ...  rebuild, then start it; extra args go to ff_tui
#                           e.g. ./build_tui.sh run -a 10.0.0.5 -d 4000

set -e

cd "$(dirname "$0")"

BUILD_DIR=./build/ff_tui

if [ "$1" = "clean" ]; then
  rm -rf "$BUILD_DIR"
  shift
fi

cmake -S tools/ff_tui -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "built: $BUILD_DIR/ff_tui"

if [ "$1" = "run" ]; then
  shift
  # 24-bit colour for the image previews; set FF_TUI_COLORTERM to override
  # (e.g. FF_TUI_COLORTERM=256color if your terminal shows garbage colours)
  COLORTERM=${FF_TUI_COLORTERM:-truecolor} exec "$BUILD_DIR/ff_tui" "$@"
fi
