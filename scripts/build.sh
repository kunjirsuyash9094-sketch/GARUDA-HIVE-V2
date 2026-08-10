#!/usr/bin/env bash
# One-command build. godot-cpp is fetched automatically — no submodules.
set -euo pipefail
cd "$(dirname "$0")/.."
CONFIG="${1:-Release}"
echo "[SkySim] Configuring ($CONFIG)..."
cmake -B build -DCMAKE_BUILD_TYPE="$CONFIG"
echo "[SkySim] Building..."
cmake --build build --config "$CONFIG" --parallel
echo "[SkySim] Done -> gdextension/bin/"
ls -la gdextension/bin/
