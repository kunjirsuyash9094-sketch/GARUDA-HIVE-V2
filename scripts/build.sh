#!/usr/bin/env bash
# One-command build. godot-cpp is fetched automatically — no submodules.
set -euo pipefail
cd "$(dirname "$0")/.."
CONFIG="${1:-Release}"
echo "[Garuda Hive] Configuring ($CONFIG)..."
cmake -B build -DCMAKE_BUILD_TYPE="$CONFIG"
echo "[Garuda Hive] Building..."
cmake --build build --config "$CONFIG" --parallel
echo "[Garuda Hive] Done -> gdextension/bin/"
ls -la gdextension/bin/
