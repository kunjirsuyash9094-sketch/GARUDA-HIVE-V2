#!/usr/bin/env bash
# Build the Garuda Hive physics core to WebAssembly. Requires Emscripten (emcc).
# Install: https://emscripten.org/docs/getting_started/downloads.html
set -e
cd "$(dirname "$0")/.."
mkdir -p web/public
emcc -std=c++20 -O3 -I include web/core/drone_core.cpp web/core/capi.cpp \
  -o web/public/garuda_core.js \
  -sMODULARIZE=1 -sEXPORT_NAME=GarudaCore \
  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","HEAPF64"]' \
  -sEXPORTED_FUNCTIONS='["_garuda_create","_garuda_destroy","_garuda_reset","_garuda_arm","_garuda_set_attitude","_garuda_set_motors","_garuda_set_wind","_garuda_step","_garuda_get_obs","_garuda_altitude","_garuda_vspeed","_garuda_thrust","_garuda_roll","_garuda_pitch","_garuda_yaw","_malloc","_free"]' \
  -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web
echo "built -> web/public/garuda_core.js + garuda_core.wasm"
