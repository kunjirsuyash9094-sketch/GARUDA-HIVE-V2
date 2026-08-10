# SkySim in the browser (Phase 4)

The physics core, compiled to WebAssembly, running client-side — no install, no
GPU, no server. This is the "anyone, zero-install" tier.

## How it works

`web/core/drone_core.{hpp,cpp}` is the SkySim physics **decoupled from Godot**:
it reuses the exact pure subsystems (blade element, atmosphere, aero effects,
flight controller, mixer) and adds a standalone 6-DOF integrator. `capi.cpp`
exposes a small C API; Emscripten compiles it to `skysim_core.js` + `.wasm`.
`index.html` loads that, renders with Three.js, and lets you fly by keyboard or
plug in a JavaScript control function.

The build is **single-threaded on purpose** — so it needs no SharedArrayBuffer
and no COOP/COEP cross-origin-isolation headers, and hosts on any static server.

## Build

```bash
# install Emscripten once: https://emscripten.org/docs/getting_started/downloads.html
./web/build_wasm.sh          # -> web/public/skysim_core.js + .wasm
```

## Run locally

```bash
cp web/public/skysim_core.* web/     # put the wasm next to index.html
cd web && python3 -m http.server 8000
# open http://localhost:8000
```

## Deploy

Pushing to `main` runs `.github/workflows/web.yml`, which builds the wasm and
publishes `web/` to GitHub Pages. Enable Pages (Settings → Pages → GitHub
Actions) once.

## Scope

Client-side is for interactive flight and **classical control / light policies**
(JavaScript, or Python via Pyodide later). For PyTorch/GPU training or firmware
SITL, point the same algorithm at the native agent server (Phase 1) — the web UI
is designed to attach to that backend over WebSocket.

## Note on the core vs the Godot build

This is an independent integrator sharing SkySim's force models. It is
physically faithful but **not bit-identical** to the Godot/native build (a
different rigid-body integrator). Cross-tier determinism is a Phase 5 concern.
