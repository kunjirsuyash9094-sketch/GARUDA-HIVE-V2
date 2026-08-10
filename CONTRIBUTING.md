# Contributing to SkySim

Thanks for helping build an open, accessible drone simulator. SkySim is a
community project — contributions of code, docs, examples, bug reports, and
test scenarios are all welcome.

## Ground rules

- Be kind. See [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
- No CLA. We use a lightweight **Developer Certificate of Origin**: add a
  `Signed-off-by:` line to your commits (`git commit -s`) to certify you wrote
  the change and can contribute it under the project's MIT license.
- Keep PRs small and focused — one logical change per PR is easiest to review.

## Where help is most useful

See [`ROADMAP.md`](ROADMAP.md). The roadmap is dependency-ordered; the highest-
leverage open work is usually in the current phase. Good first issues are
tagged `good first issue`.

## Project layout

- `src/`, `include/` — C++20 physics + SITL (the performance-critical core).
  Physics stays in C++.
- `demo/` — the Godot project's scenes and GDScript (controllers, HUD, demo
  scenes). Scene/UI logic stays in GDScript.
- `gdextension/` — the extension config and (CI-built) binaries.
- `tools/` — Python probes for the SITL bridges.
- `.github/workflows/` — CI that builds binaries and publishes releases.

Rule of thumb: **physics and anything in the per-tick hot path → C++; scene,
input, UI, and orchestration → GDScript.**

## Building from source

You only need this if you're changing the C++ extension. godot-cpp is fetched
automatically — no submodules.

```bash
./scripts/build.sh        # Linux / macOS
scripts\build.bat         # Windows
```

Then open the project folder in **Godot 4.3+** and press Play. If you're only
changing GDScript/scenes, you don't need to build C++ — just use the extension
binaries from a release.

## Running the demo

Open the project in Godot and run it. The default scene (`demo/free_flight.tscn`)
is a fly-it-yourself drone: **Enter** to arm, **W/S** throttle, **A/D** yaw,
arrow keys pitch/roll (gamepad supported).

## Coding conventions

- C++: C++20, 4-space indent, keep headers self-contained. Match the style of
  surrounding code.
- GDScript: tabs (Godot default), typed variables where practical.
- Physics changes: note any new assumptions or units in comments, and mention in
  the PR how you validated the behaviour.

## Reporting bugs / requesting features

Use the issue templates. For bugs, include your OS, Godot version, and steps to
reproduce. For firmware/SITL issues, include which firmware and version.
