# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Game-style ground control station for an ArduPilot **QuadPlane**: C++17, Qt 6 Widgets, MAVLink v2 over UDP, optional GStreamer H.264 video. See `README.md` for prerequisites, run flags, controls and settings.

## Commands

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo   # -DGCS_WITH_GSTREAMER=OFF to skip video
cmake --build build
ctest --test-dir build --output-on-failure      # all tests
ctest --test-dir build -R mission_transfer      # one test (geo, mission_builder, mission_transfer, command_sender)
./build/test_geo                                # or run a test binary directly
./build/mavlink_gcs_ui                          # listens on udp 127.0.0.1:14551
```

SITL (ArduPilot checkout at `~/ardupilot`):

```sh
cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --out=udp:127.0.0.1:14551
```

There is no linter; the build uses `-Wall -Wextra -Wpedantic` and should stay warning-free. New tests go in `tests/test_<name>.cpp` using the `CHECK` / `CHECK_NEAR` / `TEST_MAIN_RESULT()` macros from `tests/check.hpp` (no framework), and the name must be added to the `foreach` list in `CMakeLists.txt`.

## Architecture

Three layers with a strict dependency direction: `core` ← `link` ← `ui`.

- **`src/core/`**: pure C++, no Qt and no I/O. `VehicleState`, ArduPlane mode numbers and names, geo math including camera-ray-to-ground casting, and the mission builder.
- **`src/link/`**: plain C++ plus POSIX sockets, also Qt-free. It is compiled together with core into the `gcs_core` static library, which is what the tests link against. `MavlinkClient` owns a single worker thread that owns the socket, parser, `MissionTransfer` state machine and `CommandSender` (retries and ACK handling). The UI thread only ever:
  - calls `snapshot()` to copy the latest `VehicleState`,
  - calls `pollEvents()` to drain `LinkEvent`s (command results, STATUSTEXTs, mission results),
  - calls request methods (`setMode`, `runSequence`, `uploadMission`, `guidedGoto`, ...), which post closures that run on the worker.

  Keep it this way: don't touch worker-owned members from other threads.
- **`src/ui/`**: Qt widgets. `MainWindow` drives everything from a 16 ms `PreciseTimer` `frame()` tick that snapshots state, drains events and updates widgets. Rendering is mostly custom `paintEvent` code (map, camera HUD, effects overlay), not stock widgets. `ViewHost` handles the full-bleed main view and the swappable PiP. `ControlDock` holds the six action buttons and their lock/reason logic. Tiles are fetched and disk-cached by `TileCache`. Video is only compiled when `GCS_HAVE_GSTREAMER` is defined.

`include/c_library_v2/` holds the vendored, generated MAVLink headers. They are included as SYSTEM, and link sources disable `-Wpedantic` and `-Waddress-of-packed-member`. Don't edit them.

## ArduPlane / SITL gotchas (verified against SITL)

- GUIDED takeoff must use `MAV_CMD_NAV_TAKEOFF` (22), not `MAV_CMD_NAV_VTOL_TAKEOFF` (84).
- An uploaded mission is `HOME` → `NAV_VTOL_TAKEOFF` → user waypoints → `NAV_VTOL_LAND` at the last point. The final approach leg must be ≥ ~120 m, or arming fails with "VTOL land too short".
- ArduPlane reports `vtol_state` as FW while parked on the ground, so don't infer "hovering" from it alone.
- Don't gate Launch on the prearm-healthy bit. It is evaluated for the *current* mode (e.g. "QRTL mode not armable") and doesn't apply to the mode Launch switches to.

## Qt gotchas

- Qt's `emit` macro clashes with any method named `emit()`, so don't name methods that.
- `QTimer::singleShot` and default timers are coarse and can fire ~5% early. Use `Qt::PreciseTimer` where timing matters, including test and screenshot drivers.
