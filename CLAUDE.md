# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Game-style ground control station for an ArduPilot **QuadPlane**: C++17, Qt 6 Widgets, MAVLink v2 over UDP, optional GStreamer H.264 video. See `README.md` for prerequisites, run flags, controls and settings.

## Commands

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo   # -DGCS_WITH_GSTREAMER=OFF to skip video
cmake --build build
ctest --test-dir build --output-on-failure      # all tests
ctest --test-dir build -R mission_transfer      # one test (geo, mission_builder, mission_transfer, command_sender, lift_rotors, health, orbit_trim, preflight, offline_map)
./build/test_geo                                # or run a test binary directly
./build/mavlink_gcs_ui                          # listens on udp 127.0.0.1:14551
```

SITL (ArduPilot checkout at `~/ardupilot`):

```sh
tools/sitl.sh            # sim_vehicle.py quadplane at your (IP) location, else Israel; --no-geo skips the lookup
# same as: cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane -l LAT,LON,ALT,HDG \
#          --out=udp:127.0.0.1:14551 --add-param-file=tools/sitl_extra.parm
```

There is no linter; the build uses `-Wall -Wextra -Wpedantic` and should stay warning-free. New tests go in `tests/test_<name>.cpp` using the `CHECK` / `CHECK_NEAR` / `TEST_MAIN_RESULT()` macros from `tests/check.hpp` (no framework), and the name must be added to the `foreach` list in `CMakeLists.txt`.

## Architecture

Three layers with a strict dependency direction: `core` ← `link` ← `ui`.

- **`src/core/`**: pure C++, no Qt and no I/O. `VehicleState`, ArduPlane mode numbers and names, geo math including camera-ray-to-ground casting and the camera footprint, the mission builder, `health` (EKF/sensor/GPS/STATUSTEXT → plain-language advisories) and `OrbitTrim`.
- **`src/link/`**: plain C++ plus POSIX sockets, also Qt-free. It is compiled together with core into the `gcs_core` static library, which is what the tests link against. `MavlinkClient` owns a single worker thread that owns the socket, parser, `MissionTransfer` state machine and `CommandSender` (retries and ACK handling). The UI thread only ever:
  - calls `snapshot()` to copy the latest `VehicleState`,
  - calls `pollEvents()` to drain `LinkEvent`s (command results, STATUSTEXTs, mission results),
  - calls request methods (`setMode`, `runSequence`, `uploadMission`, `guidedGoto`, ...), which post closures that run on the worker.

  Keep it this way: don't touch worker-owned members from other threads.
- **`src/ui/`**: Qt widgets, touch-first. Map touch is handled in `MapView::event`: one finger goes through the same `pointer*` path as the mouse, and two fingers pinch. In PiP mode touch is ignored, so Qt synthesises the click that `ViewHost` swaps on. Other widgets rely on Qt's touch→mouse synthesis. Keep targets ≥ 44–48 px and don't put information behind hover only. `MainWindow` drives everything from a 16 ms `PreciseTimer` `frame()` tick that snapshots state, drains events and updates widgets. Rendering is mostly custom `paintEvent` code (map, camera HUD, effects overlay), not stock widgets. `ViewHost` handles the full-bleed main view and the swappable PiP. `ControlDock` holds the five action buttons and their lock/reason logic. Map tiles come from `TileCache`:
  - **Streets:** drawn by `VectorMap` from `offline/map.pmtiles`. That is OSM vector tiles (Protomaps schema), read with `core/pmtiles` + `core/mvt` + `core/gzip` and rendered on its own thread pool, never Qt's global one, which the raster engine waits on.
  - **Satellite:** the offline folder (`ui/offline_store`), then the browsing cache, then the network only when `map/online` is on (off by default).
  - **Filling gaps:** Esri's "no data" JPEG (`kEsriNoDataMd5`) is stored as a `{y}.none` marker, and `MapView` draws the nearest coarser tile enlarged, or the street map where there's no imagery at all.
  - **Downloads:** `OfflineDownloader` (country pack via `PmtilesExtract` range requests; full-detail areas drawn in `MapView::Mode::Area`) and `tools/fetch_offline.py` write the same layout, and every tile has one path, so overlapping areas share files.
  - **Ground height:** `ElevationCache` reads AWS terrarium tiles (tile maths in `core/tiles`). Video is only compiled when `GCS_HAVE_GSTREAMER` is defined. Everything internal is in metres; any altitude shown to the user goes through `ui/units` (`altText`, `altStepMeters`), which follows the m/ft setting. Problems are shown as `core::health` advisories in `AlertsPanel` (opened from SYSTEMS or the alert banner), not as raw flags.

`include/c_library_v2/` holds the vendored, generated MAVLink headers. They are included as SYSTEM, and link sources disable `-Wpedantic` and `-Waddress-of-packed-member`. Don't edit them.

## ArduPlane / SITL gotchas (verified against SITL)

- GUIDED takeoff must use `MAV_CMD_NAV_TAKEOFF` (22), not `MAV_CMD_NAV_VTOL_TAKEOFF` (84).
- Product rule: **the lift rotors are only for takeoff and landing.** Hold and Guide are GUIDED orbits via `DO_REPOSITION` (COMMAND_INT): `param2` flag CHANGE_MODE, `param3` radius, `param4` 0 = CW. Re-sending with the same centre changes the radius, altitude or direction live, which LOITER can't do (it only uses `WP_LOITER_RAD`). They are never QLOITER. A GUIDED takeoff ends in a rotor hover (`QPOS_POSITION2`), so `MainWindow::checkTakeoffHandoff` calls `hold()` at about the takeoff altitude. `Q_GUIDED_MODE` must be 0, or GUIDED targets end in a hover. The GCS reads it and alerts. Don't "solve" the rule by disarming in flight: that stops the pusher too.
- The mission layout (`core/mission.cpp`) is `HOME` → `NAV_VTOL_TAKEOFF` → WPs → `DO_JUMP`(→2, repeat −1) → optional `NAV_LOITER_TO_ALT` (the approach circle beside L, ±radius = CW/CCW, `param4` = 1 tangent exit) → `NAV_VTOL_LAND` (`param1` = 0). The landing is only reached through `MAV_CMD_DO_SET_MISSION_CURRENT`. The Land button first re-uploads the approach with the current `WIND`. Auto after an aborted landing sets the current item back to the loop.
  - ArduPlane only stays on the wing toward a VTOL_LAND beyond `transition_threshold = 1.5·v²/(2·Q_TRANS_DECEL)` (about 234 m at 25 m/s). Closer than that it goes straight to rotor position control. Its built-in spiral option (`param1` = 1) breaks out about one loiter radius from L, which is inside that threshold. In SITL that meant 79 s on the rotors. That's why the GCS builds its own circle and straight leg (`computeLandingApproach`).
  - `NAV_LOITER_TO_ALT` completes when the altitude is reached *and* the aircraft is heading toward the next item, so it leaves on the tangent.
  - The circle's `z` is the approach altitude. It must not be 0, because `Location::sanitize` reads a relative 0 as "current altitude".
  - The VTOL_LAND's `z` must be 0, meaning ground at home level. Without a rangefinder, ArduPlane uses the land item's altitude as the ground for its descent profile (`relative_ground_altitude` → `height_above_target`). With z = approach alt, it thinks it's already down and descends the whole way at `Q_LAND_FINAL_SPD` (0.5 m/s). In SITL that was 61 s instead of about 20 s.
  - The "VTOL land too short" arming check only fires when a `NAV_WAYPOINT` directly precedes the land item. The `LOITER_TO_ALT` in between avoids it.
  - A one-waypoint loop uses `NAV_LOITER_UNLIM` instead.
- ArduPlane flies a GUIDED loiter bigger than `param3`: `AP_L1_Control::loiter_radius()` scales it by EAS2TAS² when `NAVL1_LIM_BANK` = 0 (+7 % at SITL's ~700 m AMSL), and L1 overshoots small circles (80 m flew as 91 m). `core::OrbitTrim` measures whole laps and commands `requested / ratio`, so send `trim_.commandRadius()`, not the requested radius.
- Q_ASSIST can spin the lift rotors in cruise, and speed assist sends no STATUSTEXT. The LIFT ROTORS indicator therefore reads `SERVO_OUTPUT_RAW` and maps outputs with `SERVOn_FUNCTION` 33–40.
- There is no QRTL button: Land is the one way down, and failsafes fly the same landing. Every mission the GCS uploads has a landing (L, or home when the plan has none) preceded by `DO_LAND_START` (189). With `RTL_AUTOLAND` = 2 any RTL (long RC loss, GCS loss, low battery) jumps straight to it (`ModeRTL::navigate` → `jump_to_landing_sequence`). `Q_RTL_MODE` must be 0: with 2 (VTOL approach) `navigate()` returns before the autoland check, and 3 switches to QRTL on entry. `BATT_FS_LOW_ACT` 2 (Land) jumps to the sequence directly. A `DO_LAND_START` with `RTL_AUTOLAND` = 0 fails arming ("DO_LAND_START set and RTL_AUTOLAND disabled"). `MainWindow::refreshApproachForWind` re-uploads the approach when the wind turns ≥ 30°; a same-length re-upload mid-AUTO keeps the current leg.
- Safety at boot is `BRD_SAFETY_DEFLT` (1 = starts safe, reboot to apply; `AP_BoardConfig::board_init_safety`). SITL honours it.
- Gimbal: ArduPilot's `GIMBAL_DEVICE_ATTITUDE_STATUS` gives roll/pitch in the earth frame and yaw in the vehicle frame (flag). `core::cameraLos` turns that, or the fixed tilt setting, into an earth-frame camera attitude plus pan/tilt relative to the platform.
- ArduPlane reports `vtol_state` as FW while parked on the ground, so don't infer "hovering" from it alone.
- Resuming AUTO continues from the vehicle's *current* item. After a Hold during the AUTO VTOL climb that is still `NAV_VTOL_TAKEOFF`, and AUTO would hover and climb on the rotors again. `launchOrAuto` always sets the current item to `nearestLoopSeq()` first.
- After a VTOL landing the mission stays on the land item and AUTO won't arm ("In landing sequence", "missing takeoff waypoint"). Launch sends `SET_MISSION_CURRENT(1, reset)` before AUTO + arm.
- Safety switch: `SYS_STATUS` `MOTOR_OUTPUTS` is always present and *enabled* unless safety is engaged (`safetyState()`). Toggle with `MAV_CMD_DO_SET_SAFETY_SWITCH_STATE`; SITL implements it. Never while armed: it cuts every output, the pusher and servos included.
- RTL, QRTL and QLAND refuse arming ("mode not armable"), so after a failsafe landing the pre-arm bit fails although nothing is wrong. `reconcileLandedMode` turns that into an info card.
- Preflight (`core/preflight`, `ui/preflight_runner`, `ui/preflight_screen`): `MAV_CMD_DO_MOTOR_TEST` spins only the lift motors (test order A.. clockwise from front-right, `core/frame_layout` mirrors `AP_MotorsMatrix`); below ~15 % it doesn't pass spin-min. `DO_SET_SERVO` can't move assigned surfaces, so the surface test uses `MANUAL_CONTROL` in MANUAL (x = pitch, + is nose down). Calibrations ACK only when done: send once with a long timeout (`max_attempts`, `ack_timeout`). Disarm after a GCS-driven ground test with force (21196). Never one-tap `FENCE_ENABLE=1`: with no fence defined it fails pre-arm. ArduPilot after 4.6 has `ARMING_SKIPCHK` instead of `ARMING_CHECK`.
- Don't gate Launch on the prearm-healthy bit. It is evaluated for the *current* mode (e.g. "QRTL mode not armable") and doesn't apply to the mode Launch switches to.

## UI motion

Telemetry arrives at 10–25 Hz. Don't draw the latest sample through a fast exponential tween: it arrives in about 60 ms and then sits still, which looks like a staircase. `MainWindow::frame` pulls every frame and calls `predictAttitude` / `predictPosition` (last sample + rate × age, capped), then views apply light smoothing on top.

## Paint budget

Everything repaints at 60 Hz in software, so each frame's full-window repaint must stay well under 16 ms on the target screen (1920×1200 here: ~6 ms map, ~8 ms camera). Measured costs at that size: 50 full-length AA grid lines ≈ 4.6 ms, a 6000-point trail ≈ 5 ms, smooth-scaled tiles ≈ 2.4 ms, a full-window radial gradient or pattern fill ≈ 1–2.5 ms, one full-window pixmap blit ≈ 0.3 ms. So:

- Anything that doesn't change every frame goes into a cached pixmap: `MapView` draws grid + tiles + the trail into a backdrop with a margin (`paintBackdrop`) and only shifts it; `CameraView` caches its vignette + scanlines.
- Don't keep transparent full-window widgets visible when idle: `FxOverlay` hides itself, because a visible one forces a full recomposite every frame.
- Qt 6's raster engine splits big fills over the global `QThreadPool`; `main.cpp` caps it at 2 threads (same frame time as 16, ~40 % less CPU). `GCS_PAINT_THREADS` overrides.
- Clip every line drawn from map coordinates to the view first (`drawClipped`, `drawLineClipped`, `circleVisible` in `map_view.cpp`). A leg to a far-off point (SITL restarted elsewhere keeps its old mission: 13 000 km) is millions of pixels long when zoomed in, and Qt rasterises every dash of it. That held the UI thread for ~3 s per frame.
- Measure with a driver that `repaint()`s widgets and times them, not by eye. `GCS_FPS=1` prints ticks, paints, stalls and telemetry rates live.

## Qt gotchas

- Qt's `emit` macro clashes with any method named `emit()`, so don't name methods that.
- `QTimer::singleShot` and default timers are coarse and can fire ~5% early. Use `Qt::PreciseTimer` where timing matters, including test and screenshot drivers.
