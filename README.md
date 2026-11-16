# QuadPlane GCS

A minimalist, game-style ground control station for an ArduPilot **QuadPlane**
(4 lift rotors + pusher), talking MAVLink v2 over UDP. It has a full-bleed map or
camera with a swappable picture-in-picture, a compact telemetry strip, and six
buttons.

C++17 · Qt 6 Widgets · MAVLink `c_library_v2` · GStreamer (optional, for video).

## Prerequisites (Ubuntu / Debian)

```sh
sudo apt install cmake ninja-build qt6-base-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-libav gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly
```

`gstreamer1.0-libav` provides the H.264 decoder. `-ugly` is only needed for the
`x264enc` test stream below. Without the GStreamer dev packages the app still
builds, and the camera view shows the synthetic HUD only.

## Build & test

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build        # geo, mission builder, mission transfer, command sender
```

## Run

```sh
./build/mavlink_gcs_ui                      # listens on udp 127.0.0.1:14551, video on 5600
./build/mavlink_gcs_ui --udp 0.0.0.0:14550 --video-port 5600 --reduced-motion
```

The GCS listens on the UDP port and replies to whichever address the vehicle
sends from. Point the autopilot or MAVProxy output at it.

### With SITL

```sh
cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --out=udp:127.0.0.1:14551
./build/mavlink_gcs_ui
```

### Test video

```sh
gst-launch-1.0 videotestsrc ! video/x-raw,width=1280,height=720 ! x264enc tune=zerolatency \
    ! rtph264pay config-interval=1 pt=96 ! udpsink host=127.0.0.1 port=5600
```

With no frame for 2 s the camera view falls back to a synthetic horizon/grid
and shows `NO VIDEO`.

## Controls

| Key | Button | What it does |
|-----|--------|--------------|
| 1 | **Swap** | Swap map ↔ camera (clicking the PiP also swaps) |
| 2 | **Plan / Sync ↑** | First press: edit the mission on the map. Second press: upload it. **Hold**: re-download the vehicle's mission |
| 3 | **Takeoff / Launch / Auto** | On the ground (**hold 1 s** to confirm): with a mission → AUTO + arm; without → GUIDED + arm + VTOL takeoff. Airborne: resume AUTO |
| 4 | **Guide** | Toggle. Click the map, or the ground in the camera view, to fly there (GUIDED) |
| 5 | **Hold** | Instant. QLOITER when hovering / slow, LOITER when wing-borne |
| 6 | **QRTL** | Instant. Return home and land vertically |

`Esc` leaves Plan/Guide. `Ctrl+Z` undoes a plan edit. Locked buttons show why
on hover, and say it in the comms log when pressed.

**Planning:** click to add, drag to move, right-click to delete, mouse wheel
over a point to change its altitude. The upload is built as `HOME` →
`NAV_VTOL_TAKEOFF` (40 m) → your waypoints → `NAV_VTOL_LAND` at your *last*
point. ArduPlane won't arm if the final leg into the land point is shorter than
its back-transition distance (about 120 m with default parameters). The plan
strip warns when it is.

## Settings

`~/.config/mavlink_gcs_ui/mavlink_gcs_ui.conf` is created on first run.
You can edit the link and video ports, takeoff and default waypoint altitude,
cruise speed (used for the ETA), minimum land approach, camera HFOV and tilt
(used to turn camera clicks into ground targets), and reduced motion. The map
layer and follow state are remembered automatically. Command-line options
override the file for one run.

Map tiles (OpenStreetMap / Esri World Imagery) are cached under
`~/.cache/mavlink_gcs_ui/tiles`. The map works offline with whatever is
cached.

## Layout

```
src/core/   pure C++: vehicle state, ArduPlane modes, geo + camera ray casting, mission builder
src/link/   UDP worker thread, MAVLink decode, mission transfer state machine, command sender
src/ui/     Qt widgets: view host/PiP, map, camera HUD, telemetry bar, dock, effects, tiles, video
tests/      unit tests for core + link (no Qt, no network)
```
