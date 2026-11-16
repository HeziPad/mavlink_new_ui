# QuadPlane GCS

A minimalist, game-style ground control station for an ArduPilot **QuadPlane**
(4 lift rotors + pusher), talking MAVLink v2 over UDP. It has a full-bleed map or
camera with a swappable picture-in-picture, a compact telemetry strip, and six
buttons. It is touch-first, with the mouse as an alternative.

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
ctest --test-dir build        # geo, mission builder, mission transfer, command sender, lift rotors
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
| 1 | **Plan / Sync ↑** | First press: edit the mission on the map. Second press: upload it. **Hold**: re-download the vehicle's mission |
| 2 | **Takeoff / Launch / Auto** | On the ground (**hold 1 s** to confirm): with a mission → AUTO + arm; without → GUIDED + arm + VTOL takeoff, then LOITER on the wing at takeoff altitude. Airborne: resume the loop (AUTO) |
| 3 | **Guide** | Toggle. Tap the map, or the ground in the camera view, to fly there on the wing (GUIDED) |
| 4 | **Hold** | Instant. LOITER: circle on the wing. During a landing this is a go-around |
| 5 | **Land** | **Hold 1 s**. Fly to the planned landing point, spiral down on the wing, land vertically. Locked until the vehicle's mission has a landing point |
| 6 | **QRTL** | Instant. Return home on the wing and land vertically |

To swap the map and camera, tap the small picture-in-picture view.

Everything is made for touch first, with the mouse as an alternative. Locked
buttons say why when pressed (and on hover with a mouse). Tap **SYSTEMS** in
the telemetry bar for the pre-flight checklist. `Esc` deselects or leaves
Plan/Guide, `Del` deletes the selected point, and `Ctrl+Z` undoes a plan edit.

**The lift rotors are for takeoff and landing only.** Every flight action above
keeps the aircraft on the wing. The **LIFT ROTORS** chip in the telemetry bar
shows **OFF**, **ACTIVE** (hover / transition / on the ground) or **ASSIST**.
ASSIST means the rotors spun up in wing-borne flight to prevent a stall
(`Q_ASSIST_SPEED` and friends), and it also raises a toast. The aircraft stays
armed in cruise: disarming would stop the pusher too. The GCS reads
`Q_GUIDED_MODE` at connect and shows an alert if it is 1, because Guide would
then end in a rotor hover.

**Planning:** the waypoints form a **loop that is flown forever** until you
press Hold, Land or QRTL.

- **Map:** tap to add a waypoint, drag to move one, pinch to zoom.
- **Editing a point:** tap a point to select it. A panel then lets you set
  its altitude (−10 / −1 / +1 / +10, hold to repeat), insert a point after
  it, or delete it. Tap empty map to deselect.
- **Landing point:** tap **+ LANDING** in the plan strip, then tap the map.
  The landing point is optional and separate from the loop. Its altitude
  (default 30 m, editable in the panel) is the **approach altitude**: the
  aircraft spirals down to it on the wing and only then switches to the lift
  rotors for the vertical descent.
- **Mouse extras:** the wheel over a point changes its altitude by 5 m, and
  right-click deletes the point.

The upload is built as:

```
HOME → NAV_VTOL_TAKEOFF (40 m) → WP1 … WPn → DO_JUMP(WP1, forever) [→ NAV_VTOL_LAND at L, fixed-wing spiral approach, z = approach alt]
```

With a single waypoint, the aircraft circles it (`NAV_LOITER_UNLIM`) instead of
jumping. The mission never reaches the land item by itself. **Land** jumps to it
with `MAV_CMD_DO_SET_MISSION_CURRENT`.

## Settings

`~/.config/mavlink_gcs_ui/mavlink_gcs_ui.conf` is created on first run.
You can edit the link and video ports, takeoff, default waypoint and landing approach altitude,
cruise speed (used for the lap time), camera HFOV and tilt
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
