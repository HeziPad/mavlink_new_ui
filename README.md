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
| 2 | **Takeoff / Launch / Auto / Resume** | On the ground (**hold 1 s** to confirm): with a mission → AUTO + arm; without → GUIDED + arm + VTOL takeoff, then Hold (circle) on the wing at takeoff altitude. Airborne: resume the loop (AUTO), also after an aborted landing. During a landing it reads **Resume**: abort the landing and go back to the loop |
| 3 | **Guide** | Toggle. Tap the map, or the ground in the camera view, to circle there on the wing (GUIDED) |
| 4 | **Hold** | Instant. Circle where the aircraft is, on the wing (GUIDED). During a landing this is a go-around |
| 5 | **Land** | **Hold 1 s**. Circle down beside the planned landing point on the wing, leave into the wind, back-transition and land vertically. Locked until the vehicle's mission has a landing point |
| 6 | **QRTL** | Instant. Return home on the wing and land vertically |

**Hold and Guide circles:** while circling, a panel above the dock sets the
**altitude** (−10 / −1 / +1 / +10 m, or −50 … +50 ft; 20–5000 m), the
**radius** (−50 … +50 m, 40–5000 m) and the **direction** (CW / CCW). Holding a
button repeats and speeds up (×5 after 1 s, ×10 after 2.5 s). Changes apply
live. The circle starts at the vehicle's `WP_LOITER_RAD`.

ArduPlane flies a loiter **bigger than commanded**: `AP_L1_Control` scales the
radius by EAS2TAS² (about +7 % at 700 m above sea level, more at higher fields,
with `NAVL1_LIM_BANK` = 0), and the L1 loop overshoots small circles (SITL: 80 m
flew as 91 m). The GCS measures each lap and trims the radius it sends, so the
flown circle matches the dotted one. The panel shows `≈ flown 91` while it
trims and `✓ flown 80` once it matches. Both use `DO_REPOSITION`, so the vehicle shows
GUIDED. In AUTO the altitude comes from the plan: edit the waypoint instead.
The ALT chip shows the commanded altitude as `62→80` while the aircraft climbs
or descends to it.

**Land vs QRTL:**

| | **Land** | **QRTL** |
|---|---|---|
| Where | The landing point you planned (**L**) | Home, or the nearest rally point |
| Descent | On the wing, on a circle beside L, down to the approach altitude | On the wing at RTL altitude, then on the rotors from about `Q_RTL_ALT` |
| Approach direction | Into the current wind, with the circle on the side the aircraft comes from | Straight from wherever the aircraft is |
| Needs | A landing point in the mission | Nothing: always available, so it's the emergency button |

**Bank angle:** how hard the aircraft banks is the autopilot's job.
`ROLL_LIMIT_DEG` caps it (the SITL quadplane uses 65°, the ArduPlane default is
45°). `NAVL1_PERIOD` sets how aggressively it turns onto a new leg (lower =
sharper). A square loop has 90° corners, so the aircraft turns at its limit at
each one. A bigger `NAVL1_PERIOD`, a lower `ROLL_LIMIT_DEG` or rounder loops
(more points) soften it.

To swap the map and camera, tap the small picture-in-picture view.

Everything is made for touch first, with the mouse as an alternative. Locked
buttons say why when pressed (and on hover with a mouse).

**SYSTEMS** in the telemetry bar counts problems. Tap it (or the red alert
banner) to open the alerts panel: status pills (link, GPS, navigation, sensors,
power, arming) and one card per problem. Each card says what's wrong, why it
matters and what to try, with the raw data (EKF flags, the autopilot's
message) in small print. The cards come from the EKF status, `SYS_STATUS` sensor
health, GPS, battery and the link, plus the autopilot's PreArm and failsafe
messages in plain language. PreArm cards stay while ArduPilot keeps repeating
them (every 30 s) and clear when the vehicle arms. The panel only explains: it
never changes the vehicle.

The **⚙** at the end of the bar opens **Settings** (for now: altitude in metres
or feet). `Esc` deselects or leaves
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
  aircraft circles down to it on the wing and only then switches to the lift
  rotors for the vertical descent.
- **Mouse extras:** the wheel over a point changes its altitude by 5 m, and
  right-click deletes the point.

The upload is built as:

```
HOME → NAV_VTOL_TAKEOFF (40 m) → WP1 … WPn → DO_JUMP(WP1, forever)
     [→ NAV_LOITER_TO_ALT (approach circle beside L) → NAV_VTOL_LAND at L]
```

With a single waypoint, the aircraft circles it (`NAV_LOITER_UNLIM`) instead of
jumping. The mission never reaches the landing by itself.

**The landing approach.** When you press **Land**, the GCS does four things:

1. It holds where the aircraft is.
2. It recomputes the approach with the **current wind**, then re-uploads the mission.
3. It jumps to the approach with `MAV_CMD_DO_SET_MISSION_CURRENT`.
4. It remembers which waypoint to go back to if you abort the landing.

The approach is laid out like this:

- **The straight leg** points into the wind and ends at L. It is
  `1.3 × 1.5·v²/(2·Q_TRANS_DECEL) + 40 m` long (at least 150 m). ArduPlane only
  stays on the wing toward a VTOL landing when it's further away than
  `1.5·v²/(2·Q_TRANS_DECEL)`. Closer than that, it hovers its way in on the
  rotors.
- **The circle** sits beside the straight leg, on the side the aircraft is
  coming from, and has the loiter radius. The aircraft descends on it to the
  approach altitude, then leaves on the tangent, heading into the wind.
- **The back-transition** happens on the straight leg. ArduPlane starts it at its
  own stopping distance, so it doesn't overshoot. The rotors only run for that
  and the short vertical descent from the approach altitude.

The map draws the circle, the leg and a wind arrow. They are faint while
circling the loop and bright while landing.

## Settings

The in-app settings screen (⚙) switches altitudes between metres and feet
everywhere: HUD tape, telemetry bar, map labels, panels and steppers.
Distances and speeds stay metric.

`~/.config/mavlink_gcs_ui/mavlink_gcs_ui.conf` is created on first run.
You can edit the link and video ports, takeoff, default waypoint and landing approach altitude,
cruise speed (used for the lap time), camera HFOV, the fixed-mount tilt used
when no gimbal reports its angles, and reduced motion. The map
layer and follow state, the altitude unit, the camera HUD reference and the
map attitude ball (open or collapsed) are remembered automatically. Command-line options
override the file for one run.

## Camera view

The horizon, elevation ladder, roll scale and heading tape follow either the
camera's **line of sight (LOS)** or the **aircraft body**. Tap the **HUD ·
LOS / HUD · BODY** chip under the AIRCRAFT box to switch.

- **LOS** (default): the rulers overlay the video. The heading tape shows the
  LOS azimuth, with a cyan caret for the aircraft's nose.
- **BODY**: the screen centre is the nose (a `W` waterline symbol). The ladder
  shows the aircraft's pitch and roll, and the tape its heading. An amber
  `CAM` box and caret mark where the camera points. Over live video the rulers
  no longer line up with the picture.

The roll scale is numbered (10/20/30/45/60) and has a live `R 23°` / `L 12°`
readout. It turns amber beyond 45°. The LOS comes from the
gimbal (`GIMBAL_DEVICE_ATTITUDE_STATUS`), or from the fixed-mount tilt when no
gimbal reports.

Two readouts sit beside it:

- **AIRCRAFT:** roll, pitch and heading, with a small attitude ball.
- **GIMBAL · LOS:** pan and tilt **relative to the aircraft**, with the
  earth-frame azimuth and elevation underneath.

In Guide mode, a tap on the video is cast along the real LOS to find the
ground point.

**Gimbal angle frame.** The MAVLink spec says a gimbal reports roll and pitch in
the earth frame. Gimbals with attitude feedback do that. ArduPilot's **servo**
mount has no feedback, so it reports the body-frame servo angles it commands,
even though its flags claim the earth frame. For a servo-driven gimbal, set
`camera/gimbal_body_frame=true` in the settings file. Otherwise the HUD and
camera-tap targets will be off whenever the aircraft banks or pitches.

Motion is drawn every frame (about 60 fps). It is predicted from each sample's
rates and timestamp (attitude at about 25 Hz, position at 10 Hz), so
the horizon, heading tape and aircraft icon move continuously instead of in
steps.

On the map, a translucent green **camera footprint** shows the ground the
camera sees: the image border is cast onto flat ground at home level. Edges
that run past the horizon are dashed, and a dashed line of sight leads to the
boresight crosshair, labelled with its range. An **attitude ball** sits at the
top left (tap it to shrink it to a small puck).

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
