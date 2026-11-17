# QuadPlane GCS

A minimalist, game-style ground control station for an ArduPilot **QuadPlane**
(4 lift rotors + pusher), talking MAVLink v2 over UDP. It has a full-bleed map or
camera with a swappable picture-in-picture, a compact telemetry strip, and five
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

If it feels slow, run it with `GCS_FPS=1`. Once a second it prints how often it
redrew, its longest stall, and the attitude/position telemetry rates. That
tells a slow UI apart from slow telemetry. Through MAVProxy, use
`--streamrate=-1`; otherwise MAVProxy forces every stream to 4 Hz.

### With SITL

```sh
tools/sitl.sh                 # ArduPlane quadplane SITL at your location, output to 127.0.0.1:14551
./build/mavlink_gcs_ui
```

`tools/sitl.sh` starts `sim_vehicle.py` (checkout in `$ARDUPILOT`, default
`~/ardupilot`) with `tools/sitl_extra.parm`. That file sets safety ON at boot,
a battery monitor, and the failsafe settings below. The start location comes from:

1. `GCS_SITL_LOCATION=lat,lon,alt,heading`, if set;
2. otherwise your public IP's approximate location (ipinfo.io, falling back to
   ip-api.com), with the ground altitude from Open-Meteo. **This sends your IP
   address to those services.** `tools/sitl.sh --no-geo` skips it;
3. otherwise Israel (Tel Aviv, `32.0853,34.7818,17,0`).

Any other arguments go to `sim_vehicle.py` (e.g. `tools/sitl.sh --speedup 2`).

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
| 2 | **Takeoff / Launch / Resume Plan** | On the ground (**hold 1 s** to confirm): with a mission → AUTO + arm; without → GUIDED + arm + VTOL takeoff, then Hold (circle) on the wing at takeoff altitude. Locked while the safety switch is on. Airborne: **Resume Plan** goes back to the planned loop on the wing (AUTO), also mid-landing (aborts it). If the vehicle's current item isn't a loop waypoint (still the VTOL takeoff after a Hold during the climb, or the landing), it is reset to the loop first, so AUTO never re-runs a rotor takeoff. While flying the loop it reads **On Plan** |
| 3 | **Guide** | Toggle. Tap the map, or the ground in the camera view, to circle there on the wing (GUIDED) |
| 4 | **Hold** | Instant. Circle where the aircraft is, on the wing (GUIDED). During a landing this is a go-around |
| 5 | **Land** | **Hold 1 s**. Circle down beside the landing point **L** on the wing, leave into the wind, back-transition and land vertically. Without an L it lands at home the same way |

**Hold and Guide circles:** while the **Guide** button is on, a panel above the dock sets the
**altitude** (−10 / −1 / +1 / +10 m, or −50 … +50 ft; 20–5000 m), the
**radius** (−50 … +50 m, 40–5000 m) and the **direction** (CW / CCW). Holding a
button repeats and speeds up (×5 after 1 s, ×10 after 2.5 s). Changes apply
live. The circle starts at the vehicle's `WP_LOITER_RAD`. Turn Guide off to hide
the panel; the circle keeps flying.

ArduPlane flies a loiter **bigger than commanded**: `AP_L1_Control` scales the
radius by EAS2TAS² (about +7 % at 700 m above sea level, more at higher fields,
with `NAVL1_LIM_BANK` = 0), and the L1 loop overshoots small circles (SITL: 80 m
flew as 91 m). The GCS measures each lap and trims the radius it sends, so the
flown circle matches the dotted one. The panel shows `≈ flown 91` while it
trims and `✓ flown 80` once it matches. The readout is live (the last lap's
worth of samples, from a quarter lap on); the trim itself acts on full laps. Both use `DO_REPOSITION`, so the vehicle shows
GUIDED. In AUTO the altitude comes from the plan: edit the waypoint instead.
The ALT chip shows the commanded altitude as `62→80` while the aircraft climbs
or descends to it.

**Landing and failsafes:** there is one way down, and the failsafes use it too.

- **Land** recomputes the approach with the current wind and uploads it. It
  circles down beside **L** on the wing, leaves the circle on a straight leg
  into the wind, back-transitions and lands vertically. If the plan has no L,
  **L is home**: every uploaded mission gets a landing, at L or at home.
- **Failsafes fly the same landing.** The mission has a `DO_LAND_START` marker
  just before the approach. With `RTL_AUTOLAND = 2`, any RTL jumps straight
  into that landing instead of circling over home. Those RTLs are long RC loss
  (`FS_LONG_ACTN = RTL`), GCS link loss (`FS_GCS_ENABL = 1`) and low battery
  (`BATT_FS_LOW_ACT = RTL`). `Q_RTL_MODE` must be 0: other values make RTL do
  its own vertical landing at home and skip L.
- **Critical battery** stays `QLand`, landing vertically where it is, because
  there may not be enough left to reach L.
- **The on-board approach follows the wind.** In flight, the GCS re-uploads it
  when the wind direction moves 30° or more (at most once a minute), so a
  failsafe also lands into the current wind. A toast says when it does.
- **Preflight checks all of these** and fixes them with one tap.
- **Arming is refused if `RTL_AUTOLAND` is 0.** ArduPilot won't arm with a
  `DO_LAND_START` in the mission and `RTL_AUTOLAND` at 0 ("DO_LAND_START set
  and RTL_AUTOLAND disabled"); SYSTEMS explains it.

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
banner) to open the alerts panel; tap it again, tap anywhere outside the panel
or press `Esc` to close it. It shows status pills (link, GPS, navigation, sensors,
power, arming) and one card per problem. Each card says what's wrong, why it
matters and what to try, with the raw data (EKF flags, the autopilot's
message) in small print. The cards come from the EKF status, `SYS_STATUS` sensor
health, GPS, battery and the link, plus the autopilot's PreArm and failsafe
messages in plain language. PreArm cards stay while ArduPilot keeps repeating
them (every 30 s) and clear when the vehicle arms. The panel only explains: it
never changes the vehicle.

The **⚙** at the end of the bar opens **Settings**: altitude in metres or feet,
the flight trail (on/off, last 30 s / 1 / 2 / 5 / 10 min / all), and **offline
maps** (see below).

**Mouse over the map** shows the point under the cursor, bottom right:

- its latitude and longitude;
- the **ground height** above sea level, and above or below home;
- its distance and bearing from the aircraft.

Ground height comes from SRTM elevation data (AWS Terrain Tiles, about ±10 m).
It measures the surface, so it includes roofs and tree tops.

**SAFETY** in the bar is the hardware safety switch: `ON` (amber, motor
outputs cut, Launch locked) or `OFF` (red, motors live). Hold it for 1 s to switch, in either
direction (`MAV_CMD_DO_SET_SAFETY_SWITCH_STATE`). It's locked while armed:
engaging safety in flight would cut the pusher and servos too. The GCS never
changes it on its own. **Safety is ON after every power-up**
(`BRD_SAFETY_DEFLT = 1`, checked and fixed by preflight, applied after a
reboot), so turning it off is always a deliberate step.

After an RTL / QLAND landing (a failsafe) the vehicle sits in a mode ArduPlane can't
arm in, so its pre-arm check fails. SYSTEMS shows that as an info card, not a
fault: Launch switches mode before arming. `Esc` deselects or leaves
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
press Hold or Land.

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

## Preflight

On the ground, dock slot 3 is **PREFLIGHT** (a progress ring shows how far
you are). **Launch stays locked until it passes.** A 3 s hold on "launch
without preflight" overrides that, and the override is written to the report.

It is a full-window checklist in 12 stages:

| Stage | What happens |
|---|---|
| Aircraft | Firmware, frame (`Q_FRAME_CLASS/TYPE`) and output map are read; you set the battery chemistry, cells, capacity and typical flight time once |
| Site | Airspace, weather, a clear 10 m takeoff circle, the landing point, bystanders, GCS power |
| Airframe | Wing and booms, props (CW/CCW per position shown), linkages, **pitot cover off**, CG, battery, antennas |
| Power | Charge per cell for the chemistry, cell balance, current sensor, autopilot supply |
| Links & RC | Telemetry rate and radio margin; move each stick to both ends; mode-switch positions with their modes; **RC failsafe test** (turn the transmitter off) |
| Sensors | Gyro, level, accelerometer (simple or 6-position with each pose drawn), **compass** (from a known heading, or rotate it while a sphere fills in), baro/airspeed zero, pitot blow test, attitude follows the aircraft, heading matches; health, EKF, GPS, home, logging, vibration |
| Surfaces | The GCS sweeps ailerons, elevator and rudder in MANUAL (disarmed) while the picture shows which way each must move; a wrong one can be reversed with one tap; stabilisation check in FBWA |
| Motors | Props off (or tied down), safety OFF, then each lift motor A–D spins 2 s at 15 %: the picture lights the position and spin direction it must have, and the GCS checks the right output moved. The pusher spins 2 s at 12 % in MANUAL and is force-disarmed |
| Payload | Gimbal sweep, video frames arriving |
| Settings | 17 parameter checks (Q_GUIDED_MODE, Q_ASSIST_SPEED, airspeed limits, arming checks, RC/GCS/battery failsafes and voltages for your pack, fence, loiter radius vs bank limit, ...), each with the reason and a one-tap fix (confirmed, read back, logged) where there is a safe value |
| Mission | On the vehicle, landing point, altitudes, range, laps per pack |
| Ready | The GCS selects AUTO, resets the mission to its takeoff and runs the pre-arm checks (their messages are listed if any fail); then safety OFF and **READY TO LAUNCH** |

Tests that move or spin anything need a 1 s hold, have a **STOP ALL** button,
and have vehicle-side timeouts as well. A pass stays valid until the autopilot
reboots, the battery is swapped, a parameter is changed elsewhere, an hour
passes, or it flies; after a flight a short **turnaround** preflight is offered.
Every run is saved as JSON under `~/.local/share/mavlink_gcs_ui/preflight/`.

## Offline maps

The map imagery and the ground height can work with no internet.

**In the app:** open Settings ⚙ → **OFFLINE**.

1. Pick a radius (10 / 25 / 50 km) around the aircraft, or around the map
   centre when there's no aircraft.
2. Check the tile count and size estimate.
3. Press **DOWNLOAD AREA**. Coarse zooms come first, so a cancelled download
   is still useful. Downloading again resumes and only fetches what is missing.

**DELETE ALL** removes the downloaded tiles. **CHANGE** picks another folder,
for example a USB stick.

| Radius | Map zooms | Elevation zoom | About |
|---|---|---|---|
| 10 km | 8–16 (≈2 m/px) | 13 | 4 200 tiles, ~100 MB |
| 25 km | 8–15 (≈4 m/px) | 13 | 6 900 tiles, ~170 MB |
| 50 km | 8–14 (≈8 m/px) | 12 | 6 800 tiles, ~170 MB |

Sizes vary with the terrain: city satellite tiles are bigger than desert ones.

**Without the app** (another computer, or preparing the data in advance), use
`tools/fetch_offline.py`. It needs only Python 3, nothing else:

```sh
tools/fetch_offline.py --lat 32.0853 --lon 34.7818 --radius-km 25 --out ./offline   # add --dry-run for the estimate
```

Copy the resulting folder to `~/.local/share/mavlink_gcs_ui/offline/` on the
field computer, or point Settings → OFFLINE → CHANGE at it. The folder is a
plain slippy-tile tree, read before the download cache and the network:

```
offline/osm/{z}/{x}/{y}.png            streets      tile.openstreetmap.org/{z}/{x}/{y}.png
offline/esri_imagery/{z}/{x}/{y}.jpg   satellite    server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}
offline/elevation/{z}/{x}/{y}.png      elevation    s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png
```

- **Any tile source in this layout works,** for example tiles exported from
  QGIS or MOBAC. Each layer accepts `.png`, `.jpg`, `.jpeg`, `.webp` or `.img`.
- **Elevation tiles are "terrarium" PNGs:** height in metres =
  `R·256 + G + B/256 − 32768`. The lookup uses zoom 13 and falls back to zooms
  12 … 9 when 13 isn't there.
- **Tile providers have terms.** OpenStreetMap's tile policy forbids heavy bulk
  downloading, and Esri imagery has its own terms. Keep areas modest, or use a
  provider you have rights to: set `map/streets_url`, `map/satellite_url` or
  `map/elevation_url` in the settings file (templates with `{z}` `{x}` `{y}`).
  `fetch_offline.py` takes the same as `--streets-url` etc.

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

The left column holds, top to bottom:

- **AIRCRAFT:** roll, pitch and heading, with a small attitude ball.
- **GIMBAL · LOS** (or FIXED MOUNT): pan and tilt **relative to the aircraft**,
  with the earth-frame azimuth and elevation underneath.
- The **HUD RULERS** switch (LOS / BODY).
- **WIND** (tap: relative to the nose, or north-up).

The flight-path marker (circle with three ticks) is where the aircraft is
actually going. Its track is predicted every frame along with the heading, so it
stays steady in turns; in wind it sits beside the nose by the crab angle.

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
top left, with a **wind gauge** under it: north-up, the arrow shows where the
wind blows to, with the direction it comes from, its speed, and the head/cross
components for the current heading (tap either to shrink both). The camera HUD
has the same gauge, relative to the nose by default; tap it to switch to
north-up.

**Resume Plan** rejoins the loop at the **nearest waypoint** (the next one if
the aircraft is already on top of the nearest), not where it left off.

Map tiles (OpenStreetMap / Esri World Imagery) are cached under
`~/.cache/mavlink_gcs_ui/tiles`. The map works offline with whatever is
cached.

## Layout

```
src/core/   pure C++: vehicle state, ArduPlane modes, geo + camera ray casting, mission builder
src/link/   UDP worker thread, MAVLink decode, mission transfer state machine, command sender
src/ui/     Qt widgets: view host/PiP, map, camera HUD, telemetry bar, dock, effects, tiles, video
tests/      unit tests for core + link (no Qt, no network)
tools/      sitl.sh (SITL at your location), fetch_offline.py (offline map data without the app)
resources/  app icon PNGs (from `mavlink_gcs_ui --export-icon DIR`) and the .desktop entry
```

`cmake --install build` installs the binary, the desktop entry and the icon.
