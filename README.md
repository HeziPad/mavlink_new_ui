# QuadPlane GCS

A minimalist, game-style ground control station for an ArduPilot **QuadPlane**
(4 lift rotors + pusher), talking MAVLink v2 over UDP or a serial port (a Pixhawk on USB, or a
telemetry radio). It has a full-bleed map or
camera with a swappable picture-in-picture, a compact telemetry strip, and five
buttons. It is touch-first, with the mouse as an alternative.

C++17 · Qt 6 Widgets · MAVLink `c_library_v2` · GStreamer (optional, for video).

## Install, build and run

The app runs on **Linux, Windows 10/11 and macOS 12+**, and connects to a real Pixhawk over USB, a
telemetry radio, or the network (see [Connecting to the aircraft](#connecting-to-the-aircraft)).
Video needs GStreamer; without it the app still builds and runs, and the camera view shows the
synthetic HUD only. Every push is built and tested on all three systems by GitHub Actions
(`.github/workflows/build.yml`). The packaged app of each run (ZIP / DMG / tar.gz) can be
downloaded from the run's page.

### Linux (Ubuntu / Debian)

```sh
sudo apt install cmake ninja-build g++ qt6-base-dev zlib1g-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-libav gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build                    # all suites (geo, mission builder, mission transfer, transport, ...)
./build/mavlink_gcs_ui
```

- **GStreamer packages:** `gstreamer1.0-libav` provides the H.264 decoder. `-ugly` is only needed
  for the `x264enc` test stream below.
- **Serial port access:** opening the Pixhawk's port (`/dev/ttyACM0`) or a radio's
  (`/dev/ttyUSB0`) needs the `dialout` group: `sudo usermod -aG dialout $USER`, then log out and
  back in.
- **ModemManager:** if it grabs the port, run `sudo systemctl disable --now ModemManager`.
- **Installing:** `cmake --install build` installs the binary, the desktop entry and the icon.
  `cd build && cpack` makes a `.tar.gz`.

### Windows 10 / 11

1. **Install the tools:**
   - [Visual Studio 2022 Build Tools](https://visualstudio.microsoft.com/downloads/), with the
     "Desktop development with C++" workload. It includes CMake and Ninja.
   - Qt 6.5 or newer from the [Qt online installer](https://www.qt.io/download-qt-installer), with
     the component **MSVC 2022 64-bit**.
   - Optional, for video: the GStreamer **MSVC 64-bit runtime and development** installers from
     [gstreamer.freedesktop.org](https://gstreamer.freedesktop.org/download/). Choose "Complete" and
     add its `bin` folder to `PATH`.
2. **Build and package.** In the **x64 Native Tools Command Prompt for VS 2022**:
   ```bat
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=C:\Qt\6.9.3\msvc2022_64
   cmake --build build
   ctest --test-dir build
   cmake --install build --prefix dist
   ```
   - **No video:** add `-DGCS_WITH_GSTREAMER=OFF`.
   - **zlib:** CMake downloads and builds zlib itself.
   - **Result:** `dist\bin` holds `mavlink_gcs_ui.exe` with the Qt DLLs next to it, so you can copy
     the folder to another PC. `cd build && cpack -C Release` makes a ZIP of it.
3. **Run** `dist\bin\mavlink_gcs_ui.exe`.
   - **Firewall:** the first time it listens on the network, Windows Firewall asks. Allow it for
     UDP links; serial links don't need it.
   - **Pixhawk on USB:** it appears as a COM port with no driver needed. Device Manager → Ports
     shows which.
   - **SiK radios:** these need the FTDI or Silicon Labs CP210x driver.
   - Then use Settings → CONNECTION → SERIAL, or `mavlink_gcs_ui.exe --serial COM4:57600`.
   - Log messages (`GCS_FPS=1` and the like) go to the debugger, not a console.

### macOS 12+ (Apple Silicon or Intel)

```sh
xcode-select --install
brew install cmake ninja qt            # + gstreamer for video
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$(brew --prefix qt)
cmake --build build
ctest --test-dir build
cmake --install build --prefix dist    # dist/mavlink_gcs_ui.app, with the Qt frameworks inside
open dist/mavlink_gcs_ui.app
```

- **No video:** add `-DGCS_WITH_GSTREAMER=OFF`.
- **Disk image:** `cd build && cpack` makes a `.dmg`.
- **First start:** the app is not signed, so right-click it → **Open** once to get past Gatekeeper.
  When macOS asks about **Local Network**, allow it; UDP telemetry and video from a radio or the LAN
  need it.
- **Serial ports** are `/dev/cu.usbmodem…` for a Pixhawk on USB and `/dev/cu.usbserial-…` for a SiK
  radio. FTDI and CP210x drivers are built in. The CONNECTION page lists them.
  - From the command line: `dist/mavlink_gcs_ui.app/Contents/MacOS/mavlink_gcs_ui --serial /dev/cu.usbmodem1101`.
- **Zooming the map:** pinch on the trackpad, or scroll with two fingers.

### Fonts

The app bundles DejaVu Sans and DejaVu Sans Mono (`resources/fonts`, Bitstream Vera licence), so
text looks and measures the same on every system.

## Run

```sh
./build/mavlink_gcs_ui                      # the link chosen in Settings > CONNECTION (first run: udp 127.0.0.1:14551)
./build/mavlink_gcs_ui --udp 0.0.0.0:14550 --video-port 5600 --reduced-motion
./build/mavlink_gcs_ui --serial /dev/ttyACM0            # Pixhawk on USB
./build/mavlink_gcs_ui --serial /dev/ttyUSB0:57600      # SiK telemetry radio
```

### Connecting to the aircraft

Open ⚙ Settings → **CONNECTION**, choose the link, and press **CONNECT**. It switches live, with no
restart. The line beside the button says what the link is doing (CONNECTED, waiting for the
aircraft, port not found…). Changing the link while armed asks first.

- **SERIAL · USB / RADIO:** a Pixhawk on a USB cable, or a telemetry radio (SiK, RFD900). Pick the port from
  the list, which updates as things are plugged in; autopilots are listed first and marked.
  - **Baud:** 57600 for SiK radios. A USB cable ignores it.
  - **Unplugging:** if the port disappears (USB unplugged, or the autopilot rebooting), the GCS
    reopens it once a second until it is back.
  - **Permissions (Linux):** opening the port needs the `dialout` group:
    `sudo usermod -aG dialout $USER`, then log in again. If ModemManager grabs the port, run
    `sudo systemctl disable --now ModemManager`.
- **UDP · NETWORK:** WiFi/Ethernet radios, Herelink, a companion computer, MAVProxy, SITL.
  - **LISTEN ON:** the address and telemetry port to listen on. `0.0.0.0` means all networks;
    `127.0.0.1` means this computer only. The GCS replies to whichever address the vehicle sends from.
  - **SEND TO** (optional): for radios or companions that wait for the GCS to talk first.
- **VIDEO PORT:** the UDP port of the RTP/H.264 camera stream.

Addresses and ports are typed on the entry pad. In an address, `.` moves to the next number.
`--udp` / `--serial` / `--video-port` override the saved link for one run; what you CONNECT in the
app is saved.

If it feels slow, run it with `GCS_FPS=1`. Once a second it prints how often it
redrew, its longest stall, and the attitude/position telemetry rates. That
tells a slow UI apart from slow telemetry. Through MAVProxy, use
`--streamrate=-1`; otherwise MAVProxy forces every stream to 4 Hz.

### With SITL (Linux)

```sh
tools/sitl.sh                 # ArduPlane quadplane SITL at your location, output to 127.0.0.1:14551
./build/mavlink_gcs_ui
```

`tools/sitl.sh` starts `sim_vehicle.py` (checkout in `$ARDUPILOT`, default
`~/ardupilot`) with `tools/sitl_extra.parm`. That file sets safety ON at boot,
a 30 Ah simulated battery (about 2 h of flight: cruise draws ~12.6 A), a
battery monitor, and the failsafe settings below. The start location comes from:

1. `GCS_SITL_LOCATION=lat,lon,alt,heading`, if set;
2. otherwise your public IP's approximate location (ipinfo.io, falling back to
   ip-api.com), with the ground altitude from Open-Meteo. **This sends your IP
   address to those services.** `tools/sitl.sh --no-geo` skips it;
3. otherwise Israel (Tel Aviv, `32.0853,34.7818,17,0`).

Any other arguments go to `sim_vehicle.py` (e.g. `tools/sitl.sh --speedup 2`).

### With a NextVision TRIP camera (simulated)

`../nextvision_mavlink_sim` simulates a NextVision TRIP gimbal camera. It is wired like the real one, between the
autopilot (SITL SERIAL2) and this GCS:

```sh
../nextvision_mavlink_sim/scripts/sitl_trip.sh --video   # SITL + TRIP; the TRIP talks to 127.0.0.1:14551
./build/mavlink_gcs_ui
```

What you get:
- **Flying:** the vehicle is reached through the TRIP's bridge.
- **PAYLOAD chip** (next to HUD on the video): opens the camera panel.
  - Live decoded reports.
  - Every ICD command, with editable parameters.
  - Quick modes and sensors.
- **LOS on the map and HUD:** the camera's line of sight and footprint come from the TRIP's own reports.
- **Tap-to-track:** tapping the video (when not guiding) starts the TRIP's tracker on that spot.

Other payload ICDs work the same way. Drop a JSON file in `icds/` (see `icds/README.md`).

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
| 1 | **Plan / Sync ↑** | First press: edit the mission on the map. Second press: upload it; if the aircraft is flying the plan (AUTO), it switches to the new plan right away at its nearest waypoint. **Hold**: re-download the vehicle's mission |
| 2 | **Takeoff / Launch / Resume Plan** | On the ground (**hold 1 s** to confirm): GUIDED + arm + vertical takeoff. At the takeoff altitude it flies a **straight climb-out leg** (300 m, into the wind when there is some, else straight ahead) for the transition, and only once it is wing-borne does it turn to the plan (AUTO from WP1), or Hold when there is no mission. While the safety switch is ON it reads **Safety Off** (padlock): the first hold turns the safety off (motors live), and the button then becomes Takeoff / Launch for the second hold. Airborne: **Resume Plan** goes back to the planned loop on the wing (AUTO), also mid-landing (aborts it). If the vehicle's current item isn't a loop waypoint (still the VTOL takeoff after a Hold during the climb, or the landing), it is reset to the loop first, so AUTO never re-runs a rotor takeoff. While flying the loop it reads **On Plan** |
| 3 | **Fly To** | One target per press: press it, then tap the map (or the ground in the camera view) to circle there on the wing (GUIDED). The button then goes off, so a new target needs a new press |
| 4 | **Hold** | Instant. Circle where the aircraft is, on the wing (GUIDED). During a landing this is a go-around |
| 5 | **Land** | **Hold 1 s**. Circle down beside the landing point **L** on the wing, leave into the wind, back-transition and land vertically. Without an L it lands at home the same way |

**Hold and Fly To circles:** while holding (press **Hold** again to hide or show it), or while the **Fly To** button is on (press it again to adjust the circle it is flying), a panel above the dock sets the
**altitude** (−10 / −1 / +1 / +10 m, or −50 … +50 ft; 20–5000 m), the
**radius** (−50 … +50 m, 40–5000 m) and the **direction** (CW / CCW). Holding a
button repeats and speeds up (×5 after 1 s, ×10 after 2.5 s). Changes apply
live. The circle starts at the vehicle's `WP_LOITER_RAD`. Turn Fly To off to hide
the panel; the circle keeps flying.

ArduPlane flies a loiter **bigger than commanded**: `AP_L1_Control` scales the
radius by EAS2TAS² (about +7 % at 700 m above sea level, more at higher fields,
with `NAVL1_LIM_BANK` = 0), and the L1 loop overshoots small circles (SITL: 80 m
flew as 91 m). The GCS measures each lap and trims the radius it sends, so the
flown circle matches the dotted one. The panel shows `≈ flown 91` while it
trims and `✓ flown 80` once it matches. The readout is live (the last lap's
worth of samples, from a quarter lap on); the trim itself acts on full laps. Both use `DO_REPOSITION`, so the vehicle shows
GUIDED. In AUTO the altitude comes from the plan: edit the waypoint instead.
The ALT chip shows the commanded altitude as `60→80` while the aircraft climbs
or descends to it (what the GCS asked for, so it doesn't drift during the climb).

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

The **telemetry bar** keeps to the essentials, left to right: **LINK** (signal
bars only: green when strong, amber when weak or stale, a red ✕ when the link
is lost), **BATTERY** (gauge and %, green / amber under 40 % / red under 20 %),
**TIME LEFT** (battery flying time, h:mm), **FLIGHT**, **ETA**, **ALT**,
**SPEED**, **SYSTEMS** and the ⚙. GPS problems show under SYSTEMS, and armed
shows in the FLIGHT chip ("ARMED · ON GROUND").

**SYSTEMS** in the telemetry bar counts problems. Tap it (or the red alert
banner) to open the alerts panel; tap it again, tap anywhere outside the panel
to close it. It shows status pills (link, GPS, navigation, sensors,
power, arming) and one card per problem. Cards show their **title only**; tap a
title to open it (critical ones arrive open). An open card says what's wrong,
why it matters and what to try, with the raw data (EKF flags, the autopilot's
message) in small print. The cards come from the EKF status, `SYS_STATUS` sensor
health, GPS, battery and the link, plus the autopilot's PreArm and failsafe
messages in plain language. PreArm cards stay while ArduPilot keeps repeating
them (every 30 s) and clear when the vehicle arms. The panel only explains: it
never changes the vehicle.

The **FLIGHT MODE** chip says in plain words what the aircraft is doing: ON GROUND,
TAKEOFF, TRANSITION, CLIMB-OUT, ON PLAN · WP3, END HOLD, HOLD, FLY TO,
LANDING · APPROACH / VERTICAL, RTL, QLAND (other modes by their ArduPilot name). The **ETA** chip beside it shows what
it is flying to and when it gets there. A rotor glyph lights while the lift
rotors spin, and "· ROTOR ASSIST" flags Q_ASSIST in cruise.

The **ALT** chip switches reference on a tap: **ASL** (sea level), **AGL**
(the ground below: the rangefinder when its reading is valid, i.e. pointing
down, fresh, inside its range and not at its maximum, the aircraft not banked
or pitched over 30°; otherwise the terrain map) and **AFL**
(above the landing point's ground). Altitudes in the bar are rounded to 10 (m
or ft): 323 shows 320, 328 shows 330, and a value hovering at 325 doesn't
flicker (it moves on 2 past the boundary).

The **SPEED** chip has no numbers: it shows the speed the GCS commands in words
(**LOITER**, **CRUISE**, **MAX**) and the speed mode above it (**ECO**, **NORMAL**,
**FAST**, as one to three chevrons). The numbers are on the map, in the **SPEED**
box under WIND (air and ground speed). The ATTITUDE, WIND and SPEED boxes fold
together: tap any of them (folded, SPEED shows the airspeed alone). The SPEED
chip carries a small ▾, like the ALT chip's cycle mark: tap it for the speed mode.

The map's **nav box** (bottom right) shows the aircraft's coordinates and the
ground below it, the target it is flying to (WPn, APPROACH, L, HOLD, FLY TO,
END HOLD, CLIMB-OUT) with its distance (the ETA is on the ETA chip), and, under a mouse cursor, the
cursor's coordinates, ground height, distance and ETA. The camera view shows
the **CAMERA TARGET** (top of the left column, above AIRCRAFT): where the line of sight meets the
terrain, its ground height and the slant range (≈ when no terrain tile is
there and flat ground at home level is used).

The **⚙** at the end of the bar opens **Settings**: altitude in metres or feet,
speeds in m/s or knots, coordinates as decimal degrees, D° M′ S″ or UTM, the
**text size** (DISPLAY: Small, Medium 125 %, Large 150 %, applied at once; labels,
numbers and the boxes, buttons and panels around them grow, while the map and camera
picture keep their size, and the top bar grows only as far as it still fits the
screen width), the flight trail (on/off, last 30 s / 1 / 2 / 5 / 10 min / all), the
**landing reserve** for the battery time left (15–35 %, default 20 %), and **offline maps**
(see below).

**Mouse over the map** shows the point under the cursor, bottom right:

- its latitude and longitude;
- the **ground height** above sea level, and above or below home;
- its distance and bearing from the aircraft.

Ground height comes from SRTM elevation data (AWS Terrain Tiles, about ±10 m).
It measures the surface, so it includes roofs and tree tops.

**Safety switch** (`MAV_CMD_DO_SET_SAFETY_SWITCH_STATE`): it is only needed
for takeoff, so it lives on the **Takeoff / Launch** button. While safety is ON
(motor outputs cut) the button reads **Safety Off**: hold it 1 s and the motors
go live, then hold it again to take off. **After every landing the GCS turns
safety back ON** as soon as the vehicle disarms on the ground (never while
armed: that would cut the pusher and servos too). The preflight screen can
also switch it. **Safety is ON after every power-up** (`BRD_SAFETY_DEFLT = 1`,
checked and fixed by preflight, applied after a reboot), so turning it off is
always a deliberate step.

After an RTL / QLAND landing (a failsafe) the vehicle sits in a mode ArduPlane can't
arm in, so its pre-arm check fails. SYSTEMS shows that as an info card, not a
fault: Launch switches mode before arming.

**The keyboard has no shortcuts.** Everything is done by tapping; a physical
keyboard only types into the manual-entry keypad (coordinates, altitudes, hold
time): digits and `.` into the focused box, Backspace / Delete, ← / → to move
the cursor, Tab for the next box, Enter = OK.

**The lift rotors are for takeoff and landing only.** Every flight action above
keeps the aircraft on the wing. The rotor glyph on the **FLIGHT** chip lights
while the lift rotors spin, and the chip adds **ROTOR ASSIST** when they spin
in wing-borne flight. ASSIST means the rotors spun up in wing-borne flight to prevent a stall
(`Q_ASSIST_SPEED` and friends), and it also raises a toast. The aircraft stays
armed in cruise: disarming would stop the pusher too. The GCS reads
`Q_GUIDED_MODE` at connect and shows an alert if it is 1, because Fly To would
then end in a rotor hover.

**Planning:** **ROUTE** in the plan strip picks how the waypoints are flown,
until you press Hold or Land:

- **⟳ Loop** (default): 1 → 2 → 3 → 1 → … forever.
- **⇄ Back & forth**: 1 → 2 → 3 → 2 → 1 → 2 → … forever. With two
  waypoints this is the same flight as the loop.
- **→ One way, then hold**: 1 → 2 → 3 once, then it circles WP3 on the wing.
  Select the last point to set the hold circle's **radius** (−50 / −10 / +10
  / +50 m; default `WP_LOITER_RAD`) and **direction**. While the aircraft is
  circling there in flight, the same panel as Fly To (**HOLD · END**) changes
  its radius, direction and altitude live. The panel re-uploads the mission
  and keeps circling the same point.

The map draws a loop closed, the other two open, and marks the last point
TURN BACK or HOLD.

- **Map:** tap to add a waypoint, drag to move one, pinch to zoom.
- **Editing a point:** tap a point to select it. A panel then lets you set
  its altitude (−10 / −1 / +1 / +10, hold to repeat), insert a point after
  it, or delete it. Tap empty map to deselect.
- **Typing a position or an altitude:** **⌖ COORD** on the point's panel (or
  **⌖ BY COORD** in the plan strip for a new point, and on the Fly To panel)
  opens a keypad in the coordinate format chosen in Settings → UNITS (no
  switching there). Every number has its own box: `LAT [32.085300]° [N|S]`,
  `LAT [32]° [05]′ [07.1]″ [N|S]`, or UTM `ZONE [36] [NORTH|SOUTH]`,
  `EASTING`, `NORTHING`. Hemispheres are buttons, never typed letters. Each
  box shows a thick cursor: tap a box (or a spot in it) to move there, and use
  ◀ / ▶ / **NEXT ▸**. Digits and ⌫ act at the cursor, and a full box moves on
  to the next one. It shows the point in the other formats and its distance
  from the aircraft, and refuses anything more than 50 km from home. Tap an **altitude number** (waypoint,
  landing point, Hold / Fly To / end-hold panel) to type it in **ASL**,
  **AGL** or **AFL**. It is checked against the ground at that point: at least
  **30 m above the ground** (20 m for the landing approach), with a **USE
  MINIMUM** button when lower. The ± steppers stop at the same floor.
- **Holding at a waypoint:** a waypoint's **HOLD HERE** row sets how long to
  circle it before going on, the circle's radius and its direction. Tap the
  time to type it (boxes `[1] min [30] s`, with FLY THROUGH / 0:30 / 1:00 /
  2:00 / 5:00 / 10:00 chips).
  - The hold is always set as a **time**; the row shows **Radius: 80 m**
    (starting at `WP_LOITER_RAD`), stepped ±10 / ±50 m.
  - **The GCS times the hold.** The timer starts once the aircraft is on the
    circle. When it runs out, the GCS sends the mission on to the next point
    (`SET_MISSION_CURRENT`; through a `DO_JUMP` to its target), so the hold
    is exactly as long as set. The FLIGHT chip counts it down evenly
    ("HOLD 0:18"), and the ETA chip reads **HOLDING**.
  - On board, the hold is only a backstop for when the link is lost. At the
    default radius it is `NAV_LOITER_TIME`. With **its own radius** it is
    `NAV_LOITER_TURNS` (ArduPlane keeps a radius only for turns: exact up to
    255 m, 10 m steps up to 2550 m) with enough turns to cover the time plus
    one. Without the link, the hold ends up at most about a lap longer.
  - Turns don't store a time, so the GCS remembers the times it uploaded (also
    across restarts). The map and the plan editor show the time you set, not
    0. A mission from elsewhere shows turns × lap time.
- **Landing point:** tap **+ LANDING** in the plan strip, then tap the map.
  The landing point is optional and separate from the loop. Its altitude
  (default 30 m, editable in the panel) is the **approach altitude**: the
  aircraft circles down to it on the wing and only then switches to the lift
  rotors for the vertical descent.
- **Mouse extras:** the wheel over a point changes its altitude by 5 m, and
  right-click deletes the point.

The upload is built as:

```
HOME → NAV_VTOL_TAKEOFF (40 m) → route → DO_LAND_START
     → NAV_LOITER_TO_ALT (approach circle beside L) → NAV_VTOL_LAND at L

route, Loop:          WP1 … WPn → DO_JUMP(WP1, forever)
route, Back & forth:  WP1 … WPn, WPn-1 … WP2 → DO_JUMP(WP1, forever)
route, One way:       WP1 … WPn-1 → NAV_LOITER_UNLIM at WPn
```

With a single waypoint, the aircraft circles it (`NAV_LOITER_UNLIM`) whatever
the route. The mission never reaches the landing by itself.

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
| Settings | 18 parameter checks (Q_GUIDED_MODE, weathervaning into the wind, Q_ASSIST_SPEED, airspeed limits, arming checks, RC/GCS/battery failsafes and voltages for your pack, fence, loiter radius vs bank limit, ...), each with the reason and a one-tap fix (confirmed, read back, logged) where there is a safe value |
| Mission | On the vehicle, landing point, altitudes, range, laps per pack |
| Ready | The GCS selects AUTO, resets the mission to its takeoff and runs the pre-arm checks (their messages are listed if any fail); then safety OFF and **READY TO LAUNCH** |

Tests that move or spin anything need a 1 s hold, have a **STOP ALL** button,
and have vehicle-side timeouts as well. A pass stays valid until the autopilot
reboots, the battery is swapped, a parameter is changed elsewhere, an hour
passes, or it flies; after a flight a short **turnaround** preflight is offered.
Every run is saved as JSON under `~/.local/share/mavlink_gcs_ui/mavlink_gcs_ui/preflight/`.

## Offline maps

The map works **offline only** by default: nothing is fetched while you
browse. You choose what to download in Settings ⚙ → **OFFLINE**. Settings →
OFFLINE → **ONLINE TILES: ON** fetches missing satellite while browsing, if
you want that.

| Layer | Where it comes from | Coverage |
|---|---|---|
| **Map** (streets) | OpenStreetMap vector data (Protomaps daily build, ODbL), drawn by the app in the HUD's style | the **country pack**: Israel + ~50 km, every zoom, sharp to the deepest |
| **Sat** | Esri World Imagery | **country pack**: zoom 8–14 (≈10 m/px); **full-detail areas** you draw: zoom 15–18 (≈0.5 m/px) |
| Ground height (hover) | AWS Terrain Tiles (SRTM) | country: zoom 12 (≈30 m); full-detail areas: zoom 13 |

- **Missing satellite is filled in.** Where there's no satellite imagery (outside your areas, or deeper
  than Esri has, which Esri answers with a grey "Map data not yet available" tile), the map shows the
  nearest coarser imagery enlarged, or the street map. The no-data answer is remembered (`{y}.none`),
  so it isn't fetched again.
- **Street labels:** Hebrew or English, under Settings → MAP → MAP LABELS.

**Country pack.** Settings → OFFLINE → **DOWNLOAD COUNTRY**:

1. The street map: about 175,000 tiles, about 235 MB, about a minute. Only the index pieces and tile
   ranges covering the area are read from the 140 GB planet file.
2. Then elevation and satellite: about 46,000 tiles, about 1 GB.

Afterwards the button reads **CHECK & COMPLETE**. It fetches only what's new or missing:

- **The street map:** only if a newer daily build exists than the one installed (recorded in
  `map.build`). It's a ~1 s check otherwise.
- **The country's satellite and ground height:** missing tiles only.
- **Your full-detail areas:** their missing tiles, for example ones that failed before.

Nothing already on disk is downloaded again.

**Full-detail areas.** Settings → OFFLINE → **SELECT AREA ON MAP**:

- Drag a rectangle with a finger or the mouse, and drag a corner to adjust it.
- The panel shows its size, the **new tiles** and the tiles **already on disk** before you press
  **DOWNLOAD**. Areas you already have show as dashed outlines.
- Each tile is stored once, by its position, so overlapping areas share tiles and only the missing ones
  are downloaded.
- **CLEAR FULL DETAIL** removes all full-detail satellite and ground height after an "are you sure?".
  The country pack and the street map stay. **DELETE ALL** asks the same way.

**Without the app** (another computer, or preparing in advance), `tools/fetch_offline.py` does the same
and needs only Python 3:

```sh
tools/fetch_offline.py --country israel --out ./offline              # street map + country satellite + ground height
tools/fetch_offline.py --country israel --only vector --out ./offline
tools/fetch_offline.py --bbox 34.75,32.05,34.82,32.12 --out ./offline  # a full-detail area (min_lon,min_lat,max_lon,max_lat)
tools/fetch_offline.py --country israel --out ./offline --dry-run    # sizes only
```

The street map can also be cut with the official PMTiles tool:
`pmtiles extract https://build.protomaps.com/<YYYYMMDD>.pmtiles map.pmtiles --bbox=33.7,29.0,36.5,33.8`
(the dates are listed at `https://build-metadata.protomaps.dev/builds.json`).

Copy the folder to `~/.local/share/mavlink_gcs_ui/mavlink_gcs_ui/offline/` (the app name appears twice) on the field computer, or point
Settings → OFFLINE → CHANGE at it. Its layout:

```
offline/map.pmtiles                     street map (any *.pmtiles with vector tiles is used)
offline/esri_imagery/{z}/{x}/{y}.jpg    satellite        ({y}.none: no imagery at that zoom)
offline/elevation/{z}/{x}/{y}.png       ground height    ("terrarium": m = R*256 + G + B/256 - 32768)
offline/map.build                       which street map build is installed
offline/areas.json                      the full-detail areas (outlines on the map)
```

- **Other sources work too.** Any slippy-tile source in that layout works for satellite (e.g. tiles
  exported from QGIS or MOBAC). Each layer accepts `.png`, `.jpg`, `.jpeg`, `.webp` or `.img`.
- **Tile providers have terms.** Esri imagery and others have their own terms; OpenStreetMap's tile
  server refuses bulk downloads, which is why the street map comes from the vector data instead.
- **Online URLs are configurable.** `map/satellite_url`, `map/elevation_url` and `map/streets_url`
  (`{z}` `{x}` `{y}` templates) in the settings file change where online tiles come from.

## Settings

The in-app settings screen (⚙) switches altitudes between metres and feet
(only altitudes: HUD tape, telemetry bar, map labels, panels and steppers), speeds
between m/s and knots (AS/GS, HUD, wind), and coordinates between decimal
degrees, D° M′ S″ and UTM (WGS84). Distances and sizes (such as the camera's
measuring brackets) stay metric. `core/coords` also
parses those formats, for coordinate inputs.

`~/.config/mavlink_gcs_ui/mavlink_gcs_ui.conf` is created on first run.
You can edit the link (`link/type` udp|serial, `link/host`, `link/port`, `link/remote_host`,
`link/remote_port`, `link/serial_port`, `link/serial_baud`) and the video port, takeoff, default waypoint and landing approach altitude,
cruise speed (used for the lap time), camera HFOV, the fixed-mount tilt used
when no gimbal reports its angles, and reduced motion. The map
layer and follow state, the altitude unit, the camera HUD reference and the
map attitude ball (open or collapsed) are remembered automatically. Command-line options
override the file for one run.

## Camera view

The video comes first. By default (**CUSTOM**, as first installed) it shows only a crosshair, the
heading tape, a slim status line and the **CAMERA TARGET** box. The AIRCRAFT,
LOS and WIND boxes are folded to one line each, and there is no ladder, roll
arc, tapes, vignette or scanlines.

**HUD panel.** Tap **HUD ▾** (top left, under the bar) to open the HUD LOADOUT
panel on the right. Every change shows live on the video behind it, and is
remembered:

- **Presets:** CLEAN (crosshair only), CUSTOM (your own mix) and FULL
  (everything). Your edits are saved in CUSTOM: try CLEAN or FULL and tap
  CUSTOM to get your mix back. Turning something on or off under CLEAN or FULL
  starts a new CUSTOM mix from what is on screen. A preset keeps your
  crosshair, line width and colour.
- **Show on the video:** pitch ladder, roll arc, heading, speed and altitude
  tapes, flight-path marker, status line, the target / aircraft / camera LOS /
  wind boxes, and SENSOR FX (vignette + scanlines).
- **HUD rulers follow:** LOS or BODY (see below).
- **Crosshair:** cross, dot, ring, scope (duplex), box, fine (full-width lines),
  measure, or none.
- **Line width** (thin / normal / bold) and **colour** (green, cyan, amber,
  white, red, magenta). Warnings stay amber and red whatever the colour.

**Folding boxes.** Tap any box in the left column to fold it to a one-line
summary (e.g. `ACFT R+2° P−1° 003°`, `TARGET 820 m`), and tap again to open it.
On the open WIND box, the header folds it and the gauge switches north-up.

**Measuring.** The **MEASURE** crosshair is four corner brackets around the
centre. To fit them to a target:

- drag a corner (both sides) or an edge (one side);
- pinch with two fingers, use a trackpad pinch, or turn the mouse wheel.

Under the brackets it reads the target's size at the range under the
crosshair, e.g. `W 4.2 m × H 2.1 m @ 820 m`, always in metres. It is approximate:

- It assumes the target faces the camera.
- It uses the camera field of view from the settings, so that must match the lens.
- The range comes from the terrain map under the crosshair. With no terrain
  data it uses flat ground at home level and shows `≈` in amber.
- Above the horizon or beyond 3 km there is no range, and it shows the angles instead.

The horizon, elevation ladder, roll scale and heading tape follow either the
camera's **line of sight (LOS)** or the **aircraft body**. Switch it in the HUD
panel (**HUD RULERS FOLLOW**).

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

The left column holds, top to bottom (each can be hidden or folded):

- **CAMERA TARGET:** where the crosshair meets the ground, its height and the slant range.
- **AIRCRAFT:** roll, pitch and heading, with a small attitude ball.
- **GIMBAL · LOS** (or FIXED MOUNT): pan and tilt **relative to the aircraft**,
  with the earth-frame azimuth and elevation underneath.
- **WIND** (tap: relative to the nose, or north-up).

The flight-path marker (circle with three ticks) is where the aircraft is
actually going. Its track is predicted every frame along with the heading, so it
stays steady in turns; in wind it sits beside the nose by the crab angle.

In Fly To mode, a tap on the video is cast along the real LOS to find the
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

**Resume Plan** rejoins the route at the **nearest waypoint** (the next one if
the aircraft is already on top of the nearest), not where it left off. On a
back-and-forth it keeps the direction the aircraft is flying. On a one-way
route it never wraps back to WP1.

**Battery time left.** In flight the **TIME LEFT** chip shows, for example,
**0:14** (h:mm). That's how long the aircraft can keep flying the mission and still have
enough to do all of this, recomputed every 2 s:

- fly back to L (or home), into a wind taken **20 % stronger** than measured;
- circle down the approach, **go around once**, and land on the lift rotors.
  The rotor time covers the transition, the descent and 20 s of extra hover;
- with all of that **× 1.15**, and the **landing reserve** (Settings → BATTERY,
  15–35 %, default 20 % of the pack) left untouched.

It uses the lower of the two charge readings (mAh used and the autopilot's %).
It costs the return from the farthest point of the plan, not only from where
the aircraft is now. The cruise current is learned in flight and the hover
current during the takeoff (4 × cruise until measured).

The shown time is smoothed so a climb or a gust doesn't make it jump. The cruise
current is a 5 min average, and between estimates the shown time runs down like
a clock, moving only part of the way toward each new estimate (3 min time
constant). The critical alert doesn't wait for the smoothing.

- Under 5 min left: amber, with a "head back soon" card.
- At 0:00: **LAND NOW** and a warning card.
- If even a return from here would eat into the reserve: a critical card and
  the red banner.

**Airspeed.** Tap the SPEED chip to see the aircraft's three airspeeds,
read from its own parameters and never changed by the GCS: **LOITER**
(`AIRSPEED_MIN`), **CRUISE** (`AIRSPEED_CRUISE`) and **MAX** (`AIRSPEED_MAX`).
The one choice there is the **SPEED MODE**, which picks the speed per
situation (sent with `DO_CHANGE_SPEED`):

| Mode | Circling | Between points | Long descents (> 15 m) | Landing |
|---|---|---|---|---|
| ECO (default) | LOITER | CRUISE | MAX | CRUISE |
| NORMAL | CRUISE | CRUISE | CRUISE | CRUISE |
| FAST | CRUISE | MAX | MAX | CRUISE |

ArduPlane raises its minimum speed in banked turns by itself (TECS load
factor). LOITER is also kept 2 m/s above `Q_ASSIST_SPEED`, or the lift rotors
would spin to help the wing (SITL's quadplane ships `Q_ASSIST_SPEED` 18 above
`AIRSPEED_MIN` 13, so it loiters at 20 there). The active target shows in the
SPEED chip.

It only warns; it never lands by itself. Tap the chip for the card that
explains the numbers. It needs `BATT_CAPACITY` and a current sensor.

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

`cmake --install build` installs the binary, the desktop entry and the icon on Linux; on Windows and macOS it makes a self-contained folder / `.app` (see [Install, build and run](#install-build-and-run)).
