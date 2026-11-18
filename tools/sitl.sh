#!/usr/bin/env bash
# Start ArduPlane quadplane SITL for this GCS, at your location.
#
#   tools/sitl.sh [--no-geo] [extra sim_vehicle.py args...]
#
# Location, first match wins:
#   1. GCS_SITL_LOCATION=lat,lon,alt,heading   (e.g. 32.0853,34.7818,17,0)
#   2. your public IP's approximate location (ipinfo.io, then ip-api.com);
#      ground altitude from api.open-meteo.com. This sends your IP address to
#      those services; --no-geo skips it.
#   3. Israel (Tel Aviv area).
# ARDUPILOT (default ~/ardupilot) points at the ArduPilot checkout.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ardupilot="${ARDUPILOT:-$HOME/ardupilot}"
fallback="32.0853,34.7818,17,0"   # Tel Aviv

geo=1
args=()
for a in "$@"; do
    if [[ "$a" == "--no-geo" ]]; then geo=0; else args+=("$a"); fi
done

is_num() { [[ "$1" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; }

locate() {
    local lat="" lon="" loc
    loc="$(curl -fsS --max-time 4 https://ipinfo.io/loc 2>/dev/null || true)"
    if [[ "$loc" == *,* ]]; then
        lat="${loc%%,*}"; lon="${loc##*,}"
    else
        loc="$(curl -fsS --max-time 4 'http://ip-api.com/line/?fields=lat,lon' 2>/dev/null || true)"
        lat="$(sed -n 1p <<<"$loc")"; lon="$(sed -n 2p <<<"$loc")"
    fi
    lat="$(tr -d '[:space:]' <<<"$lat")"; lon="$(tr -d '[:space:]' <<<"$lon")"
    is_num "$lat" && is_num "$lon" || return 1
    local alt
    alt="$(curl -fsS --max-time 4 "https://api.open-meteo.com/v1/elevation?latitude=$lat&longitude=$lon" 2>/dev/null \
           | sed -n 's/.*"elevation":\[\([-0-9.]*\)\].*/\1/p')"
    is_num "${alt:-x}" || alt=0
    echo "$lat,$lon,$alt,0"
}

if [[ -n "${GCS_SITL_LOCATION:-}" ]]; then
    location="$GCS_SITL_LOCATION"; why="GCS_SITL_LOCATION"
elif [[ $geo == 1 ]] && location="$(locate)"; then
    why="your approximate location (from your IP)"
else
    location="$fallback"; why="default (Israel)"
fi
echo "SITL start: $location  [$why]"

cd "$ardupilot"
# --streamrate=-1: MAVProxy otherwise forces every stream to 4 Hz, overriding
# the rates the GCS asks for (the HUD then moves in visible steps)
exec Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane -l "$location" \
    --out=udp:127.0.0.1:14551 --add-param-file="$here/sitl_extra.parm" \
    --mavproxy-args="--streamrate=-1" "${args[@]}"
