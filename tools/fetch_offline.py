#!/usr/bin/env python3
"""Download map + elevation tiles for flying the GCS without internet.

Works on any computer with Python 3 (standard library only); the GCS itself
isn't needed. It writes exactly the folder layout the GCS reads:

    OUT/osm/{z}/{x}/{y}.png            streets (OpenStreetMap)
    OUT/esri_imagery/{z}/{x}/{y}.jpg   satellite (Esri World Imagery)
    OUT/elevation/{z}/{x}/{y}.png      ground height (AWS Terrain Tiles, terrarium)

Copy OUT to ~/.local/share/mavlink_gcs_ui/offline/ on the field computer, or
point Settings -> OFFLINE -> FOLDER at it. Files already present are skipped,
so an interrupted run resumes.

Examples:
    tools/fetch_offline.py --lat 32.0853 --lon 34.7818 --radius-km 25 --out ./offline
    tools/fetch_offline.py --lat 32.08 --lon 34.78 --radius-km 10 --layers elevation,satellite --out /media/usb/offline

Mind the tile providers' terms: OpenStreetMap's tile policy forbids heavy bulk
downloads, and Esri imagery has its own terms. Keep areas modest, or pass your
own provider with --streets-url / --satellite-url ({z} {x} {y} placeholders).
"""

import argparse
import math
import os
import sys
import time
import urllib.error
import urllib.request

LAYERS = {
    "streets": ("osm", "png", "https://tile.openstreetmap.org/{z}/{x}/{y}.png"),
    "satellite": ("esri_imagery", "jpg",
                  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"),
    "elevation": ("elevation", "png", "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"),
}
TYPICAL_BYTES = {"streets": 16000, "satellite": 28000, "elevation": 110000}
MAP_MIN_ZOOM = 8
USER_AGENT = "mavlink_gcs_ui-fetch_offline/1.0"


def map_max_zoom(radius_km):
    # same choice as the GCS: about 3000 tiles per map layer
    return 16 if radius_km <= 12 else 15 if radius_km <= 30 else 14


def elevation_zoom(radius_km):
    return 13 if radius_km <= 30 else 12


def tile_xy(lat, lon, z):
    n = 2 ** z
    lat = max(-85.05, min(85.05, lat))
    x = int((lon + 180.0) / 360.0 * n)
    s = math.sin(math.radians(lat))
    y = int((0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n)
    return min(max(x, 0), n - 1), min(max(y, 0), n - 1)


def tile_range(lat, lon, radius_m, z):
    dlat = math.degrees(radius_m / 6371008.8)
    dlon = dlat / max(0.01, math.cos(math.radians(lat)))
    x0, y0 = tile_xy(lat + dlat, lon - dlon, z)
    x1, y1 = tile_xy(lat - dlat, lon + dlon, z)
    return [(z, x, y) for x in range(min(x0, x1), max(x0, x1) + 1) for y in range(min(y0, y1), max(y0, y1) + 1)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--radius-km", type=float, default=25.0)
    ap.add_argument("--layers", default="streets,satellite,elevation",
                    help="comma list of streets, satellite, elevation")
    ap.add_argument("--max-zoom", type=int, help="map imagery max zoom (default by radius: 16/15/14)")
    ap.add_argument("--elevation-zoom", type=int, help="elevation zoom (default 13, 12 above 30 km)")
    ap.add_argument("--out", required=True, help="output folder (the GCS offline folder layout)")
    ap.add_argument("--streets-url")
    ap.add_argument("--satellite-url")
    ap.add_argument("--elevation-url")
    ap.add_argument("--delay", type=float, default=0.05, help="pause between requests, seconds")
    ap.add_argument("--dry-run", action="store_true", help="only print the tile count and size estimate")
    a = ap.parse_args()

    layers = [l.strip() for l in a.layers.split(",") if l.strip()]
    for l in layers:
        if l not in LAYERS:
            sys.exit(f"unknown layer {l!r}; use streets, satellite, elevation")
    urls = {"streets": a.streets_url, "satellite": a.satellite_url, "elevation": a.elevation_url}
    mz = a.max_zoom or map_max_zoom(a.radius_km)
    ez = a.elevation_zoom or elevation_zoom(a.radius_km)
    r = a.radius_km * 1000.0

    jobs = []
    for l in layers:
        zooms = [ez] if l == "elevation" else range(MAP_MIN_ZOOM, mz + 1)
        for z in zooms:
            jobs += [(l,) + t for t in tile_range(a.lat, a.lon, r, z)]
    est = sum(TYPICAL_BYTES[j[0]] for j in jobs)
    print(f"{len(jobs)} tiles, about {est / 1e6:.0f} MB "
          f"(map zoom {MAP_MIN_ZOOM}-{mz}, elevation zoom {ez}) -> {a.out}")
    if a.dry_run:
        return

    done = skipped = failed = 0
    got = 0
    t0 = time.time()
    for l, z, x, y in jobs:
        sub, ext, default_url = LAYERS[l]
        path = os.path.join(a.out, sub, str(z), str(x), f"{y}.{ext}")
        done += 1
        if os.path.exists(path):
            skipped += 1
            continue
        url = (urls[l] or default_url).format(z=z, x=x, y=y)
        req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        try:
            with urllib.request.urlopen(req, timeout=20) as resp:
                data = resp.read()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path + ".part", "wb") as f:
                f.write(data)
            os.replace(path + ".part", path)
            got += len(data)
        except (urllib.error.URLError, OSError) as e:
            failed += 1
            print(f"\n  failed {l} {z}/{x}/{y}: {e}", file=sys.stderr)
        if done % 25 == 0 or done == len(jobs):
            rate = got / max(1e-3, time.time() - t0) / 1e6
            print(f"\r{done}/{len(jobs)}  {got / 1e6:.1f} MB  {rate:.2f} MB/s  skipped {skipped}  failed {failed}",
                  end="", flush=True)
        time.sleep(a.delay)
    print(f"\ndone: {got / 1e6:.1f} MB new, {skipped} already there, {failed} failed")
    if failed:
        print("run the same command again to retry the failed tiles")


if __name__ == "__main__":
    main()
