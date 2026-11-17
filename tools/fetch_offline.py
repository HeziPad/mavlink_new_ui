#!/usr/bin/env python3
"""Download the GCS's offline map data, without the GCS.

Works on any computer with Python 3 (standard library only). It writes
exactly the folder the GCS reads (Settings -> OFFLINE -> FOLDER, default
~/.local/share/mavlink_gcs_ui/offline/):

    map.pmtiles                          street map: OpenStreetMap vector tiles (Protomaps build, ODbL)
    esri_imagery/{z}/{x}/{y}.jpg         satellite (Esri World Imagery); {y}.none = no imagery at that zoom
    elevation/{z}/{x}/{y}.png            ground height (AWS Terrain Tiles, "terrarium")
    areas.json                           the full-detail areas downloaded so far

Country pack (street map z0-15 + satellite z8-14 + elevation z12):
    tools/fetch_offline.py --country israel --out ./offline [--dry-run]

Full-detail area (satellite z15-18 + elevation z13), by bounding box:
    tools/fetch_offline.py --bbox 34.75,32.05,34.82,32.12 --out ./offline

Files already present are skipped, so runs resume and overlapping areas share
tiles (nothing is stored twice). Copy the folder to the field computer.

The street map can also be made with the official tool:
    pmtiles extract https://build.protomaps.com/<YYYYMMDD>.pmtiles map.pmtiles --bbox=33.7,29.0,36.5,33.8
"""

import argparse
import gzip
import hashlib
import json
import math
import os
import struct
import sys
import time
import urllib.error
import urllib.request

UA = "mavlink_gcs_ui-fetch_offline/2.0"
COUNTRIES = {
    # Israel + ~50 km around it: lon/lat box
    "israel": (33.7, 29.0, 36.5, 33.8),
}
SAT_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
ELEV_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
BUILDS = "https://build-metadata.protomaps.dev/builds.json"
PROTOMAPS = "https://build.protomaps.com/{key}"
# Esri answers "Map data not yet available" with this exact 256 px JPEG
ESRI_NO_DATA_MD5 = "f27d9de7f80c13501f470595e327aa6d"
COUNTRY_SAT_ZOOMS = range(8, 15)
COUNTRY_ELEV_ZOOMS = [12]
AREA_SAT_ZOOMS = range(15, 19)
AREA_ELEV_ZOOMS = [13]
VECTOR_MAX_ZOOM = 15


def get(url, rng=None, timeout=60):
    headers = {"User-Agent": UA}
    if rng:
        headers["Range"] = "bytes=%d-%d" % rng
    for attempt in range(4):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=timeout) as r:
                return r.read()
        except (urllib.error.URLError, OSError):
            if attempt == 3:
                raise
            time.sleep(2 * (attempt + 1))


# ---- tiles ---------------------------------------------------------------------------------
def tile_xy(lat, lon, z):
    n = 2 ** z
    lat = max(-85.05, min(85.05, lat))
    s = math.sin(math.radians(lat))
    x = int((lon + 180.0) / 360.0 * n)
    y = int((0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n)
    return min(max(x, 0), n - 1), min(max(y, 0), n - 1)


def tiles_in(bbox, z):
    min_lon, min_lat, max_lon, max_lat = bbox
    x0, y0 = tile_xy(max_lat, min_lon, z)
    x1, y1 = tile_xy(min_lat, max_lon, z)
    return [(z, x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]


# ---- PMTiles v3 ----------------------------------------------------------------------------
def rotate(n, x, y, rx, ry):
    if ry == 0:
        if rx == 1:
            x, y = n - 1 - x, n - 1 - y
        x, y = y, x
    return x, y


def zxy_to_id(z, x, y):
    acc = sum(4 ** t for t in range(z))
    n = 1 << z
    d = 0
    s = n // 2
    while s > 0:
        rx = 1 if x & s else 0
        ry = 1 if y & s else 0
        d += s * s * ((3 * rx) ^ ry)
        x, y = rotate(n, x, y, rx, ry)
        s //= 2
    return acc + d


def read_varint(b, p):
    v = shift = 0
    while True:
        c = b[p]
        p += 1
        v |= (c & 0x7F) << shift
        shift += 7
        if not c & 0x80:
            return v, p


def write_varint(out, v):
    while v >= 0x80:
        out.append((v & 0x7F) | 0x80)
        v >>= 7
    out.append(v)


def decode_dir(b):
    n, p = read_varint(b, 0)
    ids, runs, lens, offs = [], [], [], []
    last = 0
    for _ in range(n):
        v, p = read_varint(b, p)
        last += v
        ids.append(last)
    for _ in range(n):
        v, p = read_varint(b, p)
        runs.append(v)
    for _ in range(n):
        v, p = read_varint(b, p)
        lens.append(v)
    for i in range(n):
        v, p = read_varint(b, p)
        offs.append(offs[-1] + lens[-1 if i == 0 else i - 1] if (v == 0 and i > 0) else v - 1)
    return [(ids[i], offs[i], lens[i], runs[i]) for i in range(n)]


def encode_dir(entries):
    out = bytearray()
    write_varint(out, len(entries))
    last = 0
    for e in entries:
        write_varint(out, e[0] - last)
        last = e[0]
    for e in entries:
        write_varint(out, e[3])
    for e in entries:
        write_varint(out, e[2])
    for i, e in enumerate(entries):
        if i > 0 and e[1] == entries[i - 1][1] + entries[i - 1][2]:
            write_varint(out, 0)
        else:
            write_varint(out, e[1] + 1)
    return bytes(out)


def find_entry(d, tid):
    lo, hi = 0, len(d)
    while lo < hi:
        mid = (lo + hi) // 2
        if d[mid][0] <= tid:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return None
    e = d[lo - 1]
    if e[3] == 0 or tid - e[0] < e[3]:
        return e
    return None


class Remote:
    """Reads directories and tiles from a remote PMTiles file with range requests."""

    def __init__(self, url):
        self.url = url
        h = get(url, (0, 126))
        assert h[:7] == b"PMTiles" and h[7] == 3, "not a PMTiles v3 file"
        f = struct.unpack("<11Q", h[8:96])
        (self.root_off, self.root_len, self.meta_off, self.meta_len, self.leaf_off, self.leaf_len,
         self.data_off) = f[:7]
        self.header = h
        self.root = decode_dir(gzip.decompress(get(url, (self.root_off, self.root_off + self.root_len - 1))))
        self.leaves = {}

    def leaf(self, off, ln):
        if off not in self.leaves:
            raw = get(self.url, (self.leaf_off + off, self.leaf_off + off + ln - 1))
            self.leaves[off] = decode_dir(gzip.decompress(raw))
        return self.leaves[off]

    def lookup(self, tid):
        e = find_entry(self.root, tid)
        depth = 0
        while e is not None and e[3] == 0 and depth < 4:
            e = find_entry(self.leaf(e[1], e[2]), tid)
            depth += 1
        return e if e is not None and e[3] > 0 else None

    def metadata(self):
        return get(self.url, (self.meta_off, self.meta_off + self.meta_len - 1))


def plan_vector(remote, bbox, max_zoom, progress):
    """tile id -> (src offset, length) for every tile of the bbox, z0..max_zoom"""
    want = []
    for z in range(0, max_zoom + 1):
        want += [zxy_to_id(*t) for t in tiles_in(bbox, z)]
    want.sort()
    found = {}
    for i, tid in enumerate(want):
        e = remote.lookup(tid)
        if e is not None:
            found[tid] = (e[1], e[2])
        if progress and i % 2000 == 0:
            progress("reading the index", i, len(want))
    return found


def write_pmtiles(path, remote, found, bbox, max_zoom, progress):
    """Downloads the planned tiles in a few large range requests and writes a clustered PMTiles v3."""
    # unique source blobs, fetched in offset order with neighbouring ranges merged
    blobs = sorted(set(found.values()))
    ranges = []
    for off, ln in blobs:
        if ranges and off - ranges[-1][1] <= 256 * 1024 and off + ln - ranges[-1][0] <= 16 * 1024 * 1024:
            ranges[-1][1] = max(ranges[-1][1], off + ln)
        else:
            ranges.append([off, off + ln])
    total = sum(b - a for a, b in ranges)
    data = {}
    got = 0
    for a, b in ranges:
        chunk = get(remote.url, (remote.data_off + a, remote.data_off + b - 1), timeout=120)
        for off, ln in blobs:
            if a <= off and off + ln <= b:
                data[(off, ln)] = chunk[off - a:off - a + ln]
        got += b - a
        if progress:
            progress("street map", got, total)
    # tiles in id order; identical tiles stored once; consecutive identical ids become runs
    entries = []
    out_off = {}
    body = bytearray()
    for tid in sorted(found):
        key = found[tid]
        if key not in out_off:
            out_off[key] = len(body)
            body += data[key]
        off, ln = out_off[key], key[1]
        if entries and entries[-1][0] + entries[-1][3] == tid and entries[-1][1] == off:
            entries[-1] = (entries[-1][0], off, ln, entries[-1][3] + 1)
        else:
            entries.append((tid, off, ln, 1))
    # root directory, split into leaves when it doesn't fit the first 16 KiB
    root = gzip.compress(encode_dir(entries))
    leaves = b""
    leaf_size = 4096
    while 127 + len(root) > 16384:
        leaf_dirs, root_entries, leaves = [], [], bytearray()
        for i in range(0, len(entries), leaf_size):
            chunk = gzip.compress(encode_dir(entries[i:i + leaf_size]))
            root_entries.append((entries[i][0], len(leaves), len(chunk), 0))
            leaves += chunk
        root = gzip.compress(encode_dir(root_entries))
        leaves = bytes(leaves)
        leaf_size *= 2
    meta = remote.metadata()
    root_off = 127
    meta_off = root_off + len(root)
    leaf_off = meta_off + len(meta)
    data_off = leaf_off + len(leaves)
    min_lon, min_lat, max_lon, max_lat = bbox
    header = bytearray(b"PMTiles") + bytes([3])
    header += struct.pack("<11Q", root_off, len(root), meta_off, len(meta), leaf_off, len(leaves), data_off,
                          len(body), len(found), len(entries), len(out_off))
    header += bytes([1, 2, 2, 1, 0, max_zoom])  # clustered, gzip dirs, gzip tiles, mvt, zooms
    header += struct.pack("<4i", int(min_lon * 1e7), int(min_lat * 1e7), int(max_lon * 1e7), int(max_lat * 1e7))
    header += bytes([10]) + struct.pack("<2i", int((min_lon + max_lon) / 2 * 1e7), int((min_lat + max_lat) / 2 * 1e7))
    assert len(header) == 127
    tmp = path + ".part"
    with open(tmp, "wb") as f:
        f.write(header)
        f.write(root)
        f.write(meta)
        f.write(leaves)
        f.write(body)
    os.replace(tmp, path)
    return len(body)


def latest_build():
    builds = json.loads(get(BUILDS))
    return PROTOMAPS.format(key=builds[-1]["key"])


# ---- raster tiles ------------------------------------------------------------------------------
def raster_jobs(sub, ext, zooms, bbox, out):
    jobs = []
    for z in zooms:
        for (_, x, y) in tiles_in(bbox, z):
            base = os.path.join(out, sub, str(z), str(x), str(y))
            if not any(os.path.exists(base + "." + e) for e in ("png", "jpg", "jpeg", "webp", "img", "none")):
                jobs.append((z, x, y, base + "." + ext))
    return jobs


def fetch_rasters(jobs, url, no_data_md5, delay, label):
    got = failed = 0
    t0 = time.time()
    for i, (z, x, y, path) in enumerate(jobs):
        try:
            data = get(url.format(z=z, x=x, y=y), timeout=30)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            if no_data_md5 and hashlib.md5(data).hexdigest() == no_data_md5:
                path = os.path.splitext(path)[0] + ".none"  # remember: nothing at this zoom
                data = b""
            with open(path + ".part", "wb") as f:
                f.write(data)
            os.replace(path + ".part", path)
            got += len(data)
        except (urllib.error.URLError, OSError) as e:
            failed += 1
            print("\n  failed %s %d/%d/%d: %s" % (label, z, x, y, e), file=sys.stderr)
        if i % 25 == 0 or i == len(jobs) - 1:
            print("\r%s %d/%d  %.1f MB  %.2f MB/s  failed %d" % (label, i + 1, len(jobs), got / 1e6,
                  got / 1e6 / max(1e-3, time.time() - t0), failed), end="", flush=True)
        time.sleep(delay)
    print()
    return failed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    what = ap.add_mutually_exclusive_group(required=True)
    what.add_argument("--country", choices=sorted(COUNTRIES), help="country pack")
    what.add_argument("--bbox", help="full-detail area: min_lon,min_lat,max_lon,max_lat")
    ap.add_argument("--out", required=True, help="the offline folder")
    ap.add_argument("--only", choices=["vector", "satellite", "elevation"], help="one part of the country pack")
    ap.add_argument("--pmtiles", help="Protomaps build URL (default: the latest daily build)")
    ap.add_argument("--delay", type=float, default=0.02, help="pause between raster requests, seconds")
    ap.add_argument("--dry-run", action="store_true", help="only report what would be downloaded")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    if a.country:
        bbox = COUNTRIES[a.country]
        parts = [a.only] if a.only else ["vector", "satellite", "elevation"]
        if "vector" in parts:
            url = a.pmtiles or latest_build()
            print("street map from", url)
            remote = Remote(url)
            prog = lambda what, i, n: print("\r%s %d/%d" % (what, i, n), end="", flush=True)
            found = plan_vector(remote, bbox, VECTOR_MAX_ZOOM, prog)
            size = sum(ln for (_, ln) in set(found.values()))
            print("\rstreet map: %d tiles, %.0f MB" % (len(found), size / 1e6))
            if not a.dry_run:
                write_pmtiles(os.path.join(a.out, "map.pmtiles"), remote, found, bbox, VECTOR_MAX_ZOOM,
                              lambda what, i, n: print("\r%s %.0f/%.0f MB" % (what, i / 1e6, n / 1e6), end="",
                                                       flush=True))
                print("\nwrote", os.path.join(a.out, "map.pmtiles"))
        if "satellite" in parts:
            jobs = raster_jobs("esri_imagery", "jpg", COUNTRY_SAT_ZOOMS, bbox, a.out)
            print("satellite z8-14: %d tiles to fetch (~%.0f MB)" % (len(jobs), len(jobs) * 0.02))
            if not a.dry_run:
                fetch_rasters(jobs, SAT_URL, ESRI_NO_DATA_MD5, a.delay, "satellite")
        if "elevation" in parts:
            jobs = raster_jobs("elevation", "png", COUNTRY_ELEV_ZOOMS, bbox, a.out)
            print("elevation z12: %d tiles to fetch (~%.0f MB)" % (len(jobs), len(jobs) * 0.06))
            if not a.dry_run:
                fetch_rasters(jobs, ELEV_URL, None, a.delay, "elevation")
        return

    bbox = tuple(float(v) for v in a.bbox.split(","))
    sat = raster_jobs("esri_imagery", "jpg", AREA_SAT_ZOOMS, bbox, a.out)
    elev = raster_jobs("elevation", "png", AREA_ELEV_ZOOMS, bbox, a.out)
    print("area: satellite z15-18 %d new tiles, elevation z13 %d new tiles (~%.0f MB)" %
          (len(sat), len(elev), len(sat) * 0.02 + len(elev) * 0.06))
    if a.dry_run:
        return
    failed = fetch_rasters(sat, SAT_URL, ESRI_NO_DATA_MD5, a.delay, "satellite")
    failed += fetch_rasters(elev, ELEV_URL, None, a.delay, "elevation")
    areas_path = os.path.join(a.out, "areas.json")
    areas = json.load(open(areas_path)) if os.path.exists(areas_path) else []
    areas.append({"min_lon": bbox[0], "min_lat": bbox[1], "max_lon": bbox[2], "max_lat": bbox[3],
                  "date": time.strftime("%Y-%m-%d")})
    with open(areas_path, "w") as f:
        json.dump(areas, f, indent=1)
    if failed:
        print("%d tiles failed: run the same command again to retry them" % failed)


if __name__ == "__main__":
    main()
