#!/usr/bin/env python3
"""
generate_rf_heatmap_kml.py - Generates a Google Earth KML RF signal (dBm) heatmap
and cadastral waypoint overlay from RTK Rover serial logs, Base pacer logs, and
RTK Rover waypoint CSV files.
"""

import argparse
import csv
import datetime
import html
import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET

__filename__ = os.path.basename(__file__)
__version__ = "v1.0.1"

_EARTH_R_M = 6371000.0
_DEFAULT_HEAT_RADIUS_M = 16.0
_MIN_TRACK_STEP_M = 1.0


def haversine_m(lat1, lon1, lat2, lon2):
    """Compute great-circle distance in meters between two WGS84 coordinates."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * _EARTH_R_M * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Compute initial forward azimuth in degrees (0..360) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def destination_point(lat, lon, bear_deg, dist_m):
    """Project a WGS84 point by bearing (deg) and distance (m)."""
    ang = dist_m / _EARTH_R_M
    b = math.radians(bear_deg)
    p1 = math.radians(lat)
    l1 = math.radians(lon)
    p2 = math.asin(math.sin(p1) * math.cos(ang) + math.cos(p1) * math.sin(ang) * math.cos(b))
    l2 = l1 + math.atan2(
        math.sin(b) * math.sin(ang) * math.cos(p1),
        math.cos(ang) - math.sin(p1) * math.sin(p2),
    )
    return math.degrees(p2), math.degrees(l2)


def ts_to_sec(ts_str):
    """Convert HH:MM:SS(.mmm) to seconds since midnight."""
    h, m, s = ts_str.split(":")
    return int(h) * 3600.0 + int(m) * 60.0 + float(s)


def _unwrap_time_seq(items, key="t_raw"):
    """Monotonically unwrap HH:MM:SS timestamps across UTC midnight rollovers."""
    offset = 0.0
    prev = None
    for item in items:
        cur = item[key]
        if prev is not None and (cur + offset) < (prev - 43200.0):
            offset += 86400.0
        item["t"] = cur + offset
        prev = item["t"]
    return items


def dbm_to_kml_color(dbm, alpha_hex="B0"):
    """Map RSSI dBm (-35..-88) smoothly from green -> yellow -> orange -> red in KML aabbggrr."""
    stops = [
        (-40.0, (0, 230, 118)),   # Bright green
        (-58.0, (198, 255, 0)),   # Lime
        (-68.0, (255, 220, 0)),   # Yellow
        (-77.0, (255, 130, 0)),   # Orange
        (-86.0, (255, 23, 68)),    # Vivid red
    ]
    if dbm >= stops[0][0]:
        r, g, b = stops[0][1]
    elif dbm <= stops[-1][0]:
        r, g, b = stops[-1][1]
    else:
        r, g, b = stops[-1][1]
        for idx in range(len(stops) - 1):
            hi_d, c_hi = stops[idx]
            lo_d, c_lo = stops[idx + 1]
            if lo_d <= dbm <= hi_d:
                frac = (hi_d - dbm) / (hi_d - lo_d)
                r = int(round(c_hi[0] + frac * (c_lo[0] - c_hi[0])))
                g = int(round(c_hi[1] + frac * (c_lo[1] - c_hi[1])))
                b = int(round(c_hi[2] + frac * (c_lo[2] - c_hi[2])))
                break
    return f"{alpha_hex.upper()}{b:02X}{g:02X}{r:02X}"


def fix_code_label(code):
    """Map UM982 / telemetry numeric fix code to human-readable status."""
    mapping = {
        0: "NO_FIX",
        1: "SINGLE",
        2: "DGPS",
        4: "RTK_FIX",
        5: "RTK_FLT",
    }
    return mapping.get(code, f"FIX_{code}")


def parse_serial_run(serial_path, start_override=None, end_override=None):
    """Parse [UM982 BASE], [TELEM], and [E220 LORA TX] entries from an RTK Rover serial log."""
    base_pos = None
    telem_raw = []
    tx_raw = []

    base_re = re.compile(r"\[UM982 BASE\]\s+ID:\d+\s+Valid:1\s+Pos:([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)m")
    telem_re = re.compile(
        r"\[(\d+:\d+:\d+\.\d+)\]\s+\[TELEM\]\s+Fix:(\d+)\s+SVs:(\d+)/(\d+)\s+Hdg:([^\s]+)\s+"
        r"BL:([\d.]+)cm\s+Spd:([\d.]+)m/s\s+Base:\d+\s+Diff:([\d.]+)s.*"
        r"Noise:(-?\d+)dBm\s+Pkt:(-?\d+)dBm.*OK:(\d+).*Err:(\d+)\s+\(Trunc:(\d+)\)"
    )
    tx_re = re.compile(
        r"\[(\d+:\d+:\d+\.\d+)\]\s+\[E220 LORA TX #(\d+)\].*"
        r"Fix:(\d+)\s+SLV:(\d+)\s+Tip:([-\d.]+),([-\d.]+),([-\d.]+)m"
    )

    with open(serial_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if base_pos is None:
                mb = base_re.search(line)
                if mb:
                    base_pos = (float(mb.group(1)), float(mb.group(2)), float(mb.group(3)))
            mt = telem_re.match(line)
            if mt:
                t_raw = ts_to_sec(mt.group(1))
                telem_raw.append({
                    "ts": mt.group(1), "t_raw": t_raw,
                    "fix": int(mt.group(2)), "sv1": int(mt.group(3)), "sv2": int(mt.group(4)),
                    "hdg": mt.group(5), "bl_cm": float(mt.group(6)), "spd": float(mt.group(7)),
                    "diff": float(mt.group(8)), "noise": int(mt.group(9)), "pkt": int(mt.group(10)),
                    "ok": int(mt.group(11)), "err": int(mt.group(12)), "trunc": int(mt.group(13)),
                })
                continue
            mx = tx_re.match(line)
            if mx:
                tx_raw.append({
                    "ts": mx.group(1), "t_raw": ts_to_sec(mx.group(1)),
                    "seq": int(mx.group(2)), "fix": int(mx.group(3)), "slv": int(mx.group(4)),
                    "lat": float(mx.group(5)), "lon": float(mx.group(6)), "alt": float(mx.group(7)),
                })

    _unwrap_time_seq(telem_raw)
    _unwrap_time_seq(tx_raw)

    # Filter out pre-GNSS boot lines (< 06:00 unless entire log starts there) or apply explicit overrides
    if start_override is not None:
        t_lo = ts_to_sec(start_override)
        telem_raw = [r for r in telem_raw if r["t"] >= t_lo]
        tx_raw = [x for x in tx_raw if x["t"] >= t_lo]
    if end_override is not None:
        t_hi = ts_to_sec(end_override)
        telem_raw = [r for r in telem_raw if r["t"] <= t_hi]
        tx_raw = [x for x in tx_raw if x["t"] <= t_hi]

    # Match each 5s TELEM snapshot (starting at index 1 so delta_ok is exact) to nearest TX lat/lon
    dl_points = []
    for idx in range(1, len(telem_raw)):
        cur = telem_raw[idx]
        prv = telem_raw[idx - 1]
        if not tx_raw:
            break
        best_tx = min(tx_raw, key=lambda x: abs(x["t"] - cur["t"]))
        if abs(best_tx["t"] - cur["t"]) > 3.5:
            continue
        dok = max(0, cur["ok"] - prv["ok"])
        derr = max(0, cur["err"] - prv["err"])
        dl_points.append({
            "ts": cur["ts"], "t": cur["t"],
            "lat": best_tx["lat"], "lon": best_tx["lon"], "alt": best_tx["alt"],
            "fix": cur["fix"], "hdg": cur["hdg"], "spd": cur["spd"], "diff": cur["diff"],
            "noise": cur["noise"], "pkt": cur["pkt"],
            "dok": dok, "derr": derr, "ok_total": cur["ok"],
            "live": (dok > 0 and cur["pkt"] < 0),
        })

    t_span = (telem_raw[0]["t"] - 5.0, telem_raw[-1]["t"] + 5.0) if telem_raw else (0.0, 0.0)
    return base_pos, dl_points, t_span


def parse_pacer_log(pacer_path):
    """Parse [ROVER TEL] uplink records and align them to exact UTC time from [PACER HEARTBEAT]."""
    if not pacer_path or not os.path.isfile(pacer_path):
        return []

    hb_re = re.compile(r"(\d+:\d+:\d+)\s+\S+\s+python3\[\d+\]:\s+\[(\d+:\d+:\d+)\]\s+\[PACER HEARTBEAT\]")
    tel_re = re.compile(
        r"(\d+:\d+:\d+)\s+\S+\s+python3\[\d+\]:\s+\[ROVER TEL #(\d+)\]\s+FW:([^\s]+)\s+\|\s+"
        r"ANT1:([^\s]+).*?\|\s+ANT2:([^\s]+).*?\|\s+Hdg:\s*([\d.]+)deg\s+\[([^\]]+)\].*?"
        r"RSSI:(-?\d+)dBm\s+\|\s+Pos:([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)m"
    )

    utc_offset_s = None
    uplinks = []
    with open(pacer_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            mh = hb_re.search(line)
            if mh:
                sys_s = ts_to_sec(mh.group(1))
                utc_s = ts_to_sec(mh.group(2))
                diff = utc_s - sys_s
                if diff < -43200.0:
                    diff += 86400.0
                elif diff > 43200.0:
                    diff -= 86400.0
                utc_offset_s = diff
                continue

            mt = tel_re.search(line)
            if mt and utc_offset_s is not None:
                rssi = int(mt.group(8))
                if rssi >= 0:
                    continue
                sys_s = ts_to_sec(mt.group(1))
                utc_s = (sys_s + utc_offset_s) % 86400.0
                uplinks.append({
                    "t_raw": utc_s,
                    "ts": str(datetime.timedelta(seconds=int(round(utc_s)))),
                    "seq": int(mt.group(2)), "fw": mt.group(3),
                    "ant1": mt.group(4), "ant2": mt.group(5),
                    "hdg": mt.group(6), "hdg_src": mt.group(7),
                    "rssi": rssi,
                    "lat": float(mt.group(9)), "lon": float(mt.group(10)), "alt": float(mt.group(11)),
                })

    _unwrap_time_seq(uplinks)
    return uplinks


def parse_waypoints_csv(csv_path):
    """Read an RTK Rover waypoint CSV file into structured records."""
    rows = []
    with open(csv_path, encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            try:
                lat = float(row.get("latitude") or row.get("lat") or 0.0)
                lon = float(row.get("longitude") or row.get("lon") or 0.0)
            except ValueError:
                continue
            if lat == 0.0 or lon == 0.0:
                continue
            try:
                seq = int(float(row.get("poly_order") or 0))
            except ValueError:
                seq = 0
            rows.append({
                "label": (row.get("label") or row.get("name") or "WP").strip(),
                "lat": lat, "lon": lon,
                "seq": seq,
                "notes": (row.get("notes") or "").strip(),
                "pos_fix": (row.get("pos_fix") or "").strip(),
            })
    return rows


def circle_polygon_coords(lat, lon, radius_m, steps=20):
    """Build closed KML lon,lat,0 coordinate string for a circle around (lat, lon)."""
    coords = []
    for step in range(steps + 1):
        ang = (360.0 * step) / steps
        plat, plon = destination_point(lat, lon, ang, radius_m)
        coords.append(f"{plon:.7f},{plat:.7f},0")
    return " ".join(coords)


def _add_placemark_point(parent, name, desc_html, lat, lon, style_id):
    pm = ET.SubElement(parent, "Placemark")
    ET.SubElement(pm, "name").text = name
    ET.SubElement(pm, "description").text = desc_html
    ET.SubElement(pm, "styleUrl").text = f"#{style_id}"
    pt = ET.SubElement(pm, "Point")
    ET.SubElement(pt, "coordinates").text = f"{lon:.7f},{lat:.7f},0"


def _add_heat_disc(parent, name, desc_html, lat, lon, radius_m, fill_color, outline_color, width=1):
    pm = ET.SubElement(parent, "Placemark")
    ET.SubElement(pm, "name").text = name
    ET.SubElement(pm, "description").text = desc_html
    st = ET.SubElement(pm, "Style")
    ls = ET.SubElement(st, "LineStyle")
    ET.SubElement(ls, "color").text = outline_color
    ET.SubElement(ls, "width").text = str(width)
    ps = ET.SubElement(st, "PolyStyle")
    ET.SubElement(ps, "color").text = fill_color
    ET.SubElement(ps, "fill").text = "1"
    ET.SubElement(ps, "outline").text = "1"
    poly = ET.SubElement(pm, "Polygon")
    ET.SubElement(poly, "tessellate").text = "1"
    ob = ET.SubElement(poly, "outerBoundaryIs")
    lr = ET.SubElement(ob, "LinearRing")
    ET.SubElement(lr, "coordinates").text = circle_polygon_coords(lat, lon, radius_m, steps=18)


def _add_track_segment(parent, p1, p2, color_kml, width=4, name=None):
    """Emit a KML LineString track segment only when endpoints are strictly distinct."""
    c1 = f"{p1['lon']:.7f},{p1['lat']:.7f},0"
    c2 = f"{p2['lon']:.7f},{p2['lat']:.7f},0"
    if c1 == c2 or haversine_m(p1["lat"], p1["lon"], p2["lat"], p2["lon"]) < _MIN_TRACK_STEP_M:
        return False
    pm = ET.SubElement(parent, "Placemark")
    ET.SubElement(pm, "name").text = name or f"Track {p1['ts']}"
    st = ET.SubElement(pm, "Style")
    ls = ET.SubElement(st, "LineStyle")
    ET.SubElement(ls, "color").text = color_kml
    ET.SubElement(ls, "width").text = str(width)
    ln = ET.SubElement(pm, "LineString")
    ET.SubElement(ln, "tessellate").text = "1"
    ET.SubElement(ln, "coordinates").text = f"{c1} {c2}"
    return True


def _advance_track_segment(tracks_folder, prev_pt, pt, line_color, label):
    """Advance track anchor and emit segment only for motion within the gap threshold."""
    if prev_pt is None:
        return pt
    step_m = haversine_m(prev_pt["lat"], prev_pt["lon"], pt["lat"], pt["lon"])
    if step_m >= 250.0:
        return pt
    if step_m >= _MIN_TRACK_STEP_M:
        if _add_track_segment(tracks_folder, prev_pt, pt, line_color, width=4, name=f"{label} Track {pt['ts']}"):
            return pt
    return prev_pt


def _build_base_folder(doc, base_lat, base_lon, ring_radii_m):
    base_folder = ET.SubElement(doc, "Folder")
    ET.SubElement(base_folder, "name").text = "Base Station & Range Rings"
    ET.SubElement(base_folder, "open").text = "0"

    base_desc = f"<b>RTK Base Antenna</b><br/>Lat: {base_lat:.7f}<br/>Lon: {base_lon:.7f}"
    _add_placemark_point(base_folder, "RTK Base Station", base_desc, base_lat, base_lon, "basePinStyle")

    for r_m in ring_radii_m:
        pm = ET.SubElement(base_folder, "Placemark")
        ET.SubElement(pm, "name").text = f"Range {r_m} m ({int(round(r_m * 3.28084))} ft)"
        ET.SubElement(pm, "styleUrl").text = "#ringStyle"
        ln = ET.SubElement(pm, "LineString")
        ET.SubElement(ln, "tessellate").text = "1"
        ET.SubElement(ln, "coordinates").text = circle_polygon_coords(base_lat, base_lon, r_m, steps=64)


def _build_downlink_folder(doc, label, points, base_lat, base_lon, heat_radius_m, visible=True):
    folder = ET.SubElement(doc, "Folder")
    ET.SubElement(folder, "name").text = f"{label} - Downlink (Base -> Rover dBm + Blackouts)"
    ET.SubElement(folder, "visibility").text = "1" if visible else "0"
    ET.SubElement(folder, "open").text = "0"

    tracks = ET.SubElement(folder, "Folder")
    ET.SubElement(tracks, "name").text = "Track Segments"
    ET.SubElement(tracks, "visibility").text = "1" if visible else "0"
    blobs = ET.SubElement(folder, "Folder")
    ET.SubElement(blobs, "name").text = "5s dBm Heatmap Discs (Click for Telemetry)"
    ET.SubElement(blobs, "visibility").text = "1" if visible else "0"

    prev_track_pt = None
    for pt in points:
        dist = haversine_m(base_lat, base_lon, pt["lat"], pt["lon"])
        bear = bearing_deg(base_lat, base_lon, pt["lat"], pt["lon"])
        snr = pt["pkt"] - pt["noise"] if (pt["pkt"] < 0 and pt["noise"] < 0) else 0
        if pt["live"]:
            fill_c = dbm_to_kml_color(pt["pkt"], "88")
            line_c = dbm_to_kml_color(pt["pkt"], "EE")
            title = f"{label} DL: {pt['pkt']} dBm @ {dist:.0f}m"
            state_txt = "LIVE RTCM RX"
        else:
            fill_c = "60303030"
            line_c = "CC555555"
            title = f"{label} DL: NO RTCM @ {dist:.0f}m"
            state_txt = "DEADZONE (0 new RTCM frames; Pkt RSSI stale)"

        desc = (
            f"<b>{html.escape(title)}</b><br/>"
            f"<b>Status:</b> {state_txt}<br/>"
            f"<b>UTC:</b> {pt['ts']} | <b>Dist:</b> {dist:.1f} m ({dist * 3.28084:.0f} ft) @ {bear:.1f} deg<br/>"
            f"<b>Pkt RSSI:</b> {pt['pkt']} dBm | <b>Noise:</b> {pt['noise']} dBm | <b>SNR:</b> {snr:+d} dB<br/>"
            f"<b>+OK (5s):</b> {pt['dok']} frames (Err+: {pt['derr']}, Total OK: {pt['ok_total']})<br/>"
            f"<b>GNSS:</b> {fix_code_label(pt['fix'])} | <b>Diff Age:</b> {pt['diff']:.1f} s | <b>Spd:</b> {pt['spd']:.1f} m/s"
        )
        _add_heat_disc(blobs, title, desc, pt["lat"], pt["lon"], heat_radius_m, fill_c, line_c, width=1)
        prev_track_pt = _advance_track_segment(tracks, prev_track_pt, pt, line_c, f"{label} DL")


def _build_uplink_folder(doc, label, uplinks, base_lat, base_lon, heat_radius_m, visible=False):
    folder = ET.SubElement(doc, "Folder")
    ET.SubElement(folder, "name").text = f"{label} - Uplink (Rover -> Base dBm)"
    ET.SubElement(folder, "visibility").text = "1" if visible else "0"
    ET.SubElement(folder, "open").text = "0"

    tracks = ET.SubElement(folder, "Folder")
    ET.SubElement(tracks, "name").text = "Uplink Track Segments"
    ET.SubElement(tracks, "visibility").text = "1" if visible else "0"
    blobs = ET.SubElement(folder, "Folder")
    ET.SubElement(blobs, "name").text = "5s Base RX dBm Heatmap Discs"
    ET.SubElement(blobs, "visibility").text = "1" if visible else "0"

    prev_track_pt = None
    for pt in uplinks:
        dist = haversine_m(base_lat, base_lon, pt["lat"], pt["lon"])
        bear = bearing_deg(base_lat, base_lon, pt["lat"], pt["lon"])
        fill_c = dbm_to_kml_color(pt["rssi"], "88")
        line_c = dbm_to_kml_color(pt["rssi"], "EE")
        title = f"{label} UL #{pt['seq']}: {pt['rssi']} dBm @ {dist:.0f}m"
        desc = (
            f"<b>{html.escape(title)}</b><br/>"
            f"<b>Base RX RSSI:</b> {pt['rssi']} dBm<br/>"
            f"<b>UTC:</b> {pt['ts']} | <b>Dist:</b> {dist:.1f} m ({dist * 3.28084:.0f} ft) @ {bear:.1f} deg<br/>"
            f"<b>ANT1:</b> {html.escape(pt['ant1'])} | <b>ANT2:</b> {html.escape(pt['ant2'])} | "
            f"<b>Hdg:</b> {pt['hdg']} deg [{html.escape(pt['hdg_src'])}]"
        )
        _add_heat_disc(blobs, title, desc, pt["lat"], pt["lon"], heat_radius_m, fill_c, line_c, width=1)
        prev_track_pt = _advance_track_segment(tracks, prev_track_pt, pt, line_c, f"{label} UL")


def _build_waypoints_folder(doc, csv_path, base_lat, base_lon):
    rows = parse_waypoints_csv(csv_path)
    if not rows:
        return
    name = os.path.basename(csv_path)
    folder = ET.SubElement(doc, "Folder")
    ET.SubElement(folder, "name").text = f"Parcel: {name}"
    ET.SubElement(folder, "open").text = "0"

    by_seq = {}
    pos_pts = []
    for r in rows:
        dist = haversine_m(base_lat, base_lon, r["lat"], r["lon"])
        bear = bearing_deg(base_lat, base_lon, r["lat"], r["lon"])
        desc = (
            f"<b>{html.escape(r['label'])}</b> (seq={r['seq']})<br/>"
            f"<b>Dist from Base:</b> {dist:.1f} m ({dist * 3.28084:.0f} ft) @ {bear:.1f} deg<br/>"
            f"<b>Recorded Fix:</b> {html.escape(r['pos_fix'])}<br/>"
            f"<b>Notes:</b> {html.escape(r['notes'])}"
        )
        _add_placemark_point(folder, f"{r['label']} ({dist:.0f}m)", desc, r["lat"], r["lon"], "wpPinStyle")
        if r["seq"] > 0:
            pos_pts.append(r)
            by_seq[r["seq"]] = r

    if len(pos_pts) >= 3:
        pos_pts.sort(key=lambda x: x["seq"])
        coords = []
        for r in pos_pts:
            c = f"{r['lon']:.7f},{r['lat']:.7f},0"
            if not coords or coords[-1] != c:
                coords.append(c)
        if coords and coords[-1] != coords[0]:
            coords.append(coords[0])
        if len(set(coords)) >= 2 and len(coords) >= 2:
            pm = ET.SubElement(folder, "Placemark")
            ET.SubElement(pm, "name").text = f"{name} Boundary"
            ET.SubElement(pm, "styleUrl").text = "#parcelLineStyle"
            ln = ET.SubElement(pm, "LineString")
            ET.SubElement(ln, "tessellate").text = "1"
            ET.SubElement(ln, "coordinates").text = " ".join(coords)

    for r in rows:
        if r["seq"] < 0 and (-r["seq"]) in by_seq:
            tgt = by_seq[-r["seq"]]
            c1 = f"{r['lon']:.7f},{r['lat']:.7f},0"
            c2 = f"{tgt['lon']:.7f},{tgt['lat']:.7f},0"
            if c1 != c2:
                pm = ET.SubElement(folder, "Placemark")
                ET.SubElement(pm, "name").text = f"Tie: {r['label']} -> seq {-r['seq']}"
                ET.SubElement(pm, "styleUrl").text = "#tieLineStyle"
                ln = ET.SubElement(pm, "LineString")
                ET.SubElement(ln, "tessellate").text = "1"
                ET.SubElement(ln, "coordinates").text = f"{c1} {c2}"


def _add_shared_styles(doc):
    def icon_style(sid, color, scale, icon_href):
        st = ET.SubElement(doc, "Style", id=sid)
        ics = ET.SubElement(st, "IconStyle")
        ET.SubElement(ics, "color").text = color
        ET.SubElement(ics, "scale").text = str(scale)
        ic = ET.SubElement(ics, "Icon")
        ET.SubElement(ic, "href").text = icon_href
        lbs = ET.SubElement(st, "LabelStyle")
        ET.SubElement(lbs, "scale").text = "0.75"

    def line_style(sid, color, width):
        st = ET.SubElement(doc, "Style", id=sid)
        ls = ET.SubElement(st, "LineStyle")
        ET.SubElement(ls, "color").text = color
        ET.SubElement(ls, "width").text = str(width)

    icon_style("basePinStyle", "FF00FFFF", "1.2", "http://maps.google.com/mapfiles/kml/shapes/ranger_station.png")
    icon_style("wpPinStyle", "FFFFAA00", "0.85", "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png")
    line_style("ringStyle", "80FFFFFF", "1.5")
    line_style("parcelLineStyle", "FFE2A822", "2.5")
    line_style("tieLineStyle", "FF0088FF", "2.0")


def generate_kml(runs, pacer_log, waypoint_csvs, out_path, base_override=None, heat_radius_m=_DEFAULT_HEAT_RADIUS_M):
    """Build and write the complete KML RF signal heatmap document."""
    parsed_runs = []
    detected_base = base_override
    for spec in runs:
        parts = spec.split("::")
        if len(parts) < 2:
            raise ValueError(f"Invalid --run spec '{spec}'; expected LABEL::SERIAL_LOG[::START_UTC::END_UTC]")
        label, path = parts[0], parts[1]
        t_start = parts[2] if len(parts) >= 3 and parts[2] else None
        t_end = parts[3] if len(parts) >= 4 and parts[3] else None
        base_pos, dl_pts, t_span = parse_serial_run(path, t_start, t_end)
        if detected_base is None and base_pos is not None:
            detected_base = (base_pos[0], base_pos[1])
        parsed_runs.append((label, dl_pts, t_span))

    if detected_base is None:
        raise ValueError("Could not determine Base coordinates; provide --base-lat and --base-lon")

    base_lat, base_lon = detected_base
    uplinks = parse_pacer_log(pacer_log) if pacer_log else []

    kml = ET.Element("kml", xmlns="http://www.opengis.net/kml/2.2")
    doc = ET.SubElement(kml, "Document")
    ET.SubElement(doc, "name").text = "RTK Rover LoRa RF Heatmap (dBm)"
    ET.SubElement(doc, "description").text = (
        "Color scale: Green (-40 dBm) -> Yellow (-68 dBm) -> Red (-86 dBm); "
        "Dark Gray = 0 new RTCM frames received in 5s window (downlink blackout)."
    )
    _add_shared_styles(doc)
    _build_base_folder(doc, base_lat, base_lon, [150, 300, 450, 750])

    counts = {}
    for idx, (label, dl_pts, (t_lo, t_hi)) in enumerate(parsed_runs):
        _build_downlink_folder(doc, label, dl_pts, base_lat, base_lon, heat_radius_m, visible=(idx == 0))
        run_ul = [u for u in uplinks if t_lo <= u["t"] <= t_hi]
        if run_ul:
            _build_uplink_folder(doc, label, run_ul, base_lat, base_lon, heat_radius_m, visible=False)
        live_cnt = sum(1 for p in dl_pts if p["live"])
        dead_cnt = len(dl_pts) - live_cnt
        counts[label] = {"dl_total": len(dl_pts), "dl_live": live_cnt, "dl_dead": dead_cnt, "ul_total": len(run_ul)}

    for csv_p in waypoint_csvs:
        _build_waypoints_folder(doc, csv_p, base_lat, base_lon)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    tree = ET.ElementTree(kml)
    ET.indent(tree, space="  ")
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return counts


def run_self_test():
    """Verify KML generation cleanly using synthetic temporary fixtures."""
    with tempfile.TemporaryDirectory() as tmp:
        ser = os.path.join(tmp, "serial_synth.txt")
        pac = os.path.join(tmp, "pacer_synth.log")
        csv_p = os.path.join(tmp, "wp_synth.csv")
        out_p = os.path.join(tmp, "out_synth.kml")

        with open(ser, "w", encoding="utf-8") as f:
            f.write("[00:00:05.000] [UM982 BASE] ID:1 Valid:1 Pos:44.00000000, -70.00000000, 50.000m (ECEF: 0, 0, 0)\n")
            f.write("[12:00:00.100] [E220 LORA TX #1] Uplink 48B (FW:v0.8.17 Fix:4 SLV:4 Tip:44.0005000,-70.0000000,50.0m BL:55.0cm P:0.0deg USB:5.00V+)\n")
            f.write("[12:00:00.200] [TELEM] Fix:4 SVs:20/10 Hdg:FIX(0.0) BL:55.0cm Spd:0.0m/s Base:1 Diff:1.0s | USB:5.00V+ | COM1 RX:100B NMEA:10 RTCM_TX:500B | E220 AUX:1 (Edges:5) Noise:-96dBm Pkt:-45dBm RX:500B Fwd:500B OK:10 (1006:1) Err:0 (Trunc:0) Orph:0B\n")
            f.write("[12:00:05.100] [E220 LORA TX #2] Uplink 48B (FW:v0.8.17 Fix:4 SLV:4 Tip:44.0020000,-70.0000000,50.0m BL:55.0cm P:0.0deg USB:5.00V+)\n")
            f.write("[12:00:05.200] [TELEM] Fix:4 SVs:20/10 Hdg:FIX(0.0) BL:55.0cm Spd:1.0m/s Base:1 Diff:1.5s | USB:5.00V+ | COM1 RX:200B NMEA:20 RTCM_TX:1000B | E220 AUX:1 (Edges:10) Noise:-88dBm Pkt:-72dBm RX:1000B Fwd:1000B OK:18 (1006:2) Err:0 (Trunc:0) Orph:0B\n")
            f.write("[12:00:10.100] [E220 LORA TX #3] Uplink 48B (FW:v0.8.17 Fix:4 SLV:4 Tip:44.0020000,-70.0000000,50.0m BL:55.0cm P:0.0deg USB:5.00V+)\n")
            f.write("[12:00:10.200] [TELEM] Fix:4 SVs:20/10 Hdg:FIX(0.0) BL:55.0cm Spd:0.0m/s Base:1 Diff:6.5s | USB:5.00V+ | COM1 RX:300B NMEA:30 RTCM_TX:1000B | E220 AUX:1 (Edges:10) Noise:-84dBm Pkt:-72dBm RX:1000B Fwd:1000B OK:18 (1006:2) Err:0 (Trunc:0) Orph:0B\n")
            f.write("[12:00:15.100] [E220 LORA TX #4] Uplink 48B (FW:v0.8.17 Fix:4 SLV:4 Tip:44.0021000,-70.0000000,50.0m BL:55.0cm P:0.0deg USB:5.00V+)\n")
            f.write("[12:00:15.200] [TELEM] Fix:4 SVs:20/10 Hdg:FIX(0.0) BL:55.0cm Spd:2.2m/s Base:1 Diff:11.5s | USB:5.00V+ | COM1 RX:400B NMEA:40 RTCM_TX:1000B | E220 AUX:1 (Edges:10) Noise:-85dBm Pkt:-72dBm RX:1000B Fwd:1000B OK:18 (1006:2) Err:0 (Trunc:0) Orph:0B\n")

        with open(pac, "w", encoding="utf-8") as f:
            f.write("Nov 17 07:00:05 host python3[1]: [12:00:05] [PACER HEARTBEAT] In:100B (20.0B/s) | Out:50B (10.0B/s)\n")
            f.write("Nov 17 07:00:05 host python3[1]: [ROVER TEL #002] FW:v0.8.17 | ANT1:RTK_FIX (20M) | ANT2:SLV_FIX (10S, BL:55.0cm, P:0.0deg) | Hdg: 0.0deg [DUAL_FIX] | PWR:USB (5.0V) | RSSI:-68dBm | Pos:44.0020000, -70.0000000, 50.00m\n")
            f.write("Nov 17 07:00:10 host python3[1]: [ROVER TEL #003] FW:v0.8.17 | ANT1:RTK_FIX (20M) | ANT2:SLV_FIX (10S, BL:55.0cm, P:0.0deg) | Hdg: 0.0deg [DUAL_FIX] | PWR:USB (5.0V) | RSSI:-69dBm | Pos:44.0020000, -70.0000000, 50.00m\n")

        with open(csv_p, "w", encoding="utf-8") as f:
            f.write("label,latitude,longitude,poly_order,pos_fix,notes\n")
            f.write("SYNTH_A,44.001,-70.001,1,RTK_FIX,Corner 1\n")
            f.write("SYNTH_B,44.001,-69.999,2,RTK_FIX,Corner 2\n")
            f.write("SYNTH_C,44.002,-70.000,3,RTK_FIX,Corner 3\n")
            f.write("SYNTH_TIE,44.0015,-70.000,-2,RTK_FIX,Tie\n")

        counts = generate_kml(["SYNTH::" + ser], pac, [csv_p], out_p)
        assert counts["SYNTH"] == {"dl_total": 3, "dl_live": 1, "dl_dead": 2, "ul_total": 2}, counts
        tree = ET.parse(out_p)
        ns = {"k": "http://www.opengis.net/kml/2.2"}
        pms = tree.findall(".//k:Placemark", ns)
        assert len(pms) >= 14, f"Expected >= 14 Placemarks, got {len(pms)}"
        for pm in pms:
            assert (pm.findtext("k:name", "", ns) or "").strip(), "Placemark missing non-empty name"
        for ln in tree.findall(".//k:LineString/k:coordinates", ns):
            pts = (ln.text or "").strip().split()
            assert len(set(pts)) >= 2, f"Degenerate LineString: {pts}"
    print("[SELF-TEST] PASS: Synthetic KML RF heatmap & parcel overlay verified.")


def main():
    parser = argparse.ArgumentParser(description="Generate a Google Earth KML RF signal heatmap from RTK Rover logs.")
    parser.add_argument("--run", action="append", default=[], help="Run spec: LABEL::SERIAL_LOG[::START_UTC::END_UTC]")
    parser.add_argument("--pacer-log", default=None, help="Optional Base e220_pacer log for Rover->Base uplink RSSI")
    parser.add_argument("--waypoints", action="append", default=[], help="Optional waypoint CSV path (repeatable)")
    parser.add_argument("--base-lat", type=float, default=None, help="Optional Base latitude override")
    parser.add_argument("--base-lon", type=float, default=None, help="Optional Base longitude override")
    parser.add_argument("--radius-m", type=float, default=_DEFAULT_HEAT_RADIUS_M, help="Heatmap disc radius in meters")
    parser.add_argument("--out", default=None, help="Output KML file path")
    parser.add_argument("--self-test", action="store_true", help="Run synthetic unit verification and exit")
    args = parser.parse_args()

    print(f"=== {__filename__} {__version__} ===")

    if args.self_test:
        run_self_test()
        return

    if not args.run or not args.out:
        parser.error("--run and --out are required unless running --self-test")

    base_ov = (args.base_lat, args.base_lon) if (args.base_lat is not None and args.base_lon is not None) else None
    counts = generate_kml(args.runs if hasattr(args, "runs") else args.run, args.pacer_log, args.waypoints, args.out, base_ov, args.radius_m)
    print(f"[OK] Wrote KML: {args.out}")
    for lbl, st in counts.items():
        print(f"  {lbl}: DL={st['dl_total']} (live={st['dl_live']}, deadzone={st['dl_dead']}) | UL={st['ul_total']}")


if __name__ == "__main__":
    main()
