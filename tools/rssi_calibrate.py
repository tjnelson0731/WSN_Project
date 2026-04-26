#!/usr/bin/env python3
"""
rssi_calibrate.py — RSSI vs. Distance calibration for the WSN path-loss model.

Connects to the robot_esp serial console (USB-UART0), parses anchor-beacon
RSSI log lines, and guides the operator through collecting samples at known
distances from a single wall-mounted anchor node.

After all distances are collected, fits:
    Ptx − RSSI(d) = PL(d0) + 10·n·log10(d / d0)
and prints the PL(d0) and n values ready to paste into robot_esp/main/main.c.

Usage
-----
    pip install pyserial numpy          # one-time
    python tools/rssi_calibrate.py --port COM5
    python tools/rssi_calibrate.py --port /dev/ttyUSB0 --duration 30 --anchor 0x00

Hardware setup
--------------
  • Wall-mounted node: routing_anchor firmware  (broadcasts Cmd 0x00 beacons)
  • Robot ESP32-C6 connected to PC via USB      (logs RSSI over serial console)
  • Gateway (coordinator) must also be powered  (so robot_esp can join the PAN)

Workflow
--------
  1. Run this script.
  2. Place the robot at your first measured distance from the wall anchor.
  3. Press Enter and type the distance in metres (e.g. 1.0).
  4. Script collects for --duration seconds, then prints per-distance stats.
  5. Move robot to next distance; repeat from step 2.
  6. Type 'q' when finished.  Script fits the model and saves a CSV.
"""

import argparse
import csv
import math
import re
import sys
import threading
import time
from datetime import datetime

# ── Log-line regex ───────────────────────────────────────────────────────────
# Matches handle_beacon() output, e.g.:
#   I (12345) ROBOT_ESP: Anchor ID=0x00 (from 0x1a2b): pos=(0,0,200) cm
#             TxPwr=5 dBm  RSSI=-65 dBm  PL=70.0 dB  dist~316 cm
_BEACON_RE = re.compile(
    r'ROBOT_ESP: Anchor ID=0x([0-9a-fA-F]+)'
    r'.*?TxPwr=(-?\d+) dBm'
    r'.*?RSSI=(-?\d+) dBm',
    re.IGNORECASE,
)

# ── Shared sample queue (reader thread → main thread) ────────────────────────
_queue: list = []
_queue_lock = threading.Lock()
_stop_reader = threading.Event()


def _serial_reader(port: str, baud: int) -> None:
    """Background thread: read serial lines and parse beacon RSSI entries."""
    try:
        import serial
    except ImportError:
        print("[ERROR] pyserial not installed.  Run: pip install pyserial",
              file=sys.stderr)
        _stop_reader.set()
        return

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as exc:
        print(f"[ERROR] Cannot open {port}: {exc}", file=sys.stderr)
        _stop_reader.set()
        return

    print(f"[INFO]  Serial {port} @ {baud} baud — reading …\n")
    while not _stop_reader.is_set():
        try:
            raw = ser.readline()
        except Exception:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip()
        m = _BEACON_RE.search(line)
        if m:
            with _queue_lock:
                _queue.append((
                    time.monotonic(),
                    int(m.group(1), 16),   # anchor_id
                    int(m.group(2)),        # tx_power_dbm
                    int(m.group(3)),        # rssi_dbm
                ))
    ser.close()


def _collect(duration_s: float, anchor_filter) -> list:
    """
    Flush stale samples, then collect fresh ones for `duration_s` seconds.
    Returns list of (anchor_id, tx_power_dbm, rssi_dbm) tuples.
    anchor_filter: int or None (None = accept all anchor IDs)
    """
    with _queue_lock:
        _queue.clear()      # discard samples from while the user was walking

    deadline = time.monotonic() + duration_s
    gathered = []
    next_print = time.monotonic() + 5.0

    while time.monotonic() < deadline:
        with _queue_lock:
            while _queue:
                _, aid, txp, rssi = _queue.pop(0)
                if anchor_filter is None or aid == anchor_filter:
                    gathered.append((aid, txp, rssi))

        if time.monotonic() >= next_print:
            remaining = deadline - time.monotonic()
            n = len(gathered)
            if n:
                mean_r = sum(s[2] for s in gathered) / n
                print(f"  {remaining:5.0f} s left — {n} sample(s), "
                      f"running mean RSSI = {mean_r:.1f} dBm", flush=True)
            else:
                print(f"  {remaining:5.0f} s left — waiting for beacons …",
                      flush=True)
            next_print = time.monotonic() + 5.0

        time.sleep(0.05)

    return gathered


def _fit_pathloss(records: list, d0: float = 1.0):
    """
    OLS fit of PL(d) = PL(d0) + 10·n·log10(d/d0).
    records: list of dicts with 'distance_m' and 'mean_pl_db' keys.
    Returns (pl_d0, n, r_squared) or (None, None, None) if numpy missing.
    """
    try:
        import numpy as np
    except ImportError:
        return None, None, None

    x = np.array([math.log10(r["distance_m"] / d0) for r in records])
    y = np.array([r["mean_pl_db"] for r in records])

    # y = slope·x + intercept   →   slope = 10·n,  intercept = PL(d0)
    coeffs = np.polyfit(x, y, 1)
    slope, intercept = float(coeffs[0]), float(coeffs[1])
    n_fit  = slope / 10.0
    pl_d0  = intercept

    y_hat  = slope * x + intercept
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2     = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    return pl_d0, n_fit, r2


def main() -> None:
    ap = argparse.ArgumentParser(
        description="RSSI vs distance calibration for WSN path-loss model"
    )
    ap.add_argument("--port",     required=True,
                    help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud",     type=int, default=115200,
                    help="Baud rate (default: 115200)")
    ap.add_argument("--duration", type=float, default=30.0,
                    help="Collection window per distance in seconds (default: 30)")
    ap.add_argument("--anchor",   default=None,
                    help="Only use this anchor ID, e.g. 0x00 (default: any)")
    ap.add_argument("--output",   default=None,
                    help="CSV output path (default: auto-named with timestamp)")
    args = ap.parse_args()

    anchor_filter = int(args.anchor, 16) if args.anchor else None
    if args.output is None:
        args.output = f"rssi_cal_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    # Start background serial reader
    reader = threading.Thread(
        target=_serial_reader, args=(args.port, args.baud), daemon=True
    )
    reader.start()
    time.sleep(1.0)     # let the port open

    if _stop_reader.is_set():
        sys.exit(1)

    af_label = f"0x{anchor_filter:02x}" if anchor_filter is not None else "any"
    print("=" * 60)
    print("  WSN RSSI Calibration")
    print("=" * 60)
    print(f"  Port        : {args.port}")
    print(f"  Duration    : {args.duration:.0f} s per distance")
    print(f"  Anchor ID   : {af_label}")
    print(f"  Output CSV  : {args.output}")
    print()
    print("  Place the robot at each distance, press Enter, and type the")
    print("  distance in metres.  Type 'q' when all distances are done.")
    print("=" * 60)
    print()

    records = []

    while True:
        try:
            cmd = input("Distance (m) — or 'q' to finish and fit: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if cmd.lower() == "q":
            break

        try:
            dist_m = float(cmd)
            if dist_m <= 0:
                raise ValueError("must be positive")
        except ValueError as exc:
            print(f"  [!] Invalid distance ({exc}).  Enter a positive number.")
            continue

        print(f"  Collecting {args.duration:.0f} s of RSSI data at {dist_m:.2f} m …")
        samples = _collect(args.duration, anchor_filter)

        if not samples:
            print("  [WARN] No beacon samples received — is the anchor powered and "
                  "in range?  Try this distance again.")
            continue

        rssi_vals  = [s[2] for s in samples]
        tx_powers  = [s[1] for s in samples]
        n          = len(rssi_vals)
        mean_rssi  = sum(rssi_vals) / n
        mean_tx    = sum(tx_powers) / n
        mean_pl    = mean_tx - mean_rssi
        variance   = sum((r - mean_rssi) ** 2 for r in rssi_vals) / n
        std_rssi   = math.sqrt(variance)

        print(f"  → {n} sample(s) | mean RSSI = {mean_rssi:.1f} dBm | "
              f"σ = {std_rssi:.1f} dBm | mean PL = {mean_pl:.1f} dB")
        print()

        records.append({
            "distance_m":   dist_m,
            "n_samples":    n,
            "mean_rssi_dbm": round(mean_rssi, 2),
            "std_rssi_db":  round(std_rssi, 2),
            "mean_tx_dbm":  round(mean_tx, 1),
            "mean_pl_db":   round(mean_pl, 2),
        })

    _stop_reader.set()

    # ── Summary ──────────────────────────────────────────────────────────────
    print()
    if len(records) < 2:
        print("[WARN] Need at least 2 distance points to fit a model.")
    else:
        print(f"=== Collected {len(records)} distance point(s) ===")
        for r in records:
            print(f"  {r['distance_m']:5.2f} m  :  mean RSSI = {r['mean_rssi_dbm']:6.1f} dBm  "
                  f"σ = {r['std_rssi_db']:.1f} dBm  (n={r['n_samples']})")

        pl_d0, n_fit, r2 = _fit_pathloss(records)

        if pl_d0 is not None:
            # Propagate average σ_RSSI through the distance formula
            avg_sigma = math.sqrt(
                sum(r["std_rssi_db"] ** 2 for r in records) / len(records)
            )
            dist_frac_pct = (math.log(10) / (10 * n_fit)) * avg_sigma * 100

            print()
            print("=== Path-Loss Fit  (d₀ = 1 m) ===")
            print(f"  PL(d₀)       = {pl_d0:.1f} dB")
            print(f"  n            = {n_fit:.2f}   (expected indoor: 2.0–4.0)")
            print(f"  R²           = {r2:.3f}")
            print(f"  σ_RSSI avg   = {avg_sigma:.1f} dB  →  per-anchor "
                  f"distance uncertainty ≈ {dist_frac_pct:.0f}%")
            print()
            print("  ─── Copy these values into robot_esp/main/main.c ───")
            print(f"  #define PATHLOSS_D0_DB  {pl_d0:.1f}f")
            print(f"  #define PATHLOSS_EXP    {n_fit:.2f}f")
        else:
            print()
            print("[INFO] Install numpy to get automatic path-loss fit:")
            print("       pip install numpy")

    # ── CSV ──────────────────────────────────────────────────────────────────
    if records:
        with open(args.output, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
            writer.writeheader()
            writer.writerows(records)
        print()
        print(f"Data saved → {args.output}")


if __name__ == "__main__":
    main()
