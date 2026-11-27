#!/usr/bin/env python3
"""Run the day 6 study: two drives, two processes, one real socket.

Each point in the grid starts two `tsim_tandem_node` processes, one per side,
on two free loopback ports, with a shared CLOCK_MONOTONIC start instant so they
pace to the same timeline. Each node prints a SUMMARY line, this reads it, and
the two of them together are one point of the bilateral pair.

What the grid measures: how stiff a rendered coupling a delayed link carries
before the pair generates energy, with no passivity controller, with the one
port controller day 5 shipped, and with the two port controller that needs the
other end's energy and therefore needs this link.

The boundary rule is the one docs/results.md states for day 5: the pair is
active when the sum of both ports' generated power exceeds 10 mW over the run.
Generated power is `-min(accumulated port energy) / duration` from each drive's
own observer, which is comparable across the three controller modes because it
is measured at each port and not read from the controller.

Usage:
    python3 tools/run_day6.py                 # the full grid
    python3 tools/run_day6.py --quick         # one delay, one controller
    python3 tools/run_day6.py --trace         # the two traces the figure needs
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import socket
import subprocess
import sys
import time

NODE = pathlib.Path("build/dev/harness/tsim_tandem_node")
OUT_DIR = pathlib.Path("data/run_latest")

DELAYS_MS = [0.5, 1.0, 2.0, 4.0, 8.0]
STIFFNESSES = [20.0, 40.0, 70.0, 100.0, 150.0, 220.0, 320.0]
CONTROLLERS = ["off", "one-port", "two-port"]

# The rendered damping is small for the reason day 5 wrote down: the impedance
# mode multiplies it by rotor speed, and a 0.2 N m s/rad damper rails the current
# limit on the release. The same 0.02 is used in both days so the two boundaries
# differ by the thing being studied and not by a retune.
DAMPING_NM_S_PER_RAD = 0.02
GAP_RAD = 0.002
# 0.3 s at 20 kHz. Shorter than day 5's 0.5 s because the lockstep costs a round
# trip per window: 250 us here, so half a second of simulated time costs two and
# a half seconds of wall clock and the grid has a hundred and five points in it.
# Thirty milliseconds of release and the last thirty of ring down are still many
# periods of the softest coupling in the grid.
TICKS = 6000
TIMEOUT_S = 120.0
# No wall clock and no barrier: the nodes lock step window by window over the
# socket, so the pair advances at the speed of the slower process and the two
# timelines cannot separate. See the comment at the top of tandem_node.cpp for
# why a paced wall clock was abandoned.


def free_port() -> int:
    """A port the kernel says is free right now, closed before the node asks for
    it. The race is the kernel handing the same port to something else in the
    microseconds between, which on a quiet loopback is not a race worth a
    protocol."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def parse_summary(stdout: str) -> dict[str, str]:
    for line in stdout.splitlines():
        if line.startswith("SUMMARY "):
            fields: dict[str, str] = {}
            for token in line.split()[1:]:
                key, _, value = token.partition("=")
                fields[key] = value
            return fields
    return {}


def run_pair(
    delay_ms: float,
    stiffness: float,
    controller: str,
    ticks: int,
    master_csv: pathlib.Path,
    far_csv: pathlib.Path,
) -> list[dict[str, str]]:
    if not NODE.exists():
        raise SystemExit(f"{NODE} is missing, build the harness first")
    master_port = free_port()
    far_port = free_port()
    common = [
        "--delay-ms", f"{delay_ms}",
        "--stiffness", f"{stiffness}",
        "--damping", f"{DAMPING_NM_S_PER_RAD}",
        "--gap-rad", f"{GAP_RAD}",
        "--controller", controller,
        "--ticks", str(ticks),
    ]
    master_csv.parent.mkdir(parents=True, exist_ok=True)
    master = subprocess.Popen(
        [str(NODE), "--role", "master", "--local-port", str(master_port),
         "--peer-port", str(far_port), "--out", str(master_csv), *common],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    far = subprocess.Popen(
        [str(NODE), "--role", "far", "--local-port", str(far_port),
         "--peer-port", str(master_port), "--out", str(far_csv), *common],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    master_out, master_err = master.communicate(timeout=TIMEOUT_S)
    far_out, far_err = far.communicate(timeout=TIMEOUT_S)
    for name, code, err in (("master", master.returncode, master_err),
                            ("far", far.returncode, far_err)):
        if code != 0:
            raise SystemExit(f"{name} node exited {code}: {err.strip()}")
    rows = []
    for summary in (parse_summary(master_out), parse_summary(far_out)):
        if not summary:
            raise SystemExit("a node printed no SUMMARY line")
        rows.append(summary)
    return rows


def sweep(quick: bool) -> None:
    delays = DELAYS_MS[:2] if quick else DELAYS_MS
    stiffnesses = STIFFNESSES[2:3] if quick else STIFFNESSES
    controllers = ["two-port"] if quick else CONTROLLERS

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUT_DIR / "day6_pair.csv"
    columns = [
        "delay_ms", "stiffness", "controller", "role", "growth", "generated_own_w",
        "generated_pair_w", "energy_final_j", "gap_max_rad", "gap_final_rad", "peak_late_rad_s",
        "latency_avg_us", "latency_max_us", "delivered", "timeouts", "skew_max",
        "saturated", "faults",
    ]
    # One discarded pair first. The first processes of a run pay for cold
    # caches and first touch page faults, and measured they overrun their pacing
    # by a few hundred windows where every later pair overruns by none. A sweep
    # whose first point is its worst point is measuring the loader.
    run_pair(1.0, 70.0, "two-port", 2000, OUT_DIR / "day6_warmup_master.csv",
             OUT_DIR / "day6_warmup_far.csv")

    with out_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        print(f"{'delay':>6} {'k':>6} {'controller':>10} {'pair W':>9} {'gap rad':>8} "
              f"{'growth':>7} {'to':>3} {'skew':>5}")
        for delay_ms in delays:
            for stiffness in stiffnesses:
                for controller in controllers:
                    rows = run_pair(
                        delay_ms, stiffness, controller, TICKS,
                        OUT_DIR / "day6_master.csv", OUT_DIR / "day6_far.csv",
                    )
                    # Both ports, each measured at its own accumulator. Comparable
                    # across the three controllers, which the pair excess is not: in
                    # two port mode it already contains the far end's energy.
                    pair_power = sum(float(row["generated_own_w"]) for row in rows)
                    gap = max(float(row["gap_max_rad"]) for row in rows)
                    for row in rows:
                        writer.writerow({
                            "delay_ms": f"{delay_ms:.3f}",
                            "stiffness": f"{stiffness:.1f}",
                            "controller": controller,
                            "role": row["role"],
                            "growth": row["growth"],
                            "generated_own_w": row["generated_own_w"],
                            "generated_pair_w": row["generated_pair_w"],
                            "energy_final_j": row["energy_final_j"],
                            "gap_max_rad": row["gap_max_rad"],
                            "gap_final_rad": row["gap_final_rad"],
                            "peak_late_rad_s": row["peak_late_rad_s"],
                            "latency_avg_us": row["latency_avg_us"],
                            "latency_max_us": row["latency_max_us"],
                            "delivered": row["delivered"],
                            "timeouts": row["timeouts"],
                            "skew_max": row["skew_max"],
                            "saturated": row["saturated"],
                            "faults": row["faults"],
                        })
                    growth = max(float(row["growth"]) for row in rows)
                    timeouts = max(int(row["timeouts"]) for row in rows)
                    skew = max(int(row["skew_max"]) for row in rows)
                    print(f"{delay_ms:6.2f} {stiffness:6.0f} {controller:>10} "
                          f"{pair_power:9.4f} {gap:8.4f} {growth:7.3f} {timeouts:3d} {skew:5d}")
    print(f"wrote {out_path}")

    print()
    print("peak gap over the run, rad, controller against controller:")
    print(f"  {'delay':>6} {'k':>6} {'off':>9} {'one-port':>9} {'two-port':>9}")
    with out_path.open(newline="") as handle:
        table = list(csv.DictReader(handle))
    for delay_ms in delays:
        for stiffness in stiffnesses:
            cells = {}
            for controller in controllers:
                values = [float(row["gap_max_rad"]) for row in table
                          if abs(float(row["delay_ms"]) - delay_ms) < 1e-6
                          and abs(float(row["stiffness"]) - stiffness) < 1e-6
                          and row["controller"] == controller]
                cells[controller] = max(values) if values else float("nan")
            print(f"  {delay_ms:6.2f} {stiffness:6.0f} "
                  + " ".join(f"{cells[c]:9.4f}" for c in controllers))


def trace() -> None:
    """One pair per controller at the point the report quotes. One call writes
    both sides, because one pair is two processes and not two runs."""
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for controller in ("off", "two-port"):
        run_pair(
            2.0, 70.0, controller, TICKS,
            OUT_DIR / f"day6_trace_{controller}_master.csv",
            OUT_DIR / f"day6_trace_{controller}_far.csv",
        )
        print(f"trace {controller} at 2 ms, 70 N m/rad, both sides written")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="two points instead of the grid")
    parser.add_argument("--trace", action="store_true", help="the two traces the figure uses")
    arguments = parser.parse_args()
    if arguments.trace:
        trace()
        return 0
    sweep(arguments.quick)
    return 0


if __name__ == "__main__":
    sys.exit(main())
