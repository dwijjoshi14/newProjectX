#!/usr/bin/env python3
"""Plot the day 6 figures: what two drives over a real socket do.

Three figures, all from data/run_latest:
    day6_pair.png     one pair at 2 ms and 70 N m/rad, with and without the two
                      port controller: the two angles, the gap between them, and
                      the energy that crossed both ports
    day6_gap.png      the peak gap across the whole grid, controller against
                      controller. A bilateral pair fails by failing to close it
    day6_latency.png  the transport's own latency, measured on every packet

Every figure is labelled simulated and stamped with the commit that produced it.

Usage:
    python3 tools/run_day6.py            # the grid
    python3 tools/run_day6.py --trace    # the traces this needs
    python3 tools/plot_day6.py --commit "$(git rev-parse --short HEAD)"
"""

from __future__ import annotations

import argparse
import csv
import pathlib

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

INK = "#14161a"
GRID = "#d8dce2"
REFERENCE = "#7c828c"
MEASURED = "#1f4fd8"
SECONDARY = "#c2410c"
THIRD = "#15803d"
LIMIT = "#b42318"

DATA = pathlib.Path("data/run_latest")
FIGURES = pathlib.Path("docs/figures")


def load_table(path: pathlib.Path) -> dict[str, np.ndarray]:
    table = np.genfromtxt(path, delimiter=",", names=True)
    return {name: np.atleast_1d(table[name]) for name in table.dtype.names}


def style(axis: plt.Axes, xlabel: str, ylabel: str, title: str) -> None:
    axis.set_xlabel(xlabel, color=INK)
    axis.set_ylabel(ylabel, color=INK)
    axis.set_title(title, color=INK, loc="left", fontsize=10)
    axis.grid(True, color=GRID, linewidth=0.6)
    axis.tick_params(colors=INK, labelsize=8)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axis.spines[side].set_color(GRID)


def footnote(figure: plt.Figure, text: str) -> None:
    figure.text(0.01, 0.01, text, color=REFERENCE, fontsize=7, ha="left")


def pair_figure(commit: str) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(10.0, 6.4), sharex=True)
    for column, controller in enumerate(("off", "two-port")):
        master = load_table(DATA / f"day6_trace_{controller}_master.csv")
        far = load_table(DATA / f"day6_trace_{controller}_far.csv")
        time_ms = master["time_ms"]
        gap_rad = master["theta_own_rad"] - master["theta_peer_rad"]
        top = axes[0][column]
        top.plot(time_ms, master["theta_own_rad"], color=MEASURED, linewidth=1.2,
                 label="master angle")
        top.plot(time_ms, master["theta_peer_rad"], color=SECONDARY, linewidth=1.2,
                 label="far angle, as the master received it")
        top.plot(time_ms, gap_rad, color=THIRD, linewidth=1.0, linestyle="--",
                 label="gap between them")
        style(top, "time, ms", "angle, rad",
              f"{'no controller' if controller == 'off' else 'two port controller'}")

        bottom = axes[1][column]
        bottom.plot(time_ms, master["energy_j"], color=MEASURED, linewidth=1.2,
                    label="master port energy")
        bottom.plot(time_ms, far["energy_j"], color=SECONDARY, linewidth=1.2,
                    label="far port energy")
        bottom.plot(time_ms, master["excess_j"], color=THIRD, linewidth=1.0, linestyle="--",
                    label="pair excess")
        bottom.axhline(0.0, color=REFERENCE, linewidth=0.8)
        style(bottom, "time, ms", "energy, J", "")
    handles, labels = axes[0][0].get_legend_handles_labels()
    handles2, labels2 = axes[1][0].get_legend_handles_labels()
    figure.legend(handles + handles2, labels + labels2, frameon=False, fontsize=8,
                  labelcolor=INK, loc="lower center", bbox_to_anchor=(0.5, 0.055), ncol=3)
    figure.suptitle("Two drives, two processes, one socket at 2 ms and 70 N m/rad, simulated",
                    color=INK, fontsize=11, ha="left", x=0.01)
    figure.tight_layout(rect=(0.0, 0.16, 1.0, 0.96))
    footnote(figure, f"simulated, not measured. commit {commit}. "
                     "The far angle is the delayed sample the master rendered against, "
                     "which is what the drive saw.")
    figure.texts[-1].set_y(0.005)
    figure.savefig(FIGURES / "day6_pair.png", dpi=150)
    plt.close(figure)


def gap_figure(commit: str) -> None:
    rows = list(csv.DictReader((DATA / "day6_pair.csv").open(newline="")))
    delays = sorted({float(row["delay_ms"]) for row in rows})
    stiffnesses = sorted({float(row["stiffness"]) for row in rows})
    figure, axes = plt.subplots(1, 3, figsize=(11.0, 3.6), sharey=True)
    colours = [REFERENCE, MEASURED, SECONDARY, THIRD, "#7e22ce"]
    for axis, controller in zip(axes, ("off", "one-port", "two-port")):
        for colour, delay_ms in zip(colours, delays):
            values = []
            for stiffness in stiffnesses:
                points = [float(row["gap_max_rad"]) for row in rows
                          if row["controller"] == controller
                          and abs(float(row["delay_ms"]) - delay_ms) < 1e-6
                          and abs(float(row["stiffness"]) - stiffness) < 1e-6]
                values.append(max(points) if points else np.nan)
            axis.semilogy(stiffnesses, values, color=colour, marker="o", markersize=3,
                          linewidth=1.2, label=f"{delay_ms:g} ms")
        style(axis, "rendered coupling, N m/rad",
              "peak gap, rad" if controller == "off" else "",
              {"off": "no controller", "one-port": "one port",
               "two-port": "two port"}[controller])
        axis.legend(frameon=False, fontsize=7, labelcolor=INK, title="one way delay",
                    title_fontsize=7)
    figure.suptitle("How far apart the two joints get, by coupling and delay, simulated",
                    color=INK, fontsize=11, ha="left", x=0.01)
    figure.tight_layout(rect=(0.0, 0.04, 1.0, 0.94))
    footnote(figure, f"simulated, not measured. commit {commit}. "
                     "0.3 s per run, 2 mrad release, 0.02 N m s/rad rendered damping, "
                     "250 N m/rad shaft. Peak of |master angle - the far angle it received|.")
    figure.savefig(FIGURES / "day6_gap.png", dpi=150)
    plt.close(figure)


def latency_figure(commit: str) -> None:
    master = load_table(DATA / "day6_trace_two-port_master.csv")
    far = load_table(DATA / "day6_trace_two-port_far.csv")
    latency_us = np.concatenate([master["latency_us"], far["latency_us"]])
    latency_us = latency_us[latency_us > 0.0]
    figure, axis = plt.subplots(figsize=(7.0, 3.2))
    axis.hist(latency_us, bins=60, color=MEASURED, alpha=0.85, linewidth=0)
    style(axis, "one way transport latency, us", "packets",
          "Loopback datagram latency, measured on every packet")
    footnote(figure, f"measured, on this machine, during a simulated run. commit {commit}. "
                     f"mean {latency_us.mean():.0f} us, 99th percentile "
                     f"{np.percentile(latency_us, 99):.0f} us, max {latency_us.max():.0f} us.")
    figure.tight_layout(rect=(0.0, 0.06, 1.0, 1.0))
    figure.savefig(FIGURES / "day6_latency.png", dpi=150)
    plt.close(figure)
    print(f"latency: mean {latency_us.mean():.1f} us, p99 {np.percentile(latency_us, 99):.1f} us, "
          f"max {latency_us.max():.1f} us, {latency_us.size} packets")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--commit", default="unknown", help="commit the data came from")
    arguments = parser.parse_args()
    FIGURES.mkdir(parents=True, exist_ok=True)
    pair_figure(arguments.commit)
    gap_figure(arguments.commit)
    latency_figure(arguments.commit)
    print("wrote docs/figures/day6_pair.png, day6_gap.png, day6_latency.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
