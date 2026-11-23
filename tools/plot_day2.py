#!/usr/bin/env python3
"""Plot the day 2 traces from the deterministic harness.

Both figures are labelled simulated and stamped with the commit that produced
the data, because a figure whose provenance is a guess is not evidence. The
summary printed here is the same number the figure shows, so results.md and the
picture cannot disagree.

Usage:
    python3 tools/plot_day2.py --commit "$(git rev-parse --short HEAD)"
"""

from __future__ import annotations

import argparse
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
LIMIT = "#b42318"


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
    table = np.genfromtxt(path, delimiter=",", names=True)
    return {name: np.atleast_1d(table[name]) for name in table.dtype.names}


def style(axis: plt.Axes, xlabel: str, ylabel: str, title: str) -> None:
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.set_title(title, loc="left", fontsize=9.5, color=INK)
    axis.grid(True, color=GRID, linewidth=0.6)
    axis.set_axisbelow(True)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.spines["left"].set_color(GRID)
    axis.spines["bottom"].set_color(GRID)
    axis.tick_params(colors=INK, labelsize=8)


def stamp(figure: plt.Figure, commit: str) -> None:
    figure.suptitle(
        f"TANDEM-SIM, simulated, commit {commit}",
        x=0.01,
        ha="left",
        fontsize=9,
        color=REFERENCE,
    )
    figure.text(
        0.01,
        0.005,
        "Deterministic harness, ideal plant, FOC firmware. Simulated, not measured.",
        fontsize=7,
        color=REFERENCE,
    )


def crossing_time_ms(time_ms: np.ndarray, values: np.ndarray, level: float) -> float:
    indices = np.flatnonzero(values >= level)
    if indices.size == 0:
        return float("nan")
    index = int(indices[0])
    if index == 0:
        return float(time_ms[0])
    previous = float(values[index - 1])
    rise = float(values[index]) - previous
    if rise <= 0.0:
        return float(time_ms[index])
    return float(time_ms[index - 1] + (level - previous) / rise * (time_ms[index] - time_ms[index - 1]))


def current_figure(data: dict[str, np.ndarray], commit: str, out_path: pathlib.Path) -> None:
    time_ms = data["time_ms"]
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.0, 5.6), sharex=True)
    top.plot(time_ms, data["iq_ref_a"], color=REFERENCE, linewidth=1.2, linestyle="--",
             label="q axis reference")
    top.plot(time_ms, data["iq_a"], color=MEASURED, linewidth=1.6, label="q axis measured")
    top.plot(time_ms, data["id_a"], color=SECONDARY, linewidth=1.1, label="d axis measured")
    top.axhline(0.0, color=GRID, linewidth=0.8)
    style(top, "", "current, A", "Current loop, clamped rotor, q axis reference 2 A, -2 A, 0 A")
    top.legend(frameon=False, fontsize=8, loc="lower right", ncol=3)

    bottom.plot(time_ms, data["ia_a"], color=INK, linewidth=1.1)
    style(
        bottom,
        "time, ms",
        "phase A current, A",
        "Phase A current, with the q axis parked on phase A",
    )
    stamp(figure, commit)
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 0.94))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)


def speed_figure(data: dict[str, np.ndarray], commit: str, out_path: pathlib.Path) -> None:
    time_ms = data["time_ms"]
    load_indices = np.flatnonzero(data["load_nm"] > 0.0)
    load_ms = float(time_ms[load_indices[0]]) if load_indices.size else 0.0

    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.0, 5.8), sharex=True)
    top.plot(time_ms, data["omega_ref_rad_s"], color=REFERENCE, linewidth=1.2, linestyle="--",
             label="reference")
    top.plot(time_ms, data["omega_rad_s"], color=MEASURED, linewidth=1.6, label="measured")
    top.axvline(load_ms, color=LIMIT, linewidth=0.9, linestyle=":")
    top.annotate("50 mNm load step", xy=(load_ms, 0.55), xycoords=("data", "axes fraction"),
                 xytext=(4.0, 0.0), textcoords="offset points", fontsize=8, color=LIMIT)
    style(top, "", "rotor speed, rad/s", "Speed loop step to 200 rad/s, then a 50 mNm load step")
    top.legend(frameon=False, fontsize=8, loc="lower right", ncol=2)

    bottom.plot(time_ms, data["iq_a"], color=MEASURED, linewidth=1.4, label="q axis measured")
    bottom.axhline(2.74, color=LIMIT, linewidth=0.8, linestyle="--")
    bottom.axhline(-2.74, color=LIMIT, linewidth=0.8, linestyle="--")
    bottom.annotate("continuous current, 2.74 A", xy=(0.01, 2.74), xycoords=("axes fraction", "data"),
                    xytext=(0.0, 3.0), textcoords="offset points", fontsize=7.5, color=LIMIT)
    bottom.set_ylim(-0.5, 3.2)
    style(bottom, "time, ms", "q axis current, A", "Current command the speed loop produced")
    stamp(figure, commit)
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 0.94))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current-csv", default="data/run_latest/day2_current_step.csv")
    parser.add_argument("--speed-csv", default="data/run_latest/day2_speed_step.csv")
    parser.add_argument("--commit", required=True, help="commit that produced the data")
    parser.add_argument("--outdir", default="docs/figures")
    args = parser.parse_args()

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    current = load(pathlib.Path(args.current_csv))
    speed = load(pathlib.Path(args.speed_csv))

    current_figure(current, args.commit, outdir / "day2_current_step.png")
    speed_figure(speed, args.commit, outdir / "day2_speed_step.png")

    # Numbers quoted in docs/results.md, from the same arrays the figures use.
    time_ms = current["time_ms"]
    iq = current["iq_a"]
    first_step = time_ms < 2.0
    t10 = crossing_time_ms(time_ms[first_step], iq[first_step], 0.1 * 2.0)
    t90 = crossing_time_ms(time_ms[first_step], iq[first_step], 0.9 * 2.0)
    settled = iq[(time_ms >= 1.5) & (time_ms < 2.0)]
    print(f"current loop: rise 10-90 {1000.0 * (t90 - t10):.0f} us, "
          f"steady {settled.mean():.4f} A, peak {iq.max():.4f} A, "
          f"peak |id| {np.fabs(current['id_a']).max():.4f} A")

    speed_ms = speed["time_ms"]
    omega = speed["omega_rad_s"]
    peak = float(omega.max())
    band = 0.02 * 200.0
    outside = np.flatnonzero(np.fabs(omega - 200.0) > band)
    settle_ms = float(speed_ms[outside[-1]]) if outside.size else 0.0
    print(f"speed loop: peak {peak:.2f} rad/s, settling to 2 percent at {settle_ms:.1f} ms, "
          f"final {omega[-1]:.2f} rad/s, final iq {speed['iq_a'][-1]:.3f} A")
    print(f"wrote {outdir / 'day2_current_step.png'} and {outdir / 'day2_speed_step.png'}")


if __name__ == "__main__":
    main()
