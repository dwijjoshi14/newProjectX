#!/usr/bin/env python3
"""Plot the day 5 figures: the thermal derating curve and what a stall does to a
winding, the efficiency map, and the stability boundary of two joints over a
delayed link.

Every figure is labelled simulated and stamped with the commit that produced the
data. The numbers printed here are the numbers docs/results.md quotes.

Usage:
    python3 tools/plot_day5.py --commit "$(git rev-parse --short HEAD)"

Data it wants, all of it from the binaries in harness/:
    tsim_thermal --derating
    tsim_thermal --soak --current 2.74 --seconds 1500
    tsim_harness --scenario thermal_soak
    tsim_efficiency
    tsim_bilateral --shaft-stiffness 250
    tsim_bilateral --shaft-stiffness 5000 --out .../day5_stability_rigid.csv
    tsim_bilateral --stiffness-list 70 --delay-list 2 --trace --trace-passivity off
    tsim_bilateral --stiffness-list 70 --delay-list 2 --trace --trace-passivity on
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
FOURTH = "#7e22ce"
LIMIT = "#b42318"

CONTINUOUS_CURRENT_A = 2.74
WINDING_LIMIT_C = 155.0


def load_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def load_table(path: pathlib.Path) -> dict[str, np.ndarray]:
    table = np.genfromtxt(path, delimiter=",", names=True)
    return {name: np.atleast_1d(table[name]) for name in table.dtype.names}


def style(axis: plt.Axes, xlabel: str, ylabel: str, title: str) -> None:
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.set_title(title, loc="left", fontsize=9.5, color=INK)
    axis.grid(True, color=GRID, linewidth=0.6, which="both")
    axis.set_axisbelow(True)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.spines["left"].set_color(GRID)
    axis.spines["bottom"].set_color(GRID)
    axis.tick_params(colors=INK, labelsize=8)


def stamp(figure: plt.Figure, commit: str, note: str) -> None:
    figure.suptitle(f"TANDEM-SIM, simulated, commit {commit}", x=0.01, ha="left", fontsize=9,
                    color=REFERENCE)
    figure.text(0.01, 0.005, note, fontsize=7, color=REFERENCE)


def thermal_figure(derating: list[dict[str, str]], closed_loop: dict[str, np.ndarray],
                   soak: dict[str, np.ndarray], commit: str, out_path: pathlib.Path) -> list[str]:
    notes = []
    figure, (top, middle, bottom) = plt.subplots(3, 1, figsize=(8.4, 9.6))

    ambient = np.array([float(row["ambient_c"]) for row in derating])
    limit = np.array([float(row["limit_current_a"]) for row in derating])
    at_datasheet = np.array([float(row["winding_c_at_datasheet_a"]) for row in derating])
    runaway = float(derating[0]["runaway_current_a"])

    top.plot(ambient, limit, color=MEASURED, linewidth=1.8, marker="o", markersize=3.5,
             label="current that reaches 155 degC")
    top.axhline(CONTINUOUS_CURRENT_A, color=SECONDARY, linewidth=1.4, linestyle="--",
                label=f"datasheet continuous, {CONTINUOUS_CURRENT_A} A")
    top.axhline(runaway, color=LIMIT, linewidth=1.4, linestyle=":",
                label=f"no steady state above {runaway:.2f} A")
    style(top, "ambient, degC", "steady current, A",
          "Derating from the two published resistances: no current settles above the dotted line")
    top.legend(frameon=False, fontsize=7.6, loc="upper right")

    middle.plot(ambient, at_datasheet, color=THIRD, linewidth=1.6, marker="s", markersize=3.2,
                label="winding at the datasheet continuous current")
    middle.axhline(WINDING_LIMIT_C, color=LIMIT, linewidth=1.1, linestyle="--")
    middle.annotate("155 degC insulation limit", xy=(22.0, WINDING_LIMIT_C), xytext=(0.0, -12.0),
                    textcoords="offset points", fontsize=7.6, color=LIMIT)
    middle.axhline(100.0, color=GRID, linewidth=1.0)
    middle.annotate("100 degC, where the drive starts derating the envelope", xy=(22.0, 100.0),
                    xytext=(0.0, 4.0), textcoords="offset points", fontsize=7.4, color=REFERENCE)
    style(middle, "ambient, degC", "winding, degC",
          "The datasheet current is a hot ambient rating: 65 degC on a 25 degC bench")
    middle.legend(frameon=False, fontsize=7.6, loc="lower right")

    time_s = closed_loop["time_ms"] * 1.0e-3
    winding_c = closed_loop["winding_c"]
    soak_time_s = np.maximum(soak["time_s"], 1.0e-3)
    # Log time, because the closed loop settles in twenty seconds and the housing
    # takes a thousand: on one linear axis one of the two is a vertical line.
    bottom.semilogx(time_s, winding_c, color=SECONDARY, linewidth=1.6,
                    label="closed loop, 20 A asked for on a clamped rotor")
    bottom.semilogx(soak_time_s, soak["winding_c"], color=MEASURED, linewidth=1.2,
                    linestyle="--", label="network alone, 2.74 A held for 1500 s")
    bottom.axhline(100.0, color=GRID, linewidth=1.0)
    bottom.set_xlim(0.1, 2000.0)
    bottom2 = bottom.twinx()
    bottom2.semilogx(time_s, closed_loop["thermal_limit_a"], color=THIRD, linewidth=1.1,
                     label="current envelope")
    bottom2.set_xlim(0.1, 2000.0)
    bottom2.set_ylabel("envelope, A", color=THIRD, fontsize=8)
    bottom2.tick_params(colors=THIRD, labelsize=8)
    bottom2.spines["top"].set_visible(False)
    style(bottom, "time, s", "winding, degC",
          "A 20 A stall, and a 2.74 A soak, on a log time axis")
    handles, labels = bottom.get_legend_handles_labels()
    handles2, labels2 = bottom2.get_legend_handles_labels()
    bottom.legend(handles + handles2, labels + labels2, frameon=False, fontsize=7.4,
                  loc="center right")

    notes.append(f"derating at 20 degC ambient {limit[0]:.3f} A, at 100 degC {limit[-1]:.3f} A")
    notes.append(f"runaway current {runaway:.3f} A, ambient independent")
    notes.append(f"winding at the datasheet current {at_datasheet[0]:.1f} degC from 25 degC")
    notes.append(f"closed loop reaches {float(np.max(winding_c)):.1f} degC in "
                 f"{float(time_s[-1]):.0f} s and settles under the 130 degC latch")

    stamp(figure, commit,
          "Simulated. Winding and housing as two lumps: 2.27 K/W and 11.0 J/K inside, 8.17 K/W "
          "and\n125 J/K to the bench, copper 0.00393 per K from 20 degC. The 24 s closed loop run "
          "is the\nreal plant and firmware; the 1500 s curve is the same network integrated alone. "
          "Not measured.")
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.955))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)
    return notes


def efficiency_figure(rows: list[dict[str, str]], commit: str,
                      out_path: pathlib.Path) -> list[str]:
    notes = []
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.4, 7.4))
    torques = sorted({float(row["torque_nm"]) for row in rows})
    colours = [REFERENCE, MEASURED, THIRD, SECONDARY, FOURTH]

    for torque, colour in zip(torques, colours):
        points = [row for row in rows if float(row["torque_nm"]) == torque]
        speed = np.array([float(row["speed_rad_s"]) for row in points])
        efficiency = np.array([float(row["efficiency"]) for row in points])
        top.semilogx(speed, 100.0 * efficiency, color=colour, linewidth=1.6, marker="o",
                     markersize=3.2, label=f"{1000.0 * torque:.0f} mN m")
        notes.append(f"{1000.0 * torque:.0f} mN m: best {100.0 * float(np.max(efficiency)):.1f} "
                     f"percent at {speed[int(np.argmax(efficiency))]:.0f} rad/s")
    style(top, "speed, rad/s", "efficiency, percent",
          "Electrical to mechanical at a 100 degC winding, switching plant")
    top.legend(frameon=False, fontsize=7.8, ncol=5, loc="lower right")
    top.set_ylim(20.0, 102.0)

    at_100 = [row for row in rows if abs(float(row["speed_rad_s"]) - 100.0) < 1.0]
    at_100.sort(key=lambda row: float(row["torque_nm"]))
    positions = np.arange(len(at_100))
    copper = np.array([float(row["copper_w"]) for row in at_100])
    inverter = np.array([float(row["unaccounted_w"]) for row in at_100])
    switching = np.array([float(row["switching_w"]) for row in at_100])
    friction = np.array([float(row["friction_w"]) for row in at_100])
    mechanical = np.array([float(row["mechanical_w"]) for row in at_100])
    width = 0.6
    bottom.bar(positions, mechanical, width, color=MEASURED, label="out of the shaft")
    bottom.bar(positions, copper, width, bottom=mechanical, color=SECONDARY,
               label="copper, at 100 degC")
    bottom.bar(positions, inverter, width, bottom=mechanical + copper, color=LIMIT,
               label="device drop and dead time")
    bottom.bar(positions, switching, width, bottom=mechanical + copper + inverter, color=FOURTH,
               label="switching")
    bottom.bar(positions, friction, width,
               bottom=mechanical + copper + inverter + switching, color=REFERENCE,
               label="bearing and seal")
    bottom.set_xticks(positions)
    bottom.set_xticklabels([f"{1000.0 * float(row['torque_nm']):.0f} mN m" for row in at_100])
    for position, row in zip(positions, at_100):
        total = float(row["electrical_w"])
        bottom.annotate(f"{total:.1f} W", xy=(position, total), xytext=(0.0, 3.0),
                        textcoords="offset points", ha="center", fontsize=7.4, color=INK)
    style(bottom, "load torque at 100 rad/s", "power, W",
          "Where the input goes at 100 rad/s")
    bottom.legend(frameon=False, fontsize=7.4, loc="upper left", ncol=2)

    stamp(figure, commit,
          "Simulated. Winding resistance scaled to 100 degC (x 1.314) with the thermal network off, "
          "sensing chain on,\n500 ns dead time. The device drop and dead time bar is the difference "
          "between the link energy and\nthe winding plus shaft energy, so it is measured by "
          "difference and not by a model. Not measured.")
    figure.tight_layout(rect=(0.0, 0.045, 1.0, 0.945))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)
    return notes


def stability_figure(as_built: list[dict[str, str]], rigid: list[dict[str, str]],
                     trace_off: dict[str, np.ndarray], trace_on: dict[str, np.ndarray],
                     commit: str, out_path: pathlib.Path) -> list[str]:
    notes = []
    figure, (top, middle, bottom) = plt.subplots(3, 1, figsize=(8.4, 9.6))
    run_duration_s = 0.5

    def generated_power(rows: list[dict[str, str]], passivity: str) -> tuple[np.ndarray, ...]:
        delays = sorted({float(row["delay_ms"]) for row in rows})
        stiffness = sorted({float(row["stiffness_nm_per_rad"]) for row in rows})
        grid = np.zeros((len(delays), len(stiffness)))
        for row in rows:
            if row["passivity"] != passivity:
                continue
            row_index = delays.index(float(row["delay_ms"]))
            column_index = stiffness.index(float(row["stiffness_nm_per_rad"]))
            grid[row_index, column_index] = -float(row["excess_min_j"]) / run_duration_s
        return np.array(delays), np.array(stiffness), grid

    for axis, rows, title in (
        (top, as_built,
         "As built: the 250 N m/rad shaft is what limits the render, and the link decides how "
         "the limit fails"),
        (middle, rigid,
         "Rigid transmission: with the shaft out of the way the boundary is the link's, and it "
         "moves with the delay"),
    ):
        delays, stiffness, grid = generated_power(rows, "0")
        colours = [MEASURED, THIRD, SECONDARY, FOURTH, LIMIT]
        for index, delay in enumerate(delays):
            axis.loglog(stiffness, np.maximum(grid[index], 1.0e-4), color=colours[index % 5],
                        linewidth=1.5, marker="o", markersize=3.2, label=f"{delay:g} ms")
            notes.append(f"{title.split(':')[0]}, {delay:g} ms one way: "
                         f"{grid[index, -1]:.3g} W generated at 320 N m/rad")
        axis.axhline(1.0e-2, color=LIMIT, linewidth=1.0, linestyle="--")
        axis.annotate("10 mW: the render is generating", xy=(stiffness[0], 1.0e-2),
                      xytext=(0.0, 4.0), textcoords="offset points", fontsize=7.4, color=LIMIT)
        style(axis, "rendered coupling stiffness, N m/rad", "generated power, W", title)
        axis.legend(frameon=False, fontsize=7.4, ncol=5, loc="upper left", title="one way delay",
                    title_fontsize=7.2)
        axis.set_ylim(1.0e-4, 1.0e4)

    time_ms = trace_off["time_ms"]
    mask = time_ms <= 200.0
    bottom.plot(time_ms[mask], trace_off["theta_master_rad"][mask] * 1000.0, color=SECONDARY,
                linewidth=1.2, label="no passivity control")
    bottom.plot(time_ms[mask], trace_on["theta_master_rad"][mask] * 1000.0, color=MEASURED,
                linewidth=1.2, label="passivity control on")
    bottom.plot(time_ms[mask], trace_off["theta_far_rad"][mask] * 1000.0, color=REFERENCE,
                linewidth=0.9, linestyle=":", label="far joint, no control")
    style(bottom, "time, ms", "joint angle, mrad",
          "70 N m/rad over a 2 ms link: both runs reach the current limit, and the energy "
          "says what changes")
    bottom.legend(frameon=False, fontsize=7.4, loc="upper right", ncol=3)

    off_power = -float(np.min(trace_off["excess_j"])) / run_duration_s
    on_power = -float(np.min(trace_on["excess_j"])) / run_duration_s
    bottom.annotate(f"generated power: {off_power:.1f} W without, {on_power:.1f} W with\n"
                    f"peak angle: {1000.0 * float(np.max(np.abs(trace_off['theta_master_rad']))):.0f} "
                    f"mrad without, "
                    f"{1000.0 * float(np.max(np.abs(trace_on['theta_master_rad']))):.0f} mrad with",
                    xy=(0.02, 0.06), xycoords="axes fraction", fontsize=7.4, color=INK)
    notes.append(f"trace at 70 N m/rad over 2 ms: {off_power:.2f} W generated without the "
                 f"controller, {on_power:.2f} W with it")
    notes.append(f"peak damping the controller asked for, with the controller on: "
                 f"{float(np.max(trace_on['damping_nm_s_per_rad'])):.1f} N m s/rad")

    stamp(figure, commit,
          "Simulated. Master is the switching plant with the unmodified firmware in impedance "
          "mode; the far joint is the same\nmechanics with the link torque as an external torque, "
          "clamped at stall. 2 mrad initial stretch, 20 mrad damping,\n1 ms of jitter and 0.5 "
          "percent loss on each direction. Not measured.")
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.955))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)
    return notes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--derating-csv", default="data/run_latest/day5_derating.csv")
    parser.add_argument("--closed-loop-csv", default="data/run_latest/day5_thermal_soak.csv")
    parser.add_argument("--soak-csv", default="data/run_latest/day5_soak_continuous.csv")
    parser.add_argument("--efficiency-csv", default="data/run_latest/day5_efficiency.csv")
    parser.add_argument("--stability-csv", default="data/run_latest/day5_stability_250.csv")
    parser.add_argument("--stability-rigid-csv",
                        default="data/run_latest/day5_stability_rigid.csv")
    parser.add_argument("--trace-off-csv", default="data/run_latest/day5_bilateral_pc_off.csv")
    parser.add_argument("--trace-on-csv", default="data/run_latest/day5_bilateral_pc_on.csv")
    parser.add_argument("--commit", required=True, help="commit that produced the data")
    parser.add_argument("--outdir", default="docs/figures")
    args = parser.parse_args()

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    notes = thermal_figure(load_csv(pathlib.Path(args.derating_csv)),
                           load_table(pathlib.Path(args.closed_loop_csv)),
                           load_table(pathlib.Path(args.soak_csv)), args.commit,
                           outdir / "day5_thermal.png")
    for note in notes:
        print(note)
    print(f"wrote {outdir / 'day5_thermal.png'}")

    notes = efficiency_figure(load_csv(pathlib.Path(args.efficiency_csv)), args.commit,
                              outdir / "day5_efficiency.png")
    for note in notes:
        print(note)
    print(f"wrote {outdir / 'day5_efficiency.png'}")

    notes = stability_figure(load_csv(pathlib.Path(args.stability_csv)),
                             load_csv(pathlib.Path(args.stability_rigid_csv)),
                             load_table(pathlib.Path(args.trace_off_csv)),
                             load_table(pathlib.Path(args.trace_on_csv)), args.commit,
                             outdir / "day5_stability.png")
    for note in notes:
        print(note)
    print(f"wrote {outdir / 'day5_stability.png'}")


if __name__ == "__main__":
    main()
