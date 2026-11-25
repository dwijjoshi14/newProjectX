#!/usr/bin/env python3
"""Plot the day 4 figures: the position loop's frequency response from a swept
sine, the rendered impedance against a wall, and the lost motion of a joint with
backlash.

Every figure is labelled simulated and stamped with the commit that produced the
data. The numbers printed here are the numbers docs/results.md quotes, from the
same arrays the figures are drawn from.

Usage:
    python3 tools/plot_day4.py --commit "$(git rev-parse --short HEAD)"

The bode figure wants three sweeps: the hand with the encoder the parameters
table assumed, the same hand with a quiet encoder, and a rigid coupling:
    tsim_harness --scenario swept_sine
    tsim_harness --scenario swept_sine --encoder-noise-deg 0.01 \
        --out data/run_latest/day4_sweep_quiet.csv
    tsim_harness --scenario swept_sine --load rigid --out data/run_latest/day4_sweep_rigid.csv
The two hand sweeps are the day's real finding: the drive derates itself to the
noise its encoder reports, so the same joint closes at 1.3 Hz with 0.05 deg of
noise and at 6.7 Hz with 0.01 deg.
The contact figure wants two stiffness runs:
    tsim_harness --scenario wall_contact
    tsim_harness --scenario wall_contact --stiffness 4 --out data/run_latest/day4_wall_stiff4.csv
The backlash figure wants a rigid run and a one degree run:
    tsim_harness --scenario backlash
    tsim_harness --scenario backlash --backlash-deg 1 --out data/run_latest/day4_backlash_1deg.csv
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
THIRD = "#15803d"
LIMIT = "#b42318"

WALL_ANGLE_RAD = 0.05


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
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


def sinusoid_fit(time_s: np.ndarray, signal: np.ndarray, frequency_hz: float) -> complex:
    """Complex amplitude of a signal at a known frequency.

    Least squares against sine, cosine and a constant. The frequency is the one
    the harness generated, not one estimated from the data, because it is exact
    in integers and an estimate would only add its own error to the phase.
    """
    omega = 2.0 * np.pi * frequency_hz
    design = np.column_stack([np.sin(omega * time_s), np.cos(omega * time_s), np.ones_like(time_s)])
    coefficients, *_ = np.linalg.lstsq(design, signal, rcond=None)
    return complex(coefficients[0], coefficients[1])


def frequency_response(data: dict[str, np.ndarray], dwell_floor_s: float = 0.06) -> list[dict]:
    """One complex measurement per sweep frequency.

    T is position over reference, measured as complex amplitudes of the same
    window. S is the sensitivity, 1 - T, because the reference and the error are
    both known exactly: the harness wrote both. The loop transfer is L = T / S,
    which is what a phase margin is read off, and it comes from the same run as
    the closed loop response instead of from a model of it.
    """
    time_s = data["time_ms"] * 1.0e-3
    points = []
    for frequency_hz in np.unique(data["sweep_hz"]):
        if frequency_hz <= 0.0:
            continue
        mask = (data["sweep_hz"] == frequency_hz) & (data["sweep_measure"] > 0.5)
        if np.count_nonzero(mask) < 8:
            continue
        window_s = time_s[mask]
        reference = sinusoid_fit(window_s, data["theta_ref_rad"][mask], frequency_hz)
        measured = sinusoid_fit(window_s, data["theta_rad"][mask], frequency_hz)
        if abs(reference) < 1.0e-9:
            continue
        transfer = measured / reference
        sensitivity = 1.0 - transfer
        loop = transfer / sensitivity if abs(sensitivity) > 1.0e-9 else complex(np.inf, 0.0)
        # The residual is the part of the response the fit does not explain,
        # which is how much nonlinearity or noise is sitting under this point.
        design = np.column_stack([np.sin(2.0 * np.pi * frequency_hz * window_s),
                                  np.cos(2.0 * np.pi * frequency_hz * window_s),
                                  np.ones_like(window_s)])
        residual = data["theta_rad"][mask] - design @ np.array(
            [measured.imag, measured.real, data["theta_rad"][mask].mean()])
        points.append({
            "frequency_hz": float(frequency_hz),
            "transfer": transfer,
            "sensitivity": sensitivity,
            "loop": loop,
            "reference_a": abs(reference),
            "residual_rad": float(np.sqrt(np.mean(residual ** 2))),
            "window_s": float(window_s[-1] - window_s[0]),
        })
    return points


def describe(points: list[dict]) -> list[str]:
    notes = []
    for point in points:
        transfer = point["transfer"]
        loop = point["loop"]
        notes.append(
            f"{point['frequency_hz']:7.1f} Hz  |T| {20.0 * np.log10(abs(transfer)):7.2f} dB  "
            f"angle T {np.degrees(np.angle(transfer)):8.1f} deg  "
            f"|L| {20.0 * np.log10(abs(loop)):7.2f} dB  "
            f"angle L {np.degrees(np.angle(loop)):8.1f} deg"
        )
    return notes


def crossover(points: list[dict]) -> tuple[float, float]:
    """Frequency where the loop gain crosses unity, and the phase margin there.

    Interpolated in log frequency between the two points that straddle 0 dB, in
    dB for the magnitude and linearly for the phase. With thirteen points over
    two decades the interpolation error is a fraction of a degree, and the
    alternative is a model of a measurement.
    """
    ordered = sorted(points, key=lambda p: p["frequency_hz"])
    for before, after in zip(ordered, ordered[1:]):
        magnitude_before = 20.0 * np.log10(abs(before["loop"]))
        magnitude_after = 20.0 * np.log10(abs(after["loop"]))
        if magnitude_before >= 0.0 > magnitude_after:
            span = magnitude_before - magnitude_after
            fraction = magnitude_before / span if span > 0.0 else 0.0
            log_frequency = np.log(before["frequency_hz"]) + fraction * (
                np.log(after["frequency_hz"]) - np.log(before["frequency_hz"]))
            phase_before = np.degrees(np.angle(before["loop"]))
            phase_after = np.degrees(np.angle(after["loop"]))
            if phase_after - phase_before > 180.0:
                phase_after -= 360.0
            if phase_before - phase_after > 180.0:
                phase_after += 360.0
            phase = phase_before + fraction * (phase_after - phase_before)
            return float(np.exp(log_frequency)), 180.0 + phase
    return float("nan"), float("nan")


def bandwidth(points: list[dict]) -> float:
    """Frequency where the closed loop magnitude falls through -3 dB."""
    ordered = sorted(points, key=lambda p: p["frequency_hz"])
    for before, after in zip(ordered, ordered[1:]):
        magnitude_before = 20.0 * np.log10(abs(before["transfer"]))
        magnitude_after = 20.0 * np.log10(abs(after["transfer"]))
        if magnitude_before >= -3.0 > magnitude_after:
            span = magnitude_before - magnitude_after
            fraction = (magnitude_before + 3.0) / span if span > 0.0 else 0.0
            log_frequency = np.log(before["frequency_hz"]) + fraction * (
                np.log(after["frequency_hz"]) - np.log(before["frequency_hz"]))
            return float(np.exp(log_frequency))
    return float("nan")


def gain_margin_db(points: list[dict]) -> float:
    """Negative of the loop magnitude where the phase passes -180 degrees."""
    ordered = sorted(points, key=lambda p: p["frequency_hz"])
    for before, after in zip(ordered, ordered[1:]):
        phase_before = np.degrees(np.angle(before["loop"]))
        phase_after = np.degrees(np.angle(after["loop"]))
        if phase_before > -180.0 >= phase_after:
            span = phase_before - phase_after
            fraction = (phase_before + 180.0) / span if span > 0.0 else 0.0
            magnitude = (20.0 * np.log10(abs(before["loop"])) + fraction *
                         (20.0 * np.log10(abs(after["loop"])) -
                          20.0 * np.log10(abs(before["loop"]))))
            return float(-magnitude)
    return float("nan")


def bode_figure(hand: list[dict], hand_quiet: list[dict], rigid: list[dict], commit: str,
                out_path: pathlib.Path) -> list[str]:
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.4, 6.9), sharex=True)

    curves = ((hand, MEASURED, "hand, 0.05 deg encoder", "-"),
              (hand_quiet, SECONDARY, "hand, 0.01 deg encoder", "-"),
              (rigid, THIRD, "rigid coupling", "--"))
    for points, colour, label, dash in curves:
        frequency = np.array([p["frequency_hz"] for p in points])
        transfer = np.array([p["transfer"] for p in points])
        # Unwrapped, because a phase that jumps from -184 to +176 between two
        # points is the same phase and a reader should not have to notice that.
        transfer_phase = np.degrees(np.unwrap(np.angle(transfer)))
        top.semilogx(frequency, 20.0 * np.log10(np.abs(transfer)), dash, color=colour,
                     linewidth=1.6, marker="o", markersize=3.2, label=f"|T|, {label}")
        bottom.semilogx(frequency, transfer_phase, dash, color=colour,
                        linewidth=1.4, marker="o", markersize=3.0, label=f"angle T, {label}")

    # The loop gain is only drawn for the hand with the encoder the table
    # assumed, because three loop gains on one axis is a plot nobody reads. The
    # other two are in results.md.
    frequency = np.array([p["frequency_hz"] for p in hand])
    loop = np.array([p["loop"] for p in hand])
    top.semilogx(frequency, 20.0 * np.log10(np.abs(loop)), ":", color=INK, linewidth=1.3,
                 marker="s", markersize=2.6, label="|L|, hand 0.05 deg")
    bottom.semilogx(frequency, np.degrees(np.unwrap(np.angle(loop))), ":", color=INK,
                    linewidth=1.3, marker="s", markersize=2.6, label="angle L, hand 0.05 deg")

    top.axhline(-3.0, color=LIMIT, linewidth=0.9, linestyle="--")
    top.axhline(0.0, color=GRID, linewidth=0.9)
    # One label per case, at the top of the plot where nothing else is drawn, so
    # the lines do not have to be identified by eye against the curves.
    for points, colour, _label, _dash in curves:
        frequency_3db = bandwidth(points)
        if np.isfinite(frequency_3db):
            top.axvline(frequency_3db, color=colour, linewidth=0.9, linestyle="--", alpha=0.8)
            top.text(frequency_3db, 18.0, f" -3 dB {frequency_3db:.1f} Hz", color=colour,
                     fontsize=7.2, ha="left", va="top")
    top.set_ylim(-45.0, 20.0)
    style(top, "", "magnitude, dB",
          "Closed loop position response and loop gain, stepped sine, 0.5 to 150 Hz, 0.1 rad peak")
    handles, labels = top.get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.945), ncol=2,
                  frameon=False, fontsize=7.6)

    bottom.axhline(-180.0, color=LIMIT, linewidth=0.9, linestyle="--")
    annotation = []
    for points, colour, label, _dash in curves:
        frequency_cross, margin_deg = crossover(points)
        frequency_3db = bandwidth(points)
        if np.isfinite(frequency_cross):
            bottom.axvline(frequency_cross, color=colour, linewidth=0.9, linestyle=":", alpha=0.7)
        annotation.append(
            f"{label}: -3 dB {frequency_3db:.1f} Hz, crossover {frequency_cross:.1f} Hz, "
            f"margin {margin_deg:.0f} deg")
    if annotation:
        bottom.annotate("\n".join(annotation), xy=(0.02, 0.04), xycoords="axes fraction",
                        fontsize=7.0, color=INK)
    bottom.set_ylim(-260.0, 30.0)
    bottom.set_yticks([-240.0, -180.0, -120.0, -60.0, 0.0])
    style(bottom, "frequency, Hz", "phase, deg",
          "Phase of the loop gain, and of the closed loop response for each case")
    bottom.legend(frameon=False, fontsize=7.4, loc="lower right", ncol=2)

    stamp(figure, commit,
          "Simulated. Switching plant, sensing chain on, 500 ns dead time, 0.1 rad peak. The drive\n"
          "commissions from the encoder noise it is told, so the two hand curves are different loops. "
          "Not measured.")
    figure.tight_layout(rect=(0.0, 0.06, 1.0, 0.90))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)

    notes = []
    for points, _colour, label, _dash in curves:
        frequency_cross, margin_deg = crossover(points)
        notes.append(
            f"{label}: -3 dB at {bandwidth(points):.2f} Hz, crossover {frequency_cross:.2f} Hz, "
            f"phase margin {margin_deg:.1f} deg, gain margin {gain_margin_db(points):.1f} dB")
    return notes


def wall_figure(runs: list[tuple[str, float, dict[str, np.ndarray], str]], commit: str,
                out_path: pathlib.Path) -> list[str]:
    """Contact against a wall, and what force the drive actually renders.

    The measured force is not plotted against penetration, because the wall's
    own stiffness would set that slope and it would only be reporting the wall.
    The quantity a person feels is how much torque the drive produces for a
    commanded excursion past the wall, and that is the reference angle minus the
    wall angle: 0.1 rad in this run.
    """
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.4, 6.6))
    notes = []
    commanded_rad = 0.15 - WALL_ANGLE_RAD  # reference past the wall, rad

    labels = []
    commanded_forces = []
    rendered_forces = []
    for label, stiffness, data, colour in runs:
        time_ms = data["time_ms"]
        mask = (time_ms >= 40.0) & (time_ms <= 260.0)
        top.plot(time_ms[mask], data["theta_load_rad"][mask] * 1000.0, color=colour, linewidth=1.6,
                 label=f"handle at {label}")
        top.plot(time_ms[mask], data["theta_ref_rad"][mask] * 1000.0, color=colour, linewidth=0.9,
                 linestyle=":", label=f"reference at {label}")

        # Static contact: the last 40 ms before the release, where the joint has
        # stopped moving and the force is what the rendered spring is pushing.
        settled = (time_ms >= 210.0) & (time_ms <= 250.0)
        force_nm = float(np.mean(data["torque_wall_nm"][settled]))
        rendered_stiffness = force_nm / commanded_rad
        labels.append(f"{stiffness:.1f} N m/rad")
        commanded_forces.append(stiffness * commanded_rad)
        rendered_forces.append(force_nm)
        notes.append(
            f"commanded {stiffness:.1f} N m/rad renders {rendered_stiffness:.3f} N m/rad at the "
            f"handle, {force_nm * 1000.0:.1f} mN m for a {commanded_rad * 1000.0:.0f} mrad "
            f"excursion past the wall against {stiffness * commanded_rad * 1000.0:.0f} mN m asked "
            f"for, ratio {force_nm / (stiffness * commanded_rad):.3f}")

    top.axhline(WALL_ANGLE_RAD * 1000.0, color=LIMIT, linewidth=1.1, linestyle="--")
    top.annotate("wall at 50 mrad", xy=(45.0, WALL_ANGLE_RAD * 1000.0), xytext=(0.0, 4.0),
                 textcoords="offset points", fontsize=8, color=LIMIT)
    style(top, "time, ms", "handle angle, mrad",
          "Pushing a rendered spring into a wall, then releasing")
    top.legend(frameon=False, fontsize=7.5, loc="lower right", ncol=2)

    positions = np.arange(len(labels))
    width = 0.34
    bottom.bar(positions - width / 2.0, np.array(commanded_forces) * 1000.0, width,
               color=REFERENCE, label="commanded")
    bottom.bar(positions + width / 2.0, np.array(rendered_forces) * 1000.0, width,
               color=MEASURED, label="rendered at the handle")
    for position, commanded, rendered in zip(positions, commanded_forces, rendered_forces):
        bottom.annotate(f"{100.0 * rendered / commanded:.0f} percent",
                        xy=(position + width / 2.0, rendered * 1000.0), xytext=(0.0, 3.0),
                        textcoords="offset points", ha="center", fontsize=8, color=MEASURED)
    bottom.set_ylim(0.0, 1.12 * max(max(commanded_forces), max(rendered_forces)) * 1000.0)
    bottom.set_xticks(positions)
    bottom.set_xticklabels(labels)
    style(bottom, "commanded stiffness", "torque at the wall, mN m",
          "Torque produced for 100 mrad past the wall: what the drive renders, not what it was asked for")

    stamp(figure, commit,
          "Simulated. Switching plant, impedance mode, 0.05 N m s/rad damping, wall at 50 mrad, "
          "1.8 N m/rad. Coulomb friction 8 mN m. Not measured.")
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 0.94))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)
    return notes


def backlash_figure(runs: list[tuple[str, dict[str, np.ndarray], str]], commit: str,
                    out_path: pathlib.Path) -> list[str]:
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.4, 6.2), sharex=True)
    notes = []

    # The relative angle is the figure, not the absolute one: what a person feels
    # is how far the handle moves before the shaft catches it, and that is the
    # gap and nothing else.
    half_gap_mrad = 8.73
    for label, data, colour in runs:
        top.plot(data["time_ms"], (data["theta_load_rad"] - data["theta_rad"]) * 1000.0,
                 color=colour, linewidth=1.3, label=label)
    for sign in (-1.0, 1.0):
        top.axhline(sign * half_gap_mrad, color=LIMIT, linewidth=0.9, linestyle="--")
    top.annotate("half the backlash, 8.7 mrad", xy=(60.0, half_gap_mrad), xytext=(0.0, 5.0),
                 textcoords="offset points", fontsize=8, color=LIMIT)
    top.set_ylim(-12.0, 12.0)
    style(top, "", "handle minus shaft, mrad",
          "Through a 1 Hz triangle: the handle moves inside the gap and the shaft catches it")
    handles, labels = top.get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.945), ncol=3,
                  frameon=False, fontsize=8.0)

    bottom.axvspan(-half_gap_mrad, half_gap_mrad, color=GRID, alpha=0.55, zorder=0)
    for label, data, colour in runs:
        difference_mrad = (data["theta_load_rad"] - data["theta_rad"]) * 1000.0
        inside = np.abs(difference_mrad) < 5.0
        notes.append(f"{label}: torque at zero relative angle "
                     f"{1000.0 * float(np.mean(data['torque_coupling_nm'][inside])):+.1f} mN m, "
                     f"largest {1000.0 * float(np.max(np.abs(data['torque_coupling_nm']))):.0f} mN m, "
                     f"relative angle reached {float(np.max(np.abs(difference_mrad))):.1f} mrad")
        if label.startswith("0"):
            # A joint with no backlash has a coupling torque proportional to the
            # relative angle, so at this scale it is one line through the origin
            # and it says more in the note than in the ink.
            continue
        bottom.plot(difference_mrad, data["torque_coupling_nm"] * 1000.0, color=colour,
                    linewidth=0.9, alpha=0.85, label=f"{label} shaft")

    for sign in (-1.0, 1.0):
        bottom.axvline(sign * half_gap_mrad, color=LIMIT, linewidth=1.0, linestyle="--")
    bottom.annotate(f"half the backlash, {half_gap_mrad:.1f} mrad", xy=(half_gap_mrad, 0.0),
                    xytext=(6.0, 22.0), textcoords="offset points", fontsize=8, color=LIMIT)
    bottom.axhline(0.0, color=GRID, linewidth=0.8)
    bottom.set_xlim(-20.0, 20.0)
    bottom.set_ylim(-260.0, 260.0)
    style(bottom, "handle minus shaft, mrad", "torque in the coupling, mN m",
          "Inside the shaded gap the coupling carries nothing: that is the lost motion")
    bottom.legend(frameon=False, fontsize=8, loc="upper left")

    stamp(figure, commit,
          "Simulated. Switching plant, position mode, hand attached, 250 N m/rad shaft at "
          "2e-3 N m s/rad, 8 mN m Coulomb friction. Not measured.")
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 0.90))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)
    return notes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep-csv", default="data/run_latest/day4_sweep.csv")
    parser.add_argument("--sweep-quiet-csv", default="data/run_latest/day4_sweep_quiet.csv")
    parser.add_argument("--sweep-rigid-csv", default="data/run_latest/day4_sweep_rigid.csv")
    parser.add_argument("--wall-csv", default="data/run_latest/day4_wall_stiff1.csv")
    parser.add_argument("--wall-stiff-csv", default="data/run_latest/day4_wall_stiff4.csv")
    parser.add_argument("--backlash-csv", default="data/run_latest/day4_backlash_0.csv")
    parser.add_argument("--backlash-deg-csv", default="data/run_latest/day4_backlash_1deg.csv")
    parser.add_argument("--commit", required=True, help="commit that produced the data")
    parser.add_argument("--outdir", default="docs/figures")
    args = parser.parse_args()

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    hand = frequency_response(load(pathlib.Path(args.sweep_csv)))
    hand_quiet = frequency_response(load(pathlib.Path(args.sweep_quiet_csv)))
    rigid = frequency_response(load(pathlib.Path(args.sweep_rigid_csv)))
    notes = bode_figure(hand, hand_quiet, rigid, args.commit,
                        outdir / "day4_position_bode.png")
    for note in notes:
        print(note)
    print(f"wrote {outdir / 'day4_position_bode.png'}")
    for name, points in (("hand, 0.05 deg encoder", hand),
                         ("hand, 0.01 deg encoder", hand_quiet),
                         ("rigid coupling", rigid)):
        print(f"{name} sweep:")
        for line in describe(points):
            print("  " + line)

    wall_notes = wall_figure(
        [("1.0 N m/rad commanded", 1.0, load(pathlib.Path(args.wall_csv)), MEASURED),
         ("4.0 N m/rad commanded", 4.0, load(pathlib.Path(args.wall_stiff_csv)), SECONDARY)],
        args.commit, outdir / "day4_impedance.png")
    for note in wall_notes:
        print(note)
    print(f"wrote {outdir / 'day4_impedance.png'}")

    backlash_notes = backlash_figure(
        [("0 deg backlash", load(pathlib.Path(args.backlash_csv)), MEASURED),
         ("1 deg backlash", load(pathlib.Path(args.backlash_deg_csv)), SECONDARY)],
        args.commit, outdir / "day4_backlash.png")
    for note in backlash_notes:
        print(note)
    print(f"wrote {outdir / 'day4_backlash.png'}")


if __name__ == "__main__":
    main()
