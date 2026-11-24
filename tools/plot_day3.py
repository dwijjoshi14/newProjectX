#!/usr/bin/env python3
"""Plot the day 3 traces: dead time distortion and the sampling instant.

Both figures are labelled simulated and stamped with the commit that produced
the data. The numbers printed here are the numbers docs/results.md quotes, from
the same arrays the figures are drawn from, so the prose and the picture cannot
disagree.

Usage:
    python3 tools/plot_day3.py --commit "$(git rev-parse --short HEAD)"

The dead time figure wants three runs of the same scenario:
    tsim_harness --scenario dead_time --dead-time-ns 0 --compensation off \\
        --out data/run_latest/day3_dead_time_none.csv
    tsim_harness --scenario dead_time --compensation off
    tsim_harness --scenario dead_time --compensation on \\
        --out data/run_latest/day3_dead_time_compensated.csv
The sampling figure wants one run of tsim_sample_probe.
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

SAMPLE_RATE_HZ = 20000.0

# A Hann windowed sinusoid spreads over five bins, and the sum of their squared
# magnitudes is 1.2247 squared times the single bin value. Every amplitude below
# is taken from a two bin band and divided by this, which is what makes the
# printed fundamental agree with the q axis current in the harness summary.
HANN_BAND_AMPLITUDE = 1.2247


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


def stamp(figure: plt.Figure, commit: str, note: str) -> None:
    figure.suptitle(f"TANDEM-SIM, simulated, commit {commit}", x=0.01, ha="left", fontsize=9,
                    color=REFERENCE)
    figure.text(0.01, 0.005, note, fontsize=7, color=REFERENCE)


def harmonic_analysis(signal_a: np.ndarray, maximum_harmonic: int = 15) -> dict[str, float]:
    """THD of a sampled phase current, referred to its own fundamental.

    Hann window, then the power in a two bin band around the fundamental and
    around each harmonic. The record is an integer number of electrical cycles
    at the nominally chosen speed, but the speed loop holds that speed and not
    the number, so the bands are what keeps a one percent frequency error from
    being reported as distortion.
    """
    count = signal_a.size
    window = np.hanning(count)
    spectrum = np.abs(np.fft.rfft((signal_a - signal_a.mean()) * window))
    frequencies = np.fft.rfftfreq(count, d=1.0 / SAMPLE_RATE_HZ)
    search = (frequencies > 10.0) & (frequencies < 400.0)
    fundamental_bin = int(np.argmax(np.where(search, spectrum, 0.0)))
    fundamental_hz = float(frequencies[fundamental_bin])

    def band_power(centre_bin: int) -> float:
        low = max(centre_bin - 2, 0)
        high = min(centre_bin + 3, spectrum.size)
        return float(np.sum(spectrum[low:high] ** 2))

    fundamental_power = band_power(fundamental_bin)
    harmonic_power = np.zeros(maximum_harmonic + 1)
    for order in range(2, maximum_harmonic + 1):
        centre = int(round(fundamental_bin * order))
        if centre + 2 < spectrum.size:
            harmonic_power[order] = band_power(centre)

    thd = float(np.sqrt(np.sum(harmonic_power[2:]) / fundamental_power))
    to_amplitude = 4.0 / (count * HANN_BAND_AMPLITUDE)
    return {
        "fundamental_hz": fundamental_hz,
        "fundamental_a": float(np.sqrt(fundamental_power) * to_amplitude),
        "thd": thd,
        "fifth_a": float(np.sqrt(harmonic_power[5]) * to_amplitude),
        "seventh_a": float(np.sqrt(harmonic_power[7]) * to_amplitude),
        "harmonics": harmonic_power,
        "fundamental_bin": fundamental_bin,
    }


def fundamental_fit(time_s: np.ndarray, signal_a: np.ndarray, frequency_hz: float) -> np.ndarray:
    """Least squares sine and cosine plus a DC term, at a fitted frequency.

    The frequency is searched around the FFT estimate rather than taken from it,
    because the speed loop holds 30 rad/s and not 33.333 Hz exactly, and one
    hundredth of a hertz of error over ten cycles leaves a residual larger than
    the distortion this figure is about. Fitting the frequency is the difference
    between plotting the dead time notch and plotting the frequency error.
    """
    best_fit = np.zeros_like(signal_a)
    best_error = float("inf")
    for factor in np.linspace(0.97, 1.03, 241):
        omega = 2.0 * np.pi * frequency_hz * factor
        design = np.column_stack(
            [np.sin(omega * time_s), np.cos(omega * time_s), np.ones_like(time_s)]
        )
        coefficients, *_ = np.linalg.lstsq(design, signal_a, rcond=None)
        fit = design @ coefficients
        error = float(np.sum((signal_a - fit) ** 2))
        if error < best_error:
            best_error = error
            best_fit = fit
    return best_fit


def dead_time_figure(
    runs: list[tuple[str, dict[str, np.ndarray], str]],
    commit: str,
    out_path: pathlib.Path,
) -> list[str]:
    notes = []
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.4, 6.2), sharex=True)

    for label, data, colour in runs:
        time_ms = data["time_ms"]
        signal_a = data["ia_a"]
        analysis = harmonic_analysis(signal_a[-3000:])
        mask = (time_ms >= 150.0) & (time_ms <= 210.0)
        window_s = time_ms[mask] * 1.0e-3
        fit = fundamental_fit(window_s, signal_a[mask], analysis["fundamental_hz"])
        residual_a = signal_a[mask] - fit
        top.plot(time_ms[mask], signal_a[mask], color=colour, linewidth=1.3, label=label)
        bottom.plot(time_ms[mask], 1000.0 * residual_a, color=colour, linewidth=1.1, label=label)
        notes.append(
            f"{label}: fundamental {analysis['fundamental_a']:.3f} A at "
            f"{analysis['fundamental_hz']:.2f} Hz, THD {100.0 * analysis['thd']:.2f} percent, "
            f"5th {1000.0 * analysis['fifth_a']:.1f} mA, 7th "
            f"{1000.0 * analysis['seventh_a']:.1f} mA"
        )

    top.axhline(0.0, color=GRID, linewidth=0.8)
    style(top, "", "phase A current, A",
          "Phase A current at 30 rad/s and 1.95 A, switching plant, sensing chain off")
    handles, labels = top.get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.945), ncol=3,
                  frameon=False, fontsize=8.5)

    bottom.axhline(0.0, color=GRID, linewidth=0.8)
    style(bottom, "time, ms", "residual, mA",
          "Residual after removing the fundamental: the dead time notch, and what the compensation does")
    stamp(figure, commit,
          "Simulated. Deterministic harness, switching plant, FOC firmware. 500 ns dead time, "
          "20 kHz, 36 V bus. Not measured.")
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 0.90))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)
    return notes


def sampling_figure(data: dict[str, np.ndarray], commit: str, out_path: pathlib.Path) -> list[str]:
    offset_us = data["offset_us"]
    current_a = data["ia_a"]
    error_a = data["error_a"]
    mean_a = float(current_a.mean())

    figure, (top, bottom) = plt.subplots(2, 1, figsize=(8.0, 5.8), sharex=True)

    top.plot(offset_us, current_a, color=MEASURED, linewidth=1.6, label="winding current")
    top.axhline(mean_a, color=REFERENCE, linewidth=1.0, linestyle="--",
                label="mean over the switching period")
    top.axvline(25.0, color=THIRD, linewidth=1.1, linestyle=":",
                label="sample instant, window centre")
    style(top, "", "phase A current, A",
          "One switching period of the winding current, 20 A on a clamped rotor")
    handles, labels = top.get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.945), ncol=3,
                  frameon=False, fontsize=8.5)

    bottom.plot(offset_us, 1000.0 * error_a, color=SECONDARY, linewidth=1.4)
    bottom.axhline(0.0, color=GRID, linewidth=0.8)
    bottom.axvline(25.0, color=THIRD, linewidth=1.1, linestyle=":")
    bottom.annotate("sample here, error zero", xy=(25.0, 0.0),
                    xytext=(27.0, 0.55 * 1000.0 * error_a.max()), fontsize=8, color=THIRD)
    worst_index = int(np.argmax(np.abs(error_a)))
    bottom.annotate(f"{1000.0 * error_a[worst_index]:+.0f} mA at {offset_us[worst_index]:.0f} us",
                    xy=(offset_us[worst_index], 1000.0 * error_a[worst_index]),
                    xytext=(8.0, 6.0), textcoords="offset points", fontsize=8, color=SECONDARY)
    style(bottom, "offset from the window start, us", "sampling error, mA",
          "What the converter would read at each instant, minus the period mean")
    stamp(figure, commit,
          "Simulated. Switching plant, locked rotor, 500 ns dead time, no sensing chain. "
          "Not measured.")
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 0.90))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)

    peak_to_peak = float(current_a.max() - current_a.min())
    centre_error = float(error_a[24])
    return [
        f"ripple {1000.0 * peak_to_peak:.0f} mA peak to peak on a {mean_a:.3f} A current",
        f"sample at the centre {1000.0 * centre_error:+.1f} mA from the mean",
        f"worst instant {1000.0 * error_a[worst_index]:+.0f} mA at {offset_us[worst_index]:.0f} us",
    ]


def switching_speed_figure(data: dict[str, np.ndarray], commit: str, out_path: pathlib.Path) -> None:
    """The day 2 speed step, run through the switching inverter.

    One panel. The d axis trace from this run is deliberately not plotted: with
    the sensing chain on it is dominated by ADC quantisation and encoder noise
    at plus or minus 50 mA, which is the same size as the transient error the
    feedforward removed, and a panel that noisy would invite the reader to
    compare it with the day 2 trace and draw the wrong conclusion about why it
    changed.
    """
    time_ms = data["time_ms"]
    load_indices = np.flatnonzero(data["load_nm"] > 0.0)
    load_ms = float(time_ms[load_indices[0]]) if load_indices.size else 0.0

    figure, axis = plt.subplots(figsize=(8.0, 4.6))
    axis.plot(time_ms, data["omega_ref_rad_s"], color=REFERENCE, linewidth=1.2, linestyle="--",
              label="reference")
    axis.plot(time_ms, data["omega_rad_s"], color=MEASURED, linewidth=1.5, label="measured")
    axis.axvline(load_ms, color=LIMIT, linewidth=0.9, linestyle=":")
    axis.annotate("50 mNm load step", xy=(load_ms, 0.6), xycoords=("data", "axes fraction"),
                  xytext=(4.0, 0.0), textcoords="offset points", fontsize=8, color=LIMIT)
    style(axis, "time, ms", "rotor speed, rad/s",
          "Speed step to 200 rad/s and a load step, switching inverter, sensing chain on")
    axis.legend(frameon=False, fontsize=8, loc="lower right", ncol=2)
    stamp(figure, commit,
          "Simulated. Deterministic harness, switching plant, 500 ns dead time, sensing chain on. "
          "Not measured.")
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 0.93))
    figure.savefig(out_path, dpi=150)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--none-csv", default="data/run_latest/day3_dead_time_none.csv")
    parser.add_argument("--dead-csv", default="data/run_latest/day3_dead_time.csv")
    parser.add_argument("--comp-csv", default="data/run_latest/day3_dead_time_compensated.csv")
    parser.add_argument("--sample-csv", default="data/run_latest/day3_sample_instant.csv")
    parser.add_argument("--speed-csv", default="data/run_latest/day3_speed_step_switching.csv")
    parser.add_argument("--commit", required=True, help="commit that produced the data")
    parser.add_argument("--outdir", default="docs/figures")
    args = parser.parse_args()

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    runs = [
        ("no dead time", load(pathlib.Path(args.none_csv)), REFERENCE),
        ("500 ns, no compensation", load(pathlib.Path(args.dead_csv)), LIMIT),
        ("500 ns, compensated", load(pathlib.Path(args.comp_csv)), THIRD),
    ]
    notes = dead_time_figure(runs, args.commit, outdir / "day3_dead_time.png")
    for note in notes:
        print(note)
    print(f"wrote {outdir / 'day3_dead_time.png'}")

    sample_notes = sampling_figure(load(pathlib.Path(args.sample_csv)), args.commit,
                                   outdir / "day3_sampling_instant.png")
    for note in sample_notes:
        print(note)
    print(f"wrote {outdir / 'day3_sampling_instant.png'}")

    switching_speed_figure(load(pathlib.Path(args.speed_csv)), args.commit,
                           outdir / "day3_speed_step.png")
    print(f"wrote {outdir / 'day3_speed_step.png'}")


if __name__ == "__main__":
    main()
