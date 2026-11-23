#!/usr/bin/env python3
"""Independent model of the day 2 current loop, for comparison against a run.

The harness and this script are two implementations of the same discrete loop:
the same PI, the same zero order hold winding, the same sample 25 us into the
window and the same one window delay before the new voltage applies. If they
disagree, one of them has a timing bug and the figure is wrong. The continuous
first order approximation is printed as well, to show how far it is from both.

The gains below mirror firmware/src/firmware.c. They are duplicated on purpose:
a reference implementation that reads its gains from the code under test is not
a reference.

Usage:
    python3 tools/reference_current_loop.py [--csv PATH] [--reference-a 2.0]
"""

from __future__ import annotations

import argparse
import pathlib

import numpy as np

# Mirrors docs/parameters.md and firmware/src/firmware.c, all cited there.
RESISTANCE_OHM = 0.291
INDUCTANCE_H = 2.56e-4
KP_V_PER_A = 1.61
KI_V_PER_A_S = 1830.0
PERIOD_S = 50.0e-6


def step_response(ticks: int, reference_a: float) -> tuple[np.ndarray, np.ndarray]:
    """Sampled q current for a reference step, one window per element.

    The order is the harness order and it matters. The voltage latched for this
    window drives the winding for the whole window, the sample is taken halfway
    through it, and the new voltage is only computed at the end, so it applies
    from the next window. Applying the new voltage for the rest of the current
    window is a different loop with no delay, and it does not look like one.
    """

    def evolve(current_a: float, voltage_v: float, h_s: float) -> float:
        decay = np.exp(-RESISTANCE_OHM * h_s / INDUCTANCE_H)
        return current_a * decay + voltage_v / RESISTANCE_OHM * (1.0 - decay)

    samples = np.zeros(ticks)
    times = np.zeros(ticks)
    current_a = 0.0   # at the start of the window
    voltage_v = 0.0   # already latched for this window
    integral_v = 0.0

    for tick in range(ticks):
        samples[tick] = evolve(current_a, voltage_v, PERIOD_S / 2.0)
        times[tick] = (tick + 0.5) * PERIOD_S
        current_a = evolve(current_a, voltage_v, PERIOD_S)

        error_a = reference_a - samples[tick]
        integral_v += KI_V_PER_A_S * PERIOD_S * error_a
        voltage_v = KP_V_PER_A * error_a + integral_v

    return times, samples


def crossing(times: np.ndarray, values: np.ndarray, level: float) -> float:
    index = int(np.flatnonzero(values >= level)[0])
    if index == 0:
        return float(times[0])
    previous = float(values[index - 1])
    rise = float(values[index]) - previous
    if rise <= 0.0:
        return float(times[index])
    return float(times[index - 1] + (level - previous) / rise * (times[index] - times[index - 1]))


def measured(csv_path: pathlib.Path, reference_a: float) -> tuple[float, float]:
    table = np.genfromtxt(csv_path, delimiter=",", names=True)
    times = np.atleast_1d(table["time_ms"]) * 1.0e-3
    current = np.atleast_1d(table["iq_a"])
    first = times < 2.0e-3
    rise_s = crossing(times[first], current[first], 0.9 * reference_a) - crossing(
        times[first], current[first], 0.1 * reference_a
    )
    settled = current[(times >= 1.5e-3) & (times < 2.0e-3)]
    return rise_s, float(settled.mean())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", default="data/run_latest/day2_current_step.csv")
    parser.add_argument("--reference-a", type=float, default=2.0)
    parser.add_argument("--ticks", type=int, default=120)
    args = parser.parse_args()

    times, samples = step_response(args.ticks, args.reference_a)
    model_rise_s = crossing(times, samples, 0.9 * args.reference_a) - crossing(
        times, samples, 0.1 * args.reference_a
    )
    # Same window the measurement uses, before the reference changes again.
    settled = samples[(times >= 1.5e-3) & (times < 2.0e-3)]
    model_steady_a = float(np.mean(settled))

    csv_path = pathlib.Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"no run at {csv_path}, run the harness first")

    measured_rise_s, measured_steady_a = measured(csv_path, args.reference_a)
    print(f"reference model : rise {model_rise_s * 1e6:.0f} us, steady {model_steady_a:.4f} A")
    print(f"harness         : rise {measured_rise_s * 1e6:.0f} us, steady {measured_steady_a:.4f} A")
    print(
        f"deviation       : rise {100.0 * abs(measured_rise_s - model_rise_s) / model_rise_s:.1f} pct, "
        f"steady {100.0 * abs(measured_steady_a - model_steady_a) / model_steady_a:.2f} pct"
    )
    tau_s = INDUCTANCE_H / KP_V_PER_A
    print(f"continuous estimate: rise {2.2 * tau_s * 1e6:.0f} us, one pole at Kp / L = {1.0 / tau_s:.0f} rad/s")


if __name__ == "__main__":
    main()
