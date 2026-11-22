# TANDEM-SIM specification and plan

Owner: Dwij Joshi. Build window: seven working sessions, one per day, from
2026-11-22. This document records the plan, the contracts and every decision
that constrains later work.

## What is being built

A switching level digital twin of a one degree of freedom haptic joint, with
production style field oriented control firmware running inside it unmodified.
Two virtual joints later mirror each other over a lossy link. The engineering
claim is model based design and validation: the plant, the harness and the
firmware are all written here, and the conclusions come from measurements on
that model, not from a physics engine.

## Contracts

1. **Firmware boundary.** `firmware/src` includes `hal.h` and the C standard
   library only. Checked by `tools/check_firmware_boundary.sh` in ctest.
2. **One clock.** `IPlant::step_one()` is the only function that advances time.
   The harness owns the loop. The firmware never sees simulation time.
3. **Fixed step.** Plant 1 MHz, current loop 20 kHz, velocity 2 kHz, position
   1 kHz. No variable step anywhere, in either harness mode.
4. **Parameters.** Every constant is cited in `docs/parameters.md` or marked
   ASSUMED there. The simulator holds no numbers that are not in that table.
5. **Figures.** Every figure is labelled simulated and stamped with the commit
   that produced it. No simulated number is presented as measured.

## Timing, and why the loop is ordered the way it is

One control window is 50 us. Per window:

1. Duty written by the previous tick applies at the window start, like a timer
   shadow register reload.
2. The plant integrates 25 steps of 1 us.
3. The converter latches phase currents and bus voltage, the ADC trigger at the
   window centre.
4. The plant integrates the remaining 25 steps.
5. The firmware tick runs, reads the latched sample and writes the next duty.

Sample to actuation is therefore 25 us to 75 us, mean 50 us, which is one
switching period. That delay is real and shows up in the measured phase margin.
A harness that calls the firmware at the start of the window would report a
phase margin the hardware could never deliver.

## Determinism policy

Deterministic mode reads no clock, schedules on integer nanoseconds, seeds
randomness from the scenario file and writes a `meta.json` beside the run. A
repeated run of the same binary is byte identical. Across compilers, last bit
floating point differences remain, so cross compiler bit identity is not
claimed. The CI reference is generated and compared on Ubuntu x86-64.

Real time mode (day 5) uses absolute deadlines from `steady_clock`, counts
lateness above one period as a deadline miss, and measures per function WCET
with `hal_cpu_cycles()`. It does not change the plant step, so simulation time
can lag wall time and the lag is logged.

## Seven day plan

| Day | Work | Artefact at the end |
|---|---|---|
| 1 | Scaffolding, hal.h, portable target, boundary and size gates, ideal plant, deterministic harness | Repo that builds and tests on a clean clone |
| 2 | Clarke, Park, SVPWM, current and speed loops, validation against analytic plant response | Closed loop single motor, speed and current plot |
| 3 | Real inverter: dead time, Ron, switching loss, bus ripple, compensation; sensing chain | Dead time THD figure, sampling instant figure |
| 4 | Mechanics and load: friction, backlash, two mass, hand, wall; velocity and position cascade; impedance control | Bode plot from swept sine, bandwidth and margin |
| 5 | Thermal, efficiency map, network delay, second joint, bilateral control, passivity observer | Stability boundary, derating curve, efficiency map |
| 6 | Real time harness, jitter and WCET, twin back end and front end | Live twin with instability demo, timing report |
| 7 | CI hardening, docs, results table, portfolio card, resume bullets, video shot list | Shippable repo and application material |

Honest scope note: this is a compressed build. The original estimate was about
214 hours. Seven sessions will not reach the depth of that plan, so each day
ends with a written note on what was cut or reduced.

## Day 1 decisions

- Build system is CMake 3.22 with presets `dev`, `release`, `asan`, `tsan`,
  `ci`. Ninja is the generator because presets cannot choose one conditionally.
- Test framework is in repo and dependency free. No GoogleTest.
- Plant integrates with forward Euler at 1 us. Electrical time constant is
  0.88 ms, so the step is 1/880 of it. A convergence test later will confirm
  the step is small enough rather than assume it.
- The portable target compiles `src/firmware.c` directly with `-ffreestanding`,
  so the freestanding claim covers the real sources, not a proxy file.
- The harness implements `hal.h` itself, so day 1 already exercises the full
  seam: plant, HAL, firmware tick, duty, plant.

## Method notes

- Tests are written before the feature. The day 1 plant tests came before any
  firmware control code, which is why day 1 has no controller.
- Every physical sign convention lives in one place: amplitude invariant
  Clarke, Park with `iq = -i_alpha sin + i_beta cos`, torque
  `T = 1.5 p lambda iq`, back EMF `e_a = -E sin(theta_e)` with
  `E = p lambda omega_m`. The energy balance test is what will catch a mistake.
