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
| 3 | done. Real inverter: dead time, Ron, switching loss, bus ripple, compensation; sensing chain | Dead time THD figure, sampling instant figure |
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

## Day 2 decisions

- Control structure: Clarke, Park and space vector modulation, a PI current
  loop on d and q at 20 kHz, a PI speed loop at 2 kHz whose output is the q
  current reference, and a first order filter in front of the speed reference.
  No other compensator exists yet.
- Current loop gains. Kp = 1.61 V/A puts the crossover at Kp / L =
  1.61 / 0.256 mH = 6.29 krad/s, which is 1.0 kHz. Ki = Kp R / L = 1830 V/(A s)
  puts the PI zero on the winding pole at 1137 rad/s, so the loop reduces to one
  pole at the crossover. The 75 us pipeline delay costs 27 degrees there, which
  leaves about 60. Both gains come from the datasheet row in
  `docs/parameters.md`, not from a tuning session.
- Speed loop gains. Kp = 0.0313 A per rad/s puts the crossover at
  Kp Kt / J = 627 rad/s, about 100 Hz. Ki = 2.95 A/(rad s) puts the integral
  zero at 94 rad/s, roughly a seventh of the crossover, so the margin before
  the sampling lag is 81 degrees. The ten tick velocity estimate and the 2 kHz
  update take about 30 of those back, which is what the measured response shows.
- The speed command is filtered at 10 ms. A step command enters a loop with
  about 50 degrees of margin and comes out as an 18 percent overshoot, measured
  during the day. The filter removes that excitation without touching
  disturbance rejection, and the joint still answers a step in 17 ms.
- The q current reference is clamped at stall current, 61.8 A, not at
  continuous current, because a haptic joint has to be able to answer a peak
  torque and there is no thermal network yet to make the continuous number
  real. The speed loop's output is clamped at 2.74 A, the conservative choice
  while the thermal model is missing.
- Modulation is minimum maximum zero sequence injection with the voltage
  magnitude limited to 95 percent of bus / sqrt(3). No allowance is made for
  dead time, because there is no dead time until day 3.
- Sine and cosine are an eleventh order polynomial after a quadrant fold, worst
  case 1.4e-7, because the firmware may not link libm. The square root used for
  the voltage circle normalises into [1, 4) with exact power of two steps and
  then runs six Newton iterations, which is a fixed cost.
- The closed loop test and the harness share one control window function,
  `harness/src/sim_loop.cpp`, so the timing cannot drift between the thing that
  is measured and the thing that draws the picture.

## Day 2 status and what was cut

One motor, closed loop in current and in speed, with two figures and a measured
comparison against an independent discrete model. Cut or deferred, in the order
they matter:

- No d/q decoupling and no back EMF feedforward. The PI rejects both, and the
  price is visible: 0.071 A of d axis current during a 200 rad/s transient.
  Day 3 adds them, because dead time compensation needs the same voltage
  estimates.
- No field weakening and no overmodulation. The voltage limit is a circle at
  95 percent of the linear range.
- No thermal limit. Current mode can ask for stall current indefinitely, which
  this model cannot punish and a real motor would punish quickly.
- No current sensor noise, offset drift or gain error. The plant currents are
  exact and the offsets are calibrated once at boot.
- The velocity estimate is a ten tick encoder difference with no observer, so
  it lags by up to 0.5 ms and quantises to 0.77 rad/s at 14 bits.
- The speed loop is measured on the bare rotor. At 2.3e-6 kg m^2 a 50 mNm load
  step is a violent disturbance, and most of the 13.7 percent dip is inertia
  rather than tuning. Day 4 puts a handle on it.
- Saturation, cogging, Coulomb friction, backlash and switching remain
  unmodelled, as listed in `docs/model-limits.md`.

## Day 3 decisions

- **The switching plant is a second class, not a mode of the first.** The ideal
  plant stays exactly as it was, so day 1 and day 2 results remain reproducible
  at their commits, and the harness selects between them. The price of two
  implementations of the same electrical equations is that they can drift, so
  `the_switching_plant_reduces_to_the_average_model` runs both and compares
  them sample by sample with the inverter set to ideal. It found a 1e-8
  relative difference, which is float rounding in the average plant's neutral
  calculation, and that is now written down rather than tolerated silently.
- **The carrier is resolved at the plant step, with the edge placed inside the
  step rather than snapped to it.** A 1 MHz plant cannot put an edge on a finer
  grid than 1 us, and 1 us of a 50 us period is 2 percent of the duty, which is
  0.72 V at 36 V. The dead time being studied is 0.36 V. Averaging the fraction
  of each step a leg is high costs one min and one max per leg and brings the
  quantisation error to nothing. The current ripple is then piecewise linear at
  1 us, which is the honest limit of the step.
- **Dead time is the standard average voltage error per leg**, signed by the
  phase current and faded linearly below 0.5 A. Fading is not cosmetic: with
  1.76 A peak to peak of ripple, the current crosses zero inside every period
  near a zero crossing of the fundamental, so a hard sign would add the
  distortion it is supposed to remove.
- **Switching loss is the overlap triangle, not a fixed energy per event.** Day
  1 recorded 20 uJ per device transition, which cannot be right because it does
  not scale with current, and the day 3 energy account showed it burning 4.8 W
  while the motor delivered 10 W. Replaced with a 50 ns crossover, which is the
  same model in the form that has the current in it: half of bus times current
  times the crossover, four device events per leg per period. The correction is
  in the bring-up log, and the parameter is still ASSUMED.
- **The link is a capacitor, an ESR and a source resistance**, and the converter
  samples its terminal voltage at the window centre like everything else. Bus
  ripple is therefore a result and not a formula, and the modulator sees
  whatever the link happens to be at that instant.
- **The sensing chain is opt-in in the HAL and on by default in the harness.**
  Tests that are about the controller keep it off so that they measure the
  controller. The distortion study turns it off too, and runs again with it on,
  because at 30 rad/s the encoder quantisation is a bigger distortion source
  than the inverter and hiding that would be a lie of omission.
- **Dead time compensation is applied after the dq magnitude limit, in the
  phase frame, with the common mode removed.** The correction on alpha is
  0.48 V for a 0.36 V per leg error, and the difference is the neutral shift. A
  compensator that added the three corrections without removing their mean
  would push its own neutral around and call it a fix.
- **Feedforward uses the 2 kHz velocity estimate, and the PI limits are reduced
  by the feedforward term.** The first keeps the implementation honest, because
  there is no faster speed signal, and it is the reason the load dip does not
  improve with feedforward. The second keeps anti-windup meaningful now that
  the feedforward spends part of the voltage budget before the PI gets it.
- **The motor's inductance and flux linkage are board config, not firmware
  constants.** A drive that decouples has to know its motor, and changing the
  motor should be a config change. The values come from the datasheet
  derivation in `docs/parameters.md` through the HAL, not from the plant.
- **Day 2 numbers are pinned to their commit.** The control path changed, so
  they are not reproduced at HEAD and the day 3 table says so, with the
  difference attributed rather than smoothed over.

## Day 3 status and what was cut

One motor, closed loop, driving a switching inverter with dead time, device
resistance, transition loss, a link that moves and a quantising sensor chain.
The firmware compensates dead time and feeds forward cross coupling and back
EMF. Cut or deferred, in the order they matter:

- No observer. The velocity estimate is still a ten tick encoder difference, so
  at 30 rad/s it is quantised to 2.4 percent of its own value and noisy to 5
  percent once the sensing chain is on. That, not the inverter, sets the
  distortion floor in the low speed result.
- No dead time asymmetry, which is what turns dead time into a DC current
  offset. No body diode forward drop either, so freewheeling is lossless and
  the energy the dead time removes shows up as a voltage error only.
- No overmodulation, no field weakening, no six step. The limit is still the
  circle at 95 percent of bus / sqrt(3).
- No thermal limit and no thermal model. The sampling probe runs 20 A on a
  clamped rotor, seven times continuous, and nothing objects.
- No anti-alias filter and no sampling jitter, so the converter sees the
  window centre exactly and the ripple aliases to nothing.
- No DC link protection: a back driven motor raises the bus in this model and
  nothing stops it, which matters for a haptic joint and not yet for this one.
- The transition loss parameter is ASSUMED with no device selected, so the
  efficiency number in `docs/results.md` is an estimate with a wide bar.

## Method notes

- Tests are written before the feature. The day 1 plant tests came before any
  firmware control code, which is why day 1 has no controller.
- Every physical sign convention lives in one place: amplitude invariant
  Clarke, Park with `iq = -i_alpha sin + i_beta cos`, torque
  `T = 1.5 p lambda iq`, back EMF `e_a = -E sin(theta_e)` with
  `E = p lambda omega_m`. The energy balance test is what will catch a mistake.
