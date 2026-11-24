# Results

A spec sheet of simulated numbers, each with its method and the commit that
produced it. No measured number appears in this file, because nothing has been
measured. Every row can be regenerated from the commit beside it.

Status: day 3 of 7. One motor, closed loop in current and in speed, now driving
a switching inverter with dead time, a link that moves and a sensing chain in
the loop. The firmware compensates dead time and feeds forward the cross
coupling and back EMF.

Every row below the day 3 heading comes from commit 41433a1. The day 2 rows
further down are from commit a781608 and are kept as they were measured: the
control path changed on day 3, so rerunning those scenarios at HEAD gives
slightly different numbers, and the day 3 table says what changed.

## Dead time, and what compensation buys

![Dead time distortion and compensation, simulated, commit 41433a1](figures/day3_dead_time.png)

Simulated, commit 41433a1. Scenario `dead_time`: 29.9216 rad/s, which with
seven pole pairs is 33.333 Hz electrical and exactly ten cycles in the 300 ms
record, 90 mNm of load so the current is 1.96 A, 500 ns of gate dead time,
36 V bus, 20 kHz. The sensing chain is off for these three runs so the number
belongs to the inverter and not to the converter.

The measure is total harmonic distortion of the phase A current over the last
150 ms: Hann window, the power in a two bin band around the fundamental and
around harmonics 2 to 15, referred to the fundamental. Banding is what stops a
one percent error in the speed the loop actually holds from being reported as
distortion.

| Case | Fundamental | THD | 5th | 7th |
|---|---|---|---|---|
| no dead time | 1.959 A | 0.21 percent | 0.4 mA | 1.3 mA |
| 500 ns, no compensation | 1.959 A | 1.66 percent | 29.7 mA | 12.5 mA |
| 500 ns, compensated | 1.958 A | 0.27 percent | 1.7 mA | 1.8 mA |

The compensation takes the distortion to within 0.06 of a percentage point of
the no dead time case, and removes 94 percent of the fifth harmonic. That is
the whole claim of day 3, and it is a claim about this inverter model at this
operating point.

The same scenario with the sensing chain on, which is what a real joint would
have, gives 2.09 percent uncompensated and 1.13 percent compensated. The
compensation still halves the distortion, but the floor is now set by the
encoder, not the inverter: at 30 rad/s the velocity estimate is quantised to
2.4 percent of its own value and carries 2.3 counts of noise, so the speed loop
commands a current that moves by several percent. That is a sensor problem and
it is on the day 5 list, not something more compensation can fix.

Holding the speed is a second, blunter measure of the same thing:

| Case | Rotor speed at 29.9216 rad/s reference | Steady d axis current |
|---|---|---|
| no dead time | 29.887 rad/s, 0.12 percent low | -0.00002 A |
| 500 ns, no compensation | 29.721 rad/s, 0.67 percent low | -0.0396 A |
| 500 ns, compensated | 29.899 rad/s, 0.08 percent low | -0.0005 A |

The uncompensated error is not a gain error in the loop. The dead time error is
a function of the sign of the current, so it rectifies, and the loop cannot
integrate it away: the result is a d axis current of 0.04 A that produces no
torque and a speed that sits 0.9 rad/s below where it was asked to be.

The residual in the lower panel of the figure is the current with its
fundamental fitted and removed. The uncompensated trace is the classic dead
time shape, six humps per electrical cycle, plus or minus 40 mA on a 1.96 A
current. The compensated trace sits inside plus or minus 10 mA, and its texture
is the compensation switching on and off as each phase current crosses zero.

## Sampling instant

![Sampling instant, simulated, commit 41433a1](figures/day3_sampling_instant.png)

Simulated, commit 41433a1. `tsim_sample_probe`: the current loop settles 20 A
on a clamped rotor through the switching plant, the last duty is then held and
the plant is stepped at 1 us for exactly one switching period with phase A
recorded. Nothing runs the firmware during the capture, so nothing reacts to
the sample and the figure is a property of the plant and the modulator.

| Quantity | Value | Method |
|---|---|---|
| winding ripple | 447 mA peak to peak on 20.001 A | max minus min over one period |
| sample at the window centre | -1.1 mA from the period mean | value of the trace at 25 us |
| worst instant | -224 mA at 34 us | largest absolute deviation, 1.1 percent of 20 A |
| bus at the sample instant | 35.659 V, from 36 V | capacitor and ESR under a 20 A load |

The 20 A current is 7 times the continuous rating and a third of stall. It is
used because the ripple scales with the voltage the modulator applies, and at
the 2 A this joint normally carries the whole effect is 90 mA peak to peak,
which is true and invisible. There is no thermal model yet to object to a stall
test, which is a limit this file states rather than hides.

The point of the figure is the second panel: sampling anywhere other than the
centre of a symmetric segment reads the ripple, not the average. The two
unbiased instants in a centre aligned period are the middle of the active
vector, which is the window centre, and the middle of the zero vector, which is
the window boundary. The biased instants are the switching edges, and their
position moves with the duty ratio, so a converter triggered off the counter
underflow rather than the period centre would have to know where those edges
are. Sampling at the centre needs no such knowledge.

## What the switching inverter cost the day 2 loops

![Speed step on the switching plant, simulated, commit 41433a1](figures/day3_speed_step.png)

Simulated, commit 41433a1. Scenario `speed_step`, 200 rad/s reference and a
50 mNm load step at 150 ms, run three ways through the same firmware.

| Quantity | Ideal plant | Switching, no sensing | Switching, sensing on |
|---|---|---|---|
| 10 to 90 percent rise | 18.0 ms | 17.8 ms | 18.0 ms |
| peak speed | 200.11 rad/s | 200.09 rad/s | 200.96 rad/s |
| dip after the load step | 29.72 rad/s, 14.9 percent | 31.54 rad/s, 15.8 percent | 31.52 rad/s, 15.8 percent |
| back inside 2 percent | 172.6 ms | 172.3 ms | 173.1 ms |
| steady q current | 1.138 A | 1.122 A | 1.139 A |
| peak d axis current | 0.025 A | 0.012 A | 0.053 A |
| peak duty | 0.656 | 0.666 | 0.669 |
| lowest bus | 36.000 V | 35.976 V | 35.976 V |

Two honest readings of this table. First, the switching inverter costs about
1.8 rad/s of extra dip and 0.02 A of extra current on this step, which is 6
percent of a number that is already dominated by the inertia of a bare rotor.
Second, the sensing chain costs 0.9 rad/s of overshoot and 0.04 A of d axis
current. Neither is dramatic, and the reason is that a 20 kHz converter on a
0.256 mH winding is a small perturbation next to 2.3e-6 kg m^2 and 46 mNm/A.

The day 2 numbers are not reproducible at HEAD and that is expected: the
control path now has feedforward. At commit a781608 the ideal plant gave a
17.0 ms rise, a 200.17 rad/s peak and a 27.45 rad/s dip with 0.071 A of d axis
current. Rerunning that scenario at HEAD gives 18.0 ms, 200.11 rad/s and 29.72
rad/s with 0.025 A. So the d axis error during the transient improved by a
factor of 2.8 and the load dip got 2.3 rad/s worse. The feedforward is computed
from the 2 kHz velocity estimate, which lags the real speed by up to 0.5 ms,
and during a dip the back EMF is falling fast, so the feedforward is
momentarily wrong in the same direction as the disturbance. That is a real
trade, not a tuning accident, and it is why an observer sits on the day 6 list.

## Losses, and what the switching assumption is worth

The harness prints the plant's energy account, so the loss is measured rather
than quoted. Same `dead_time` scenario, 300 ms, 1.95 A, 30 rad/s:

| Account | No dead time | 500 ns dead time | 500 ns, compensated |
|---|---|---|---|
| electrical, from the link | 1.299 J | 1.689 J | 1.690 J |
| copper, in the windings | 0.507 J | 0.507 J | 0.507 J |
| device transitions | 0.081 J | 0.081 J | 0.081 J |
| mechanical, out of the shaft | 0.726 J | 0.726 J | 0.726 J |
| unaccounted, device drop and freewheel | -0.015 J | 0.375 J | 0.376 J |

The transition loss is 0.081 J in 0.3 s, which is 0.27 W at 1.95 A. That comes
from an ASSUMED 50 ns crossover, so it is an order of magnitude estimate with a
parameter that no device datasheet has confirmed.

The 0.38 J that only appears when the dead time is on is the dead time itself:
0.36 V per leg times the sum of the three phase currents, integrated over the
run, is 0.40 J. It is 1.3 W out of a 5.6 W input, and it is large because at 30
rad/s the machine voltage is only 1.7 V, so a 0.36 V error per leg is a fifth of
it. Compensation does not remove this loss. It restores the current waveform by
asking the modulator for more voltage, and the freewheeling current still
dissipates the difference in whichever body diode is conducting. A model that
compensated the distortion and also removed the loss would be wrong.

On the 200 rad/s speed step, where the machine voltage is 9.2 V, the same
inverter delivers 1.641 J of mechanical work from 1.777 J of link energy in
300 ms, which is 92 percent. Do not quote that as an efficiency: it is one
operating point, the transition loss is an assumed parameter, and there is no
thermal model to make the copper loss temperature dependent.

## Bus ripple

| Scenario | Bus range | Peak to peak |
|---|---|---|
| 200 rad/s, 1.14 A | 35.976 to 35.988 V | 24 mV |
| 30 rad/s, 1.96 A | 35.977 to 35.990 V | 13 mV |

Measured as the minimum and maximum of the terminal voltage across the run.
0.07 percent of the bus, against the 1 percent the day 1 parameter table
predicted for a 100 uF link. It is smaller because the DC current is the sum of
three phase legs, whose pulses overlap, rather than one leg's pulse. The
converter samples the bus at the window centre and the firmware uses that
number for the modulator, so a 24 mV ripple is a 0.07 percent modulation error,
which is why it does not show up anywhere else in these results.

## Day 2 results, at commit a781608

Kept as measured. See the table above for what the same scenarios give now.

### Current loop, clamped rotor

![Current loop step response, simulated, commit a781608](figures/day2_current_step.png)

| Quantity | Value | Method |
|---|---|---|
| 10 to 90 percent rise, first step | 222 us | deterministic harness, interpolated between 50 us samples |
| steady state at 1.5 to 2.0 ms | 1.9979 A | mean of the last ten samples before the next step |
| discrete reference model, same loop | 222 us, 1.9981 A | `tools/reference_current_loop.py` |
| continuous first order estimate | 350 us | one pole at Kp / L = 6290 rad/s, times 2.2 |

The harness and the reference model are independent implementations of the same
discrete loop and agree to 0.1 percent on the rise. The continuous estimate is
58 percent away, because it models neither the sample and hold nor the
rectangular integrator.

### Speed loop and load rejection

![Speed step and load rejection, simulated, commit a781608](figures/day2_speed_step.png)

| Quantity | Value | Method |
|---|---|---|
| 10 to 90 percent rise, 0 to 200 rad/s | 17.0 ms | harness, `speed_step` |
| peak speed | 200.17 rad/s | 0.09 percent above the reference |
| dip after the 50 mNm step | 27.45 rad/s, 13.7 percent | minimum speed in the 20 ms after the step |
| time back inside 2 percent | 22 ms | last sample outside the band at 172.1 ms |
| steady q current after the load | 1.123 A | analytic 0.05 / 0.0461 + 0.04 viscous = 1.13 A |
| peak d axis current | 0.071 A | no decoupling feedforward on day 2 |
| peak duty | 0.673 | 35 percent of the linear range |

## Plant integration error

Forward Euler at three step sizes against the exact solution of the same RL
step, commit 40ceb96. 3.6 V across 0.291 ohm and 0.256 mH, 2 ms.

| Plant step | Current after 2 ms | Relative error against exact |
|---|---|---|
| 1 us, the step the harness uses | 11.097629 A | 1.8e-5 |
| 0.5 us | 11.097530 A | 8.9e-6 |
| 0.25 us | 11.097481 A | 4.5e-6 |
| exact | 11.097431 A | |

## Reproduce

```sh
cmake --preset dev
cmake --build --preset dev -j

# the day 3 distortion study: three runs of the same scenario
./build/dev/harness/tsim_harness --scenario dead_time --dead-time-ns 0 --compensation off \
  --out data/run_latest/day3_dead_time_none.csv
./build/dev/harness/tsim_harness --scenario dead_time --compensation off
./build/dev/harness/tsim_harness --scenario dead_time --compensation on \
  --out data/run_latest/day3_dead_time_compensated.csv
./build/dev/harness/tsim_sample_probe
python3 tools/plot_day3.py --commit "$(git rev-parse --short HEAD)"

# the switching cost table and the figure that goes with it
./build/dev/harness/tsim_harness --scenario speed_step --plant ideal --imperfections off \
  --out data/run_latest/day3_speed_ideal.csv
./build/dev/harness/tsim_harness --scenario speed_step --plant switching --imperfections off \
  --out data/run_latest/day3_speed_switching_nosense.csv
./build/dev/harness/tsim_harness --scenario speed_step --out data/run_latest/day3_speed_step_switching.csv

# day 2 reproduction needs the ideal plant and the sensing chain off
./build/dev/harness/tsim_harness --scenario current_step --plant ideal --imperfections off
./build/dev/harness/tsim_harness --scenario speed_step --plant ideal --imperfections off
python3 tools/plot_day2.py --commit a781608
```

The harness writes `data/run_latest/`, which is ignored by git, and a figure
stamp is the commit that contains the code that produced it.
