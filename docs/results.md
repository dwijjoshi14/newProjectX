# Results

A spec sheet of simulated numbers, each with its method and the commit that
produced it. No measured number appears in this file, because nothing has been
measured. Every row can be regenerated from the commit beside it.

Status: day 2 of 7. One motor is closed loop in current and in speed. The two
figures below are the first traces this project has produced.

## Current loop, clamped rotor

![Current loop step response, simulated, commit a781608](figures/day2_current_step.png)

Simulated, commit a781608. Scenario `current_step`: the shaft is clamped, the q
axis is parked on phase A, and the current reference steps 0 to 2 A, to -2 A, to
0.

| Quantity | Value | Method |
|---|---|---|
| 10 to 90 percent rise, first step | 222 us | deterministic harness, interpolated between 50 us samples |
| steady state at 1.5 to 2.0 ms | 1.9979 A | mean of the last ten samples before the next step |
| peak q current | 1.9986 A | no overshoot, the approach is monotone |
| d axis current during the step | below 1e-6 A | the rotor is clamped, so there is no cross coupling to excite |
| discrete reference model, same loop | 222 us, 1.9981 A | `tools/reference_current_loop.py` |
| continuous first order estimate | 350 us | one pole at Kp / L = 6290 rad/s, that is 159 us, times 2.2 |

The harness and the reference model are independent implementations of the same
discrete loop, and they agree to 0.1 percent on the rise and 0.01 percent on
the steady state. That is the validation being claimed here: the harness timing
is the designed timing, down to the 25 us sample offset and the one window
actuation delay. It says nothing about whether the plant is right, which is a
different question and a harder one.

The continuous estimate is 58 percent away from both. It models neither the
sample and hold nor the rectangular integrator, and at a 1 kHz crossover those
are not small corrections. That is why the test tolerance is a band and why the
reference model is committed as a tool.

## Speed loop and load rejection

![Speed step and load rejection, simulated, commit a781608](figures/day2_speed_step.png)

Simulated, commit a781608. Scenario `speed_step`: 200 rad/s reference, then
50 mNm applied at 150 ms. The q current trace is sampled at the current loop
rate, so its 2 kHz staircase is the speed loop's update period and not noise.

| Quantity | Value | Method |
|---|---|---|
| 10 to 90 percent rise, 0 to 200 rad/s | 17.0 ms | harness, `speed_step` |
| peak speed | 200.17 rad/s | 0.09 percent above the reference |
| steady state error at 150 ms | 1.0 rad/s | 0.5 percent |
| dip after the 50 mNm step | 27.45 rad/s, 13.7 percent | minimum speed in the 20 ms after the step |
| time back inside 2 percent after the step | 22 ms | last sample outside the band at 172.1 ms |
| steady q current after the load | 1.123 A | analytic 0.05 / 0.0461 + 0.04 viscous = 1.13 A, 0.6 percent |
| peak q current | 1.232 A | 45 percent of the 2.74 A continuous rating |
| peak d axis current | 0.071 A | no decoupling feedforward, so the PI rejects the cross coupling term |
| peak duty | 0.673 | from the 0.5 zero vector, so the trajectory uses 35 percent of the linear range |

Two notes that belong with the numbers rather than with the prose.

The rise time is set by the 10 ms command filter, not by the loop. The loop
alone crosses over near 100 Hz and would follow a step in about 3 ms with an
18 percent overshoot; that version was measured during the day and is in the
bring-up log, not in the repository. The filter trades the overshoot for 17 ms
of rise and costs nothing in steady state accuracy.

The dip is a property of the inertia, not of the tuning. A 50 mNm step on a
2.3e-6 kg m^2 rotor accelerates it at 21700 rad/s^2, so the speed falls until
the loop can answer, and the pipeline delay alone allows 0.05 N m * 0.6 ms / J =
13 rad/s. Adding a handle and a gearbox on day 4 raises the inertia by two
orders of magnitude and makes this loop look easy. That is what the ladder is
for.

## Plant integration error

Forward Euler at three step sizes against the exact solution of the same RL
step, commit 40ceb96. Scenario: 3.6 V across 0.291 ohm and 0.256 mH, 2 ms.

| Plant step | Current after 2 ms | Relative error against exact |
|---|---|---|
| 1 us, the step the harness uses | 11.097629 A | 1.8e-5 |
| 0.5 us | 11.097530 A | 8.9e-6 |
| 0.25 us | 11.097481 A | 4.5e-6 |
| exact | 11.097431 A | |

Halving the step halves the error, which is what a first order integrator has to
do, and at the step this simulator actually runs the error is 0.0018 percent.
`docs/model-limits.md` said this was unquantified on day 1. It is a test now.

## Reproduce

```sh
cmake --preset dev
cmake --build --preset dev -j
./build/dev/harness/tsim_harness --scenario current_step
./build/dev/harness/tsim_harness --scenario speed_step
python3 tools/plot_day2.py --commit "$(git rev-parse --short HEAD)"
python3 tools/reference_current_loop.py
```

The harness writes `data/run_latest/`, which is ignored by git, and the figure
stamp is the commit that contains the code that produced it.
