# TANDEM-SIM

A switching level digital twin of a haptic robot joint, with field oriented
control firmware running inside it unmodified. No hardware, no cloud, one
machine.

**Status: day 3 of 7. One motor is closed loop in current and in speed, driving
a switching inverter with dead time, a DC link that moves and a quantising
sensing chain.** There is no load, no second joint and no thermal model yet, so
this is a single drive on a switching plant, not a twin. Every number in this
repository is simulated, and the figures say so on their face.

![Dead time distortion and compensation, simulated, commit 41433a1](docs/figures/day3_dead_time.png)

500 ns of gate dead time at 20 kHz makes 1.66 percent current distortion on a
1.96 A operating point, with a fifth harmonic at 30 mA. The firmware's
compensation brings that to 0.27 percent, which is within 0.06 of the same
drive with no dead time at all. The lower panel is the current with its
fundamental removed, so the notch and its absence are visible at the size they
actually are.

## What exists

| Path | What it is |
|---|---|
| `firmware/` | C11. Clarke, Park, space vector modulation, a PI current loop at 20 kHz, a PI speed loop at 2 kHz, d/q decoupling and back EMF feedforward, gate dead time compensation, boot offset calibration. Includes only its own headers, `hal.h` and three standard headers. Links no libm. |
| `firmware_portable` | Compile only target, `-ffreestanding -Werror -Wdouble-promotion`, 8271 bytes of `.text` and `.data` against a 16384 byte budget. |
| `sim/` | C++20. Two plants. `IdealPlant` is the average voltage source of day 1 and 2. `SwitchingPlant` resolves the carrier at 1 us, signs the dead time error by phase current, puts one device of on resistance in each leg, counts transition energy and integrates a 100 uF link with ESR behind a source resistance. A test asserts the second reduces to the first. |
| `harness/` | C++20. The only thing that owns time. One control window function shared by the harness and the closed loop tests. Three scenarios (`speed_step`, `current_step`, `dead_time`) with switches for plant, dead time, compensation and sensing chain, plus `tsim_sample_probe`, which holds a duty and records one switching period at 1 us. |
| `tests/` | In repo assert framework, 35 cases in 6 ctest entries: plant and convergence, inverter physics, transforms and firmware, closed loop and sensing, deterministic repeat, firmware boundary, size budget. |
| `tools/` | Boundary check, size check, determinism check, the day 2 and day 3 plots, a THD analyser, and an independent discrete model of the current loop. |
| `docs/` | `spec.md` (plan and decisions), `parameters.md` (every constant cited or marked ASSUMED), `results.md` (numbers with methods and commits), `bringup-log.md`, `model-limits.md`, `validation-plan.md`, `figures/`. |

## The numbers, briefly

| Quantity | Value | Source |
|---|---|---|
| dead time distortion, 500 ns, 1.96 A, 30 rad/s | 1.66 percent, 29.7 mA of fifth harmonic | `docs/results.md`, commit 41433a1 |
| same, with compensation | 0.27 percent, 1.7 mA | `docs/results.md` |
| same, sensing chain on | 2.09 percent, and 1.13 percent compensated | encoder floor, see results |
| sampling at the window centre, 20 A | 1.1 mA from the period mean | `docs/results.md` |
| sampling at a switching edge | 224 mA off, 1.1 percent of 20 A | `docs/results.md` |
| speed dip on a 50 mNm load step, switching plant | 15.8 percent, against 14.9 on the average plant | `docs/results.md` |
| transition loss at 1.96 A | 0.27 W from an assumed 50 ns crossover | `docs/results.md` |
| current loop, 10 to 90 percent rise, 2 A step | 222 us | `docs/results.md`, commit a781608 |
| plant integration error at 1 us | 1.8e-5 relative, halves with the step | `docs/results.md` |

None of these is a measurement. They are predictions from a model whose limits
are written down in `docs/model-limits.md`, and the speed numbers in particular
are a property of the bare rotor, not of the joint.

## Build

Three commands on a fresh clone:

```sh
cmake --preset dev
cmake --build --preset dev -j
ctest --preset dev --output-on-failure
```

Sanitizers:

```sh
cmake --preset asan
cmake --build --preset asan -j
ctest --preset asan --output-on-failure
```

Requirements: a C11 and C++20 compiler, CMake 3.22 or newer, Ninja, and Python
3 for the size check. The plotting tool additionally wants numpy and matplotlib
(`requirements.txt`).

To reproduce the day 3 figures:

```sh
./build/dev/harness/tsim_harness --scenario dead_time --dead-time-ns 0 --compensation off \
  --out data/run_latest/day3_dead_time_none.csv
./build/dev/harness/tsim_harness --scenario dead_time --compensation off
./build/dev/harness/tsim_harness --scenario dead_time --compensation on \
  --out data/run_latest/day3_dead_time_compensated.csv
./build/dev/harness/tsim_sample_probe
python3 tools/plot_day3.py --commit "$(git rev-parse --short HEAD)"
```

The harness defaults to the switching plant and to the sensing chain on, which
is the honest configuration. `--plant ideal --imperfections off` reproduces the
day 2 plant, though the day 2 control path is at commit a781608 and not at HEAD.

## Portability

`firmware/` may include its own headers, `hal.h` and three standard headers,
nothing else. A board adds `hal_stm32.c` and changes no firmware source: the
motor's inductance, its flux linkage and the gate dead time all arrive through
`hal_board_config`, because a drive that decouples and compensates has to know
them. The host simulator implements the same header.
`tools/check_firmware_boundary.sh` fails the build if a source file includes
anything outside that rule, or if a compiled object references a symbol outside
`hal_*` and the compiler runtime. The rule is checked, not promised, and it has
been proved to fail on purpose.

## Determinism

The harness reads no clock and schedules on integer nanoseconds, so two runs of
the same binary with the same flags produce byte identical output. That is a
test (`deterministic_repeat`), not a claim, and it covers the sensing chain's
noise too, because the generator is a seeded xorshift in the HAL and not
`rand()`. Cross compiler bit identity is a different and much stronger
statement, and it is not made.

## Honesty

Every constant in this repo is either cited in `docs/parameters.md` or marked
ASSUMED there. Every figure is labelled simulated and stamped with the commit
that produced it. No simulated number is presented as measured. Where the model
does not capture something, `docs/model-limits.md` says so, and each day ends
with a note on what was cut. One day 1 parameter was corrected on day 3 rather
than defended: the switching loss, which had been recorded in a form that could
not scale with current. The bring-up log has the entry.
