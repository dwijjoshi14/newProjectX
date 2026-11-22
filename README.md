# TANDEM-SIM

A switching level digital twin of a haptic robot joint, with field oriented
control firmware running inside it unmodified. No hardware, no cloud, one
machine.

**Status: day 1 of 7. This is scaffolding, the hardware contract and an ideal
plant. No controller is closed yet, so there is no demo and no number to
report.** Anything below is a description of what exists, not a result.

## What exists on day 1

| Path | What it is |
|---|---|
| `firmware/` | C11. FOC entry points, boot offset calibration, one control tick. Includes only `hal.h` and the C standard library. |
| `firmware_portable` | Compile only target built with `-ffreestanding -Wall -Wextra -Werror` against an in repo HAL stub. Proves the portability claim. |
| `sim/` | C++20. Ideal plant: phase resistance and inductance, back EMF, rotor inertia, viscous friction, stepped at 1 MHz with energy accounting. |
| `harness/` | C++20. Deterministic loop. Owns time, applies duty at the window boundary, latches the ADC sample at the window centre, calls the firmware tick. |
| `tests/` | In repo assert framework, five plant tests, two firmware tests, plus the boundary and size gates. |
| `tools/` | `check_firmware_boundary.sh`, `check_size.py`. |
| `docs/` | `spec.md` (plan and decisions), `parameters.md` (every constant, sourced or marked ASSUMED), `bringup-log.md`, `model-limits.md`, `validation-plan.md`, `results.md`. |

## Build

Two commands on a fresh clone, plus the test run:

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
3 for the size check. Ninja is the one non default tool; it is in every distro
and in the GitHub Ubuntu image.

## Portability

`firmware/src` may include `hal.h` and the C standard library, nothing else. A
board adds `hal_stm32.c` and changes no firmware source. The host simulator
implements the same header. This is enforced by
`tools/check_firmware_boundary.sh`, which fails the build if a source file
includes anything else or if a compiled object references a symbol outside
`hal_*` and the compiler runtime. The rule is checked, not promised.

## Honesty

Every constant in this repo is either cited in `docs/parameters.md` or marked
ASSUMED there. Every figure generated later is labelled simulated and stamped
with the commit that produced it. No simulated number is ever presented as
measured. Where a model does not capture something, `docs/model-limits.md`
says so.
