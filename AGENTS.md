# TANDEM-SIM

A switching-level digital twin of a haptic robot joint, with production-style FOC firmware running inside it unmodified. Zero hardware, zero purchases, everything runs locally.

Owner: Dwij Joshi, Mechatronics Engineering, TMU, graduating April 2029.

## Stack
- firmware/  C11, CMake. STM32G4 target must always compile. Never flashed.
- sim/       C++20, CMake. Fixed 1 MHz plant step. No third party physics engine.
- harness/   C++20. Deterministic mode and real-time thread mode.
- tools/     Python 3, numpy, matplotlib.
- twin/      Vite, TypeScript, Three.js.
- docs/      parameters, model limits, validation plan, bringup log.

## Hard rules
1. Firmware never includes a simulator header. All hardware access goes through hal.h. The simulator implements hal.h on the host.
2. Every plant parameter has a cited source in docs/parameters.md. Anything unavailable is marked assumed, in writing.
3. Every figure is labelled simulated and stamped with the commit hash. No simulated number is ever presented as measured.
4. Write the host test before the feature.
5. No em-dashes anywhere: code, comments, docs, commits, chat.
6. Verify datasheet values and API signatures with a search before using them. Never guess a parameter.
7. Log every bug and fix in docs/bringup-log.md with a date, as if this were hardware.
8. Keep MEMORY.md current with decisions and corrections.

## Account skills in use
core-operating-rules, critic, deep-research, tooling-connections (design-taste and design-audit for the twin and README presentation only).

## Current phase
Phase 1, scaffolding and rung 1 of the fidelity ladder. See docs/spec.md.

## Day 1 decisions

- Build window is seven working sessions, one per day. Each session ends with a
  commit series and a one paragraph status note that also lists what was cut.
- The simulator holds no physical constant that is not in docs/parameters.md.
  Every row is cited or marked ASSUMED, with no third category.
- The firmware boundary is enforced by tools/check_firmware_boundary.sh inside
  ctest, not by review.
- Determinism claim: byte identical for repeated runs of the same compiler and
  flags on the same architecture. Cross compiler bit identity is not claimed.
- Commit dates are real. No backdating to imply a longer build.
- This sandbox has no root and no apt. cmake, ninja, clang-format and clang-tidy
  live in .venv. cppcheck runs in CI only, and that gap is stated, never hidden.
- Physical sign conventions live in docs/spec.md. Change them only with a test
  that would fail under the old convention.
