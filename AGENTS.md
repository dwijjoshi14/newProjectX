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
