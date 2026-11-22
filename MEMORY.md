# MEMORY

Durable decisions and corrections. Newest last.

- 2026-11-22 project start. Seven working sessions agreed with Dwij, one per day. Day scope in `docs/spec.md`.
- 2026-11-22 motor is Maxon EC-i 40, 70 W, 36 V, part 496655. Rejected the EC 45 flat 50 W (251601 and 651611) because both are marked NRND on maxon's site today.
- 2026-11-22 plant uses amplitude invariant Clarke and Park with torque `T = 1.5 p lambda iq` and back EMF `e_a = -E sin(theta_e)`. Written down because sign convention errors are the top rung 1 risk.
- 2026-11-22 determinism claim is byte identical for repeated runs of the same compiler and flags on the same architecture. Cross compiler bit identity is not claimed.
- 2026-11-22 sandbox has no root and no apt. cmake, ninja, clang-format,
  clang-tidy 18.1.8 and cppcheck 2.22.0 all live in `.venv`. clang-tidy is
  pinned to 18.1.8 because that is the Ubuntu runner version, and a local clean
  run only means something if it uses the same LLVM release.
