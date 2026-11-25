# MEMORY

Durable decisions and corrections. Newest last.

- 2026-11-22 project start. Seven working sessions agreed with Dwij, one per day. Day scope in `docs/spec.md`.
- 2026-11-22 motor is Maxon EC-i 40, 70 W, 36 V, part 496655. Rejected the EC 45 flat 50 W (251601 and 651611) because both are marked NRND on maxon's site today.
- 2026-11-22 plant uses amplitude invariant Clarke and Park with torque `T = 1.5 p lambda iq` and back EMF `e_a = -E sin(theta_e)`. Written down because sign convention errors are the top rung 1 risk.
- 2026-11-22 determinism claim is byte identical for repeated runs of the same compiler and flags on the same architecture. Cross compiler bit identity is not claimed.
- 2026-11-22 sandbox has no root and no apt. cmake, ninja, clang-format,
  clang-tidy and cppcheck all live in `.venv`. clang-tidy is pinned to 18.1.8
  because that is the Ubuntu runner version, and a local clean run only means
  something if it uses the same LLVM release. clang-format moved to 21.1.6 on
  2026-11-25 for the same reason, see the day 4 entry.
- 2026-11-23 day 2. Current loop is a PI at 20 kHz with Kp = 1.61 V/A, which is
  a 1 kHz crossover on 0.256 mH, and Ki on the winding pole at 1137 rad/s.
  Speed loop is 100 Hz behind a 10 ms command filter, because the ten tick
  velocity estimate costs about 0.6 ms of lag and the delay eats 30 degrees of
  the formula's margin. Both loops validated against a discrete reference model
  (`tools/reference_current_loop.py`), not against a continuous formula: the
  reference model agrees with the harness to 0.1 percent, 222 us both ways, while
  the continuous estimate for the same rise is 350 us. The continuous formula is
  not a tolerance for a sampled loop at a 1 kHz crossover; the model is.
- 2026-11-23 the bare rotor makes a 50 mNm load step violent, because T / J is
  21700 rad/s^2 and the delay alone allows a 13 rad/s dip. The measured 13.7
  percent dip is inertia, not tuning. Do not quote it as a joint property until
  day 4 puts the handle on.
- 2026-11-23 cppcheck 2.22.0 no longer resolves from this sandbox's pip index,
  so static analysis is CI only again and the day 1 claim about a local
  cppcheck run does not hold today. clang-tidy 18.1.8 still runs locally.
- 2026-11-24 day 3. The inverter switches: separate `SwitchingPlant`, carrier
  resolved at the plant step with the on time of each step computed as a
  fraction so a 1 us grid does not quantise the duty to 2 percent, which is
  0.72 V and twice the dead time error being studied. Dead time is the standard
  per leg average voltage error, signed by phase current and faded below 0.5 A.
- 2026-11-24 the switching loss parameter from day 1 was wrong in form, not in
  value: 20 uJ per device transition does not scale with current and implied
  4.8 W at 2 A. It is now a 50 ns crossover with the current in the energy, and
  the same run loses 0.027 J instead of 1.44 J. Still ASSUMED, still unverified.
- 2026-11-24 dead time compensation: after the limit, in the phase frame, with
  the mean of the three corrections removed. That makes the alpha correction
  0.48 V for a 0.36 V per leg error, and the extra 0.12 V is the neutral shift,
  not a fudge.
- 2026-11-24 feedforward is not free. Decoupling plus back EMF took the peak d
  axis error in a 200 rad/s step from 0.071 A to 0.025 A on the ideal plant and
  made the load dip 2.3 rad/s worse, because it is computed from the 2 kHz
  velocity estimate, which lags by up to 0.5 ms. Do not present feedforward as
  an unqualified improvement; the fix for the trade is an observer.
- 2026-11-24 distortion measurements at low speed have a floor set by the
  encoder, not the inverter: at 30 rad/s one count is 2.4 percent of the speed
  and the encoder noise is 2.3 counts, so with the sensing chain on the THD is
  2.09 percent against 1.66 percent with it off. Both numbers are published and
  the second one is labelled an encoder floor.
- 2026-11-24 boot offset calibration needed the latch to survive hal_init and
  the harness to publish one conversion before boot, because a board's ADC is
  free running before the bridge is enabled. Without it the drive ran with a
  1.5 mV amplifier offset, which is 0.06 A, and it showed up as 0.047 A of d
  axis current.
- 2026-11-25 the ci format gate is clang-format 21.1.6, because ubuntu-latest
  moved to 26.04 and apt now installs 21.1.6 there. The sandbox venv pins the
  same version, since 18 and 23 each disagree with 21 about how to lay out one
  aggregate initialiser in `stub_hal.c` and formatting with either of them turns
  the gate red. Pin the formatter to the runner's, not the newest.
- 2026-11-25 day 4. Mechanics is one `MechanicalSystem` shared by both plants,
  because two plants disagreeing about inertia is the bug this whole project is
  built to avoid. The rigid configuration reproduces the day 1 body bit for bit
  and a test asserts it, so the refactor is checked rather than trusted.
- 2026-11-25 the assumed 0.9 N m/rad coupling was the bug of the day. A 6 mm
  steel shaft 40 mm long is 250, its anti resonance with the handle is 205 Hz,
  and the first version of the day closed the position loop at 12.7 Hz on top of
  a 12.3 Hz anti resonance with |T| = -12.8 dB at 2 Hz and the current railed.
  Assume a stiffness, cap the loop, and the sweep says so immediately.
- 2026-11-25 the drive now commissions its own crossover from two limits: a
  quarter of the anti resonance, and the crossover where velocity noise alone
  would command a tenth of continuous current (J wc sigma_omega / Kt). Both are
  exactly 1.0 for a bare rotor, so day 1 to day 3 numbers do not move. On the
  hand the sensor limit binds by a factor of 9.6: 33.6 rad/s from 0.05 deg of
  encoder noise against 322.7 from the mechanics.
- 2026-11-25 the position loop is sensor limited, not mechanics limited. 0.81 Hz
  with 0.05 deg of encoder noise and 6.62 Hz with 0.01 deg, same mechanics, same
  gains rule. The velocity estimate is a ten tick difference, so 2.5 rad/s of
  noise at 0.05 deg, and the gain the handle's inertia needs turns that into the
  continuous current. An observer is the fix and it is not in the seven days.
- 2026-11-25 there is a 1 degree dead band at the handle and no gain removes it:
  8 mN m of shaft friction over a 0.45 N m/rad hand is 17.8 mrad. The joint
  hunts inside the band instead of settling, which is why the step test asserts
  two bands and why the results say hunt rather than settle.
- 2026-11-25 no torque sensor means the rendered stiffness is 63 percent of the
  command at 1 N m/rad and 30 percent at 4, and the series arithmetic with the
  250 N m/rad shaft and the 1.8 N m/rad wall predicts both to better than 2
  percent. Publish the ratio, not the command.
