# Bring-up log

Written as if this were hardware: dated, one entry per bug or decision that
cost time. This log is the raw material for the README and for interviews.

## 2026-11-22, day 1

**Entry 1, no root in the build environment.** `apt-get` and `sudo` are both
unavailable in this sandbox, and the system Python venv is read only. Fix:
create the project `.venv` and install `cmake`, `ninja` and `clang-format`
there. Consequence: the documented commands assume `cmake` is on PATH, so the
README says so, and local runs here prefix `PATH` with `.venv/bin`.

**Entry 2, cppcheck not available.** No root means no package install. cppcheck
runs in CI on Ubuntu only. Recorded as a known gap, not as a pass.

**Entry 3, motor selection changed on the day.** First choice was maxon EC 45
flat 50 W 651611. The product page carries an NRND flag, and so does 251601.
Switched to EC-i 40 70 W 496655. The parameter table was rebuilt from the
retrieved datasheet rather than edited, and the two consistency cross checks
(mechanical time constant, Kt against Ke) were run before accepting it.

**Entry 4, sign convention before any control code.** Back EMF, q axis and
torque conventions were written down first because a sign error here produces a
loop that is stable in the one test you happened to run. The plant test asserts
that an alpha aligned voltage at zero electrical angle gives no torque, which is
only true for one consistent set of conventions.

**Entry 5, ripple discovered from the real winding.** 0.256 mH at 20 kHz gives
1.76 A peak to peak at 50 percent duty, 64 percent of continuous current. This
is a property of the motor, not of the model. Recorded in parameters.md before
the switching rung exists so that the later result cannot be presented as a
surprise, or as someone else's fault.

**Entry 6, the size gate failed its own first sanitizer run.** Under ASan the
portable target measured 16645 bytes against a 16384 byte budget, while the
debug build measured 2706. Cause is sanitizer instrumentation, not firmware
growth. Fix: `TSIM_SANITIZERS` cache variable, and the budget test is only
registered for builds that could ship. The gate now fails for the right reason
and stays silent for the wrong one.

**Entry 7, clang-tidy flagged two findings, both the same check.**
`bugprone-easily-swappable-parameters` fired on `hal_panic(code, line)` and on
`clarke_beta(a, b, c)`. These are same-type parameters by design. Rather than
paper over them with NOLINT lines, the check is disabled in `.clang-tidy` with
the reason written next to it, and the sign convention tests remain the real
protection.

**Entry 8, first CI run failed on two gates, both in the pipeline rather than
the code.** The size step pointed at `build/dev/firmware_portable`, which does
not exist; CMake puts it in `build/dev/firmware/`. Fixed by finding the binary
instead of hard coding the path. The clang-tidy step exited 123 (xargs
propagating a non-zero tool exit) with the diagnostic buried in a log the run
page does not show, so the step now prints every finding as an annotation. A
gate that cannot say why it failed is not a gate, it is a coin flip.

**Entry 9, CI clang-tidy is not the local clang-tidy.** CI runs 18, this
sandbox has 23, and 18 reports `performance-enum-size` on `hal_status_t`. The
suggested fix is a fixed enum underlying type, which C11 cannot express (it is
C23 or C++). Disabled in `.clang-tidy` with that reason next to it, and the
version drift is now written down: the CI compiler of record is the one that
decides, and a local clean run is not evidence about CI.

**Entry 10, correction to entry 2.** cppcheck is available without root after
all, through the `cppcheck` pip wheel (version 2.22.0), installed into `.venv`.
Entry 2 was true when written and is superseded, not deleted. Running it on
firmware, sim and harness with `--enable=warning,style,performance
--error-exitcode=1` reports nothing, so the local run now matches what CI is
asked to do.

**Entry 11, the disable did not disable anything.** The `#` reason was written
inside the folded `Checks:` scalar, so YAML folded it into the check list as
literal text and the following `-performance-enum-size` was lost. Moved the
reasons above the key. Also installed clang-tidy 18.1.8 from the pip wheel,
which is the same LLVM release as the Ubuntu runner, so a local run now says
something about CI instead of approximating it. Verified: tidy 18 exits 0 on
all five translation units.

## 2026-11-23, day 2

**Entry 12, cppcheck pin no longer resolves.** The same `cppcheck==2.22.0` pin
that installed on day 1 is refused by this index today: only versions up to
1.5.3 are offered. Rather than install an ancient cppcheck that would report
different findings and call it a pass, static analysis is CI only again until
the pin resolves. The rest of the toolchain reinstalled from the same pins, and
clang-tidy 18.1.8 still runs here, so the tidy claim stands and the cppcheck
claim does not.

**Entry 13, the sine approximation had the wrong sign on its last term.** The
eleventh order coefficient was entered as plus 1/11! instead of minus, so the
error at the peak of the sine was 7e-6 instead of 1.4e-7. The transform round
trip test failed at its 1e-5 tolerance, which is the only reason it was found
before a loop ran on it. One character, and a reminder that a polynomial is a
place where a sign is as easy to get wrong as in a frame transform.

**Entry 14, the first speed loop could not hold its own rotor.** Designed at
12 Hz from the standard PI on an inertia formula, it looked fine on paper. A
50 mNm load step dropped the shaft from 200 rad/s to 39 before it reacted. The
disturbance rate is T / J = 21700 rad/s^2, so the loop's 13 ms reaction time was
most of the available speed. Redesigned at 100 Hz. No amount of reading the
datasheet table shows you this; the trace does.

**Entry 15, then it rang.** At 80 Hz with a slew limited command the step still
overshot 17.6 percent, because the ten tick velocity estimate adds about 0.6 ms
of lag, and at that crossover 0.6 ms is 30 degrees of the margin the formula
promised. The fix was not more damping in the PI and it was not a lower gain,
which would have made the load dip worse. It was a 10 ms first order filter on
the command, which removes the excitation and spends nothing from the voltage
budget. Overshoot after the change, 0.09 percent. Dip unchanged at 13.7 percent.

**Entry 16, a square root that crawls.** `foc_sqrt` ran eight Newton steps from
max(x, 1). For an argument of 1e-6 that is ten halvings short of the
neighbourhood of the root, so the answer was 0.0039 instead of 0.001. The range
test caught it before any loop used it. Fixed by normalising into [1, 4) with
exact power of two steps and then running six Newton steps, which is also a
fixed cost, and a fixed cost is what a 50 us budget wants.

**Entry 17, the HAL applied duties until it found a bad one.**
`hal_pwm_set_duty` validated and wrote in the same loop, so a vector whose third
phase was out of range left two phases on the new value and one on the old. No
timer can be in that state. Found while adding the space vector modulation,
whose whole job is to keep the vector inside the range. Fixed by validating all
three before writing any.

**Entry 18, tests shared an offset through a stale latch.** The boot current
offset calibration averages the latched sample. In a test executable, the second
test to run inherited the first test's last current as its offset, so the speed
test failed only when it ran after the saturation test and passed on its own.
That kind of order dependence is worse than a broken test, because it teaches
you to rerun until it passes. Fixed in the harness: attaching a plant clears the
latch, and init reports no valid sample, so calibration on the host is the
documented no op. A board free runs its ADC and this path returns real samples.

**Entry 19, the determinism gate caught the gate.** The new repeat run test
compares two runs byte for byte and failed on the first try because the console
summary prints the output path, which differs by construction. Normalised that
one line out. Kept in the log on purpose: a gate that fails for a reason
unrelated to what it checks is a gate people learn to ignore.

**Entry 20, clang-tidy on the control code.** `bugprone-macro-parentheses`
fired on the negative sine coefficients. Fixed by parenthesising the constants,
which is what the check asks for, rather than disabling the check. Everything
else in the new code passed 18.1.8 with warnings as errors on the firmware, sim
and harness translation units.

## Template for later entries

```
## DATE, day N
**Entry N, one line title.** What was observed, what the cause was, what was
changed, what the check now proves.
```
