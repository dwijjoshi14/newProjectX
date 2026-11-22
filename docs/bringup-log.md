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

## Template for later entries

```
## DATE, day N
**Entry N, one line title.** What was observed, what the cause was, what was
changed, what the check now proves.
```
