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

## 2026-11-24, day 3

**Entry 21, the modulator was coarser than the fault it was measuring.** The
first distortion run reported about 6 percent THD in every case, and the dead
time compensation made it slightly worse. The cause was not the compensation:
a 1 us plant step cannot place a PWM edge more finely than 1 us, which over a
50 us period is 2 percent of the duty, and at 36 V that is 0.72 V. The dead
time error being studied is 0.36 V. The model was measuring its own grid. Fixed
by computing the fraction of each step a leg is high instead of snapping the
edge to the step. The same run then gave 1.2 percent uncompensated and 0.3
percent compensated, and the fifth harmonic fell by 94 percent. Nothing about
the firmware changed, which is the point: the instrument was wrong, not the
thing being instrumented.

**Entry 22, the harness logged the wrong current.** `ia_a` in the CSV was the
plant's current at the end of the window, not the sample the converter latched
at the centre. At 20 kHz those differ by half the ripple, so a distortion figure
drawn from that column would report switching ripple as controller behaviour.
The harness now logs the measured current and the plant current side by side,
and the distortion analysis uses the measured one because that is what the
firmware acted on.

**Entry 23, the switching loss assumption could not be right.** Day 1 recorded
20 uJ per device transition. The day 3 energy account showed the inverter
drawing 3.19 J from the link to deliver 1.64 J of mechanical work, with 1.44 J
in transitions: 4.8 W of switching loss at a 1.1 A phase current. A fixed
energy per event implies a 550 ns crossover at 2 A, which is not a device, and
it also cannot be right at 20 A because it does not scale with current. Replaced
with the overlap triangle and a 50 ns crossover, which is the same model with
the current in it. The same run now loses 0.027 J to transitions. This is a
correction to the parameter table, not a tuning change, and the parameter is
still assumed.

**Entry 24, the dead time correction is not 0.36 V.** The obvious implementation
adds bus times dead time over period to each phase, which is 0.36 V. Run through
the transform it moves alpha by 0.48 V, because three corrections that do not
sum to zero move the neutral, and the winding only sees the differential part.
The first version of the test asserted 0.36 and failed. The function now removes
the mean of the three corrections before transforming back, and the test asserts
0.48 with the arithmetic in the comment.

**Entry 25, the reducing case test found a float sum.** The new switching plant
must reproduce the average plant exactly when its inverter is ideal, and it did
not: 1.9e-7 A on a 9 A current. The cause is that the average plant adds three
float duty ratios in float and the switching plant adds three double voltages.
That is 1e-8 relative in the neutral offset and harmless, so it is documented
in the test rather than papered over by changing day 2 results in their last
bits. A test that fails by a nanoamp and says why is worth more than one that
passes by luck.

**Entry 26, the drive booted with an uncalibrated shunt.** The firmware averages
the latched sample at boot to find its current offsets, and `hal_init` was
clearing the latch, so on the host that loop never saw a sample and the offsets
stayed at zero. With the sensing chain on, that is a 1.5 mV amplifier offset,
which is 0.06 A of current error the loop then has to live with, and it showed
up as 0.047 A of steady d axis current. On a board the ADC is free running
before the bridge is enabled. Fixed by leaving the latch alone in `hal_init`
(attaching a plant still clears it, so a fresh run starts empty) and by having
the harness publish the first conversion, which is the true zero current,
before boot. The peak d axis error at 200 rad/s on the ideal plant fell from
0.057 A to 0.025 A.

**Entry 27, anti-windup had to learn about feedforward.** The current PI clips
its integral at the voltage limit. Once the feedforward spends 6.3 V of a 19.7 V
budget at 200 rad/s, a PI limited to the full 19.7 V winds up around voltage it
never gets to apply, and unwinds slowly after the transient. The PI limit is now
the remaining budget, and it is floored at zero rather than going negative when
the feedforward alone exceeds the limit.

**Entry 28, a 30 rad/s run would not hold 30 rad/s.** With the sensing chain on,
the dead time scenario settled at 30.6 rad/s against a 29.92 reference. The
cause is the velocity estimate: at 30 rad/s the rotor moves 42 encoder counts
per 0.5 ms window, so one count is 2.4 percent of the speed, and the 2.3 count
encoder noise makes the difference over ten ticks about 5 percent. The loop is
chasing a number that moves by more than the error it is correcting. Rather
than tune around it, the distortion result is quoted with the chain off and
again with it on, and the second number is labelled as an encoder floor. An
observer is the fix and it is not on the day 3 list.

## 2026-11-25, day 4

**Entry 29, the stiction term destroyed momentum.** The first mechanical system
zeroed the relative velocity of the two bodies whenever the joint was inside
the stiction band. That is what stiction does, but it has to be an impulse
bounded by the friction cone, mu N dt, and not a discontinuous zeroing: the
first version deleted the momentum of a joint coasting at 4 rad/s in a single
step, and the energy test caught it as 2.4 mJ appearing out of nothing. The
friction impulse is now clamped to mu N dt and the account has a column for it.
Without that test the joint would have stopped dead on every reversal and
every static result would have been wrong in the same direction.

**Entry 30, the energy account was short by the spring.** The day 1 account
balanced to 3 percent with the two mass path on, and the missing term was
0.5 k theta_rel^2 stored in the coupling. Added as `elastic_j`. The residual is
now 1e-9 relative on a 1 s run, which is float noise and not a physics gap.

**Entry 31, a 0.9 N m/rad coupling cannot hold a handle.** The first sweep used
the stiffness the parameter table carried, 0.9 N m/rad, which is a rubber
coupling and not a shaft. With the hand on it, the anti resonance is at 12.3 Hz
and the position loop was closing at 12.7 Hz, so the outer loop was asking the
inner one to work on top of the resonance it was supposed to stay below. The
result was not marginal: |T| was -12.8 dB at 2 Hz, iq_ref sat on the 2.74 A
limit, the load swung +-20 rad/s and the joint stopped 27 mrad short of a 0.1
rad step. Replaced the assumption with arithmetic, a 6 mm steel shaft 40 mm
between clamp faces is 250 N m/rad, which puts the anti resonance at 205 Hz,
and made the drive read the stiffness from the board config and commission its
own crossover at a quarter of that. The assumed number was the bug, not the
loop: it had been in the table since day 1 with no arithmetic behind it.

**Entry 32, then it railed on its own encoder.** With 250 N m/rad the drive
commissioned to 51 Hz and the sweep was still bad: -5.2 dB at 2 Hz, iq on the
limit again, and the trace showed the velocity estimate swinging between -8 and
+8 rad/s while the joint moved at 1.3. The 0.05 deg encoder noise in the table
is 2.3 counts, and a ten tick difference turns that into 2.5 rad/s. The speed
gain the handle's inertia needs, J wc / Kt, turns 2.5 rad/s into 2.7 A, which is
the continuous current. The loop was not badly tuned, it was driving the bridge
with its sensor noise. Fixed by making the commissioning read the encoder: the
noise current is J wc sigma_omega / Kt, holding it to a tenth of continuous
gives 33.6 rad/s for this board, and the position loop follows it down to 8.4
per second. The same joint with 0.01 deg of encoder noise commissions to 168
rad/s and closes at 6.6 Hz, which is the whole point of the day 4 result.

**Entry 33, the friction band is a degree wide and the test wanted a point.**
The step test asserted the joint lands within one friction band of the
reference, 8 mN m over 0.45 N m/rad, which is 17.8 mrad. It lands at 19.9 mrad
and holds there, and the trace shows why: the loop hunts inside the band rather
than settling, because the spring force only has to be within the friction cone
for the joint to stand still. Widening the assertion to two bands is not the
fix on its own, so the comment says what the numbers mean and `docs/results.md`
reports the hunt. The joint has a one degree dead band at the handle and no
amount of gain removes it; friction compensation or a torque sensor would.

**Entry 34, the harness accepted a flag and ignored it.** `--coupling-stiffness`
parsed, appeared in the summary line, and never reached the plant, because
`mechanics_for` returned the preset directly and the override was written
against the wrong variable. A flag that changes the banner and not the physics
is worse than no flag, because a run that looks like a sweep of stiffness is
not one. Applied now, and the summary line prints the coupling the plant is
actually using rather than the one the preset asked for.

## 2026-11-26, day 5

**Entry 35, the thermal inversion returned the square of the current.**
`thermal_current_for_winding_c` solved the loss balance for current and returned
the square of it, so the derating table said a 25 degC bench allowed 18.6 A
instead of 4.32 and the runaway current came out at 55.8 A instead of 7.47. The
test that caught it asserted a value, not a shape, and that is why it caught it:
a square looks plausible in a table. Two missing `std::sqrt` calls, and the
lesson is that an inversion function needs a round trip test that puts the
answer back through the forward function.

**Entry 36, the passivity controller counted energy it never delivered.** It
computed a damping from the whole energy deficit, multiplied by the rotor speed,
and got 290 N m out of a 2.85 N m motor. The torque then went through the
current clamp and the joint carried on at the limit, while the observer
subtracted the full `beta omega^2 dt` and reported an excess of -0.09 J, which
reads as passive. The trace is what exposed it: `torque_master_nm` at -290 next
to a 2.85 N m stall. Fixed by clamping the correction to stall torque and
charging the observer for the torque actually applied. The consequence is
honest and worse looking: the generated power column in `docs/results.md` went
from small numbers to hundreds of watts, because the earlier numbers were the
lie. A control test that asserted the controlled run generated less had to be
deleted rather than tuned, because in that test the reference is the joint's own
past angle and the controller changes the trajectory, so the assertion was not
even meaningful.

**Entry 37, the efficiency grid measured a joint being back driven.** The first
torque list ran to 0.13 N m, which is above the datasheet continuous torque and
above the speed PI's output limit of 2.74 A, so the top row of the map was the
drive saturating and the load dragging the joint. The efficiency at the top
torque came out above the one below it, which is not a thing that happens. The
grid now runs 0.02 to 0.12 N m and every point is inside the continuous rating.

**Entry 38, the bilateral study railed its own current on the damping term.**
The impedance mode multiplies the rendered damping by rotor speed, and through
the 7:1 gearbox the rotor turns seven times the output, so 0.2 N m s/rad at a
seeded 20 mrad of stretch asked for more torque than the bridge could make and
the first study was measuring its own clamp. Damping reduced to 0.02 and the
seeded stretch reduced, and the amplitude table now starts where the loop is
linear.

**Entry 39, the stub HAL kept its temperature between tests.** The stub's
temperature is process state, the new thermal tests left it at 131 degC, and the
firmware test that runs a tick against the stub then latched an over
temperature fault and failed on a fault word it had never asked about. Test
order dependence of this kind is the worst sort, because it passes alone and
fails in the suite. The test sets 25 degC at the top now, which is the same
principle as every other stub reset in this project.

**Entry 40, clang-tidy had opinions the code did not have to earn.** Signed
operand with a bitwise operator on the fault masks and on the delay line's
xorshift, and a floating point loop counter in the derating table. The fault
bits became `(1u << Nu)` in `hal.h` and the ambient sweep became an integer
index. Worth noting that the local clang-tidy is stricter than the one CI ran on
day 4, so a green run there is not evidence about the code.

## Template for later entries

```
## DATE, day N
**Entry N, one line title.** What was observed, what the cause was, what was
changed, what the check now proves.
```
