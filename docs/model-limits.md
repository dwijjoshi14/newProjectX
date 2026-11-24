# Model limits

What this model does not capture. Every item here is a claim that is not being
made, which is the point of the document.

Status: day 3. The inverter switches now, the link moves, and the sensing chain
is in the loop. Most of the day 2 list is therefore gone, and what replaced it
is a longer list of things that a switching model still does not say.

## What day 3 added, and what it still does not say

- **Switching.** The carrier is resolved at the 1 MHz plant step, with the
  fraction of each step a leg is high computed analytically rather than snapped
  to the step, because a 1 us grid is a 2 percent duty error over a 50 us
  period and that is 0.72 V at 36 V. The current ripple is therefore piecewise
  linear at 1 us and not continuous through the edge. Ripple amplitude and
  timing are right; the corner of each triangle is one step wide.
- **Dead time.** Modelled as the standard average voltage error per leg, signed
  by the phase current and faded linearly below 0.5 A. It says nothing about
  the dead time asymmetry that a real driver has, which is what turns dead time
  into a DC offset in the current rather than only distortion. A 40 ns
  asymmetry at 36 V is 0.03 V, or 0.1 A at this winding, and it is not here.
- **Conduction loss.** One device of on resistance in series with each leg, and
  no body diode forward drop. Freewheeling during the dead interval is therefore
  lossless in the model, so the energy the dead time removes appears as a
  voltage loss and not as a diode loss. Both are watts at these currents and
  the totals differ by the diode drop times the current.
- **Switching loss.** A triangular overlap of bus and current over a 50 ns
  crossover. It ignores the current and voltage dependence of the real
  transition, diode reverse recovery, and the output charge of the device.
- **Sensing.** Shunt gain error, amplifier offset, 12 bit quantisation, 1.5 us
  of encoder latency and 0.05 deg RMS of encoder noise. It does not model
  drift with temperature, common mode rejection, sampling jitter, or the
  aperture of the sample and hold. The converter is otherwise noiseless.
- **The current measurement instant** is exact: the plant is sampled at the
  window centre and no anti alias filter sits in front of the converter, so the
  alias of the switching ripple into the current loop is not represented beyond
  the sampling instant itself.
- **The DC link** is a capacitor, an ESR and a source resistance. There is no
  rectifier, no precharge, no regeneration limit and no bus overvoltage from
  energy returned by the motor. A haptic joint that is back driven will raise
  the bus in this model and nothing stops it.

## Not modelled at all, as of day 3

- Saturation. Inductance is constant against current.
- Cogging. Torque is a pure function of q axis current and rotor angle through
  the back EMF only.
- Coulomb friction and stiction. Only viscous friction exists, so a real joint
  will hold position better and start worse than this.
- Backlash, compliance, two mass behaviour. One rigid inertia.
- Thermal behaviour. Temperature is a fixed 25 degC, and there is no thermal
  limit on the current reference. Current mode can command stall current for
  the length of a run and the model will allow it.
- Network effects. There is no second joint and no channel.

## Modelled, but with known gaps that stay

- Back EMF and torque share one flux linkage constant. In reality the two
  differ slightly because of end effects and manufacturing tolerance.
- Viscous friction is linear. Real bearings are not.
- The current loop now has d/q decoupling and back EMF feedforward, and both use
  the 2 kHz velocity estimate, which lags by up to 0.5 ms and quantises to one
  encoder count per update. Feedforward is only as good as that estimate, and
  the day 3 transient results show it: the d axis error during a 200 rad/s step
  fell from 0.071 A to 0.025 A, while the load dip grew from 27.5 to 29.7
  rad/s. Neither number is a property of the joint.
- The velocity estimate is a ten tick encoder difference divided by the nominal
  period. Its resolution at 200 rad/s is 0.77 rad/s and at 30 rad/s it is 0.77
  rad/s as well, because the count rate falls with the speed: at 30 rad/s the
  estimate is quantised to 2.4 percent of its own value, and with the sensing
  chain on, the 2.3 count encoder noise makes it 5 percent. That is why the
  low speed distortion result is quoted with the sensing chain off and again
  with it on, and why the floor of the second number is the encoder and not the
  inverter. A joint meant to run this slowly needs an observer, which this
  project does not have yet.
- The modulation removes its own common mode, so the plant cannot tell minimum
  maximum injection from sine triangle modulation. On hardware the difference
  is 15 percent of linear range, and it is asserted in the transform tests
  rather than in the plant.
- The voltage limit is 95 percent of bus / sqrt(3), and the dead time
  compensation is applied after it, so a compensated request near the limit
  clips in the modulator instead of in the limiter.
- The speed loop is measured with only the rotor's own inertia, 2.3e-6 kg m^2.
  Day 4 changes that by two orders of magnitude.
- The plant step is a fixed forward Euler integrator with a measured error of
  1.8e-5 relative on an RL step at 2 ms, halving with the step.

## Never captured, at any rung

- Eddy currents and proximity effect in the winding.
- Temperature dependence of resistance beyond a linear coefficient.
- Manufacturing spread between two units of the same motor.
- Acoustic noise, bearing wear, lubricant behaviour over life.
- Magnetic saturation depth beyond the fitted curve, including cross
  saturation between d and q axes.
- Mechanical compliance beyond the two mass lump.
- Sensor non linearity beyond the stated gain and offset error.

Each of these is listed again in `docs/validation-plan.md` with the instrument
that would measure it and the order in which it should be checked on hardware.
