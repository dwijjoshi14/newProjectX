# Model limits

What this model does not capture. Every item here is a claim that is not being
made, which is the point of the document.

Status: day 2. The plant is still the ideal one, so most of the day 1 list
stands. What changed is that a controller now runs against it, which puts a few
of the gaps on the control path where they cost something.

## Not modelled at all, as of day 2

- Switching. The inverter is an average voltage source. No PWM ripple, no dead
  time, no conduction or switching loss, no bus ripple. The modulator's duty
  ratios are exact, so the 1.76 A peak to peak ripple that
  `docs/parameters.md` predicts does not appear anywhere in these results.
- Sensing error. Currents are exact doubles. The encoder quantises to 14 bits
  and has no latency and no noise.
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
- The current loop has no cross coupling or back EMF feedforward, so the d axis
  PI earns its living rejecting the omega_e L iq term. Measured cost during a
  200 rad/s transient: 0.071 A of d axis current, 2.6 percent of the continuous
  rating. Day 3 adds the feedforward with the dead time compensation.
- The modulator removes its own common mode, so the plant cannot tell minimum
  maximum injection from sine triangle modulation. On hardware the difference
  is 15 percent of linear range, and it is asserted in the transform tests
  rather than in the plant.
- The voltage limit is 95 percent of bus / sqrt(3) with no dead time allowance.
  A real inverter at 500 ns and 20 kHz loses 1 percent of the volt seconds, and
  the compensation for it is day 3.
- The velocity estimate is a ten tick encoder difference divided by the nominal
  period, so it lags by up to half a millisecond and its resolution at 200 rad/s
  is 0.77 rad/s. That quantisation is visible as the 2 kHz staircase in the q
  current of the speed plot. It is real behaviour at the current loop rate, not
  plotting noise.
- The speed loop is measured with only the rotor's own inertia, 2.3e-6 kg m^2.
  A 50 mNm load step therefore looks violent. The dip is a property of the
  inertia and the pipeline delay, and day 4 changes it by two orders of
  magnitude when the handle arrives. Do not quote the day 2 dip as a property
  of the joint.
- The plant step is a fixed forward Euler integrator. Day 1 said its
  discretisation error was unquantified and scheduled a step halving test. That
  test exists now: the error at 1 us is 1.8e-5 relative on an RL step at 2 ms,
  and it halves with the step. The numbers are in `docs/results.md`.

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
