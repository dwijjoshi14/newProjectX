# Model limits

What this model does not capture. Every item here is a claim that is not being
made, which is the point of the document.

Status: day 1. The only plant that exists is the ideal one, so this list is
longer than the model. It shrinks as rungs land, and it never reaches empty.

## Not modelled at all, as of day 1

- Switching. The inverter is an average voltage source. No PWM, no dead time,
  no conduction or switching loss, no bus ripple.
- Sensing error. Currents are exact doubles, the encoder is exact and has no
  latency.
- Saturation. Inductance is constant.
- Cogging. Torque is a pure function of q axis current.
- Coulomb friction and stiction. Only viscous friction exists.
- Backlash, compliance, two mass behaviour. One rigid inertia.
- Thermal behaviour. Temperature is a fixed 25 degC.
- Network effects. There is no second joint and no channel.

## Modelled, but with known gaps that stay

- Back EMF and torque share one flux linkage constant. In reality the two
  differ slightly because of end effects and manufacturing tolerance.
- Viscous friction is linear. Real bearings are not, and stiction will be added
  in the mechanics rung rather than folded into a made up coefficient.
- The plant step is a fixed 1 us forward Euler integrator. It is stable for the
  0.88 ms electrical time constant, but the discretisation error is first order
  and has not yet been quantified by a step halving test. That test is scheduled
  before any result is published.

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
