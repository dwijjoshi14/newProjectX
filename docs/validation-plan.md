# Validation plan

How each model claim would be checked on real hardware, in order, with the
instrument named. This exists so that no simulated result is ever defended with
"it would probably match".

Status: day 3. This is the plan, not a record of tests performed. Nothing here
has been run on hardware, because there is no hardware. Items 6 and 7 now have
something to compare against, which is the point of having a model of them at
all: the dead time measurement has a predicted distortion to beat, and the
current sensor measurement has a predicted 32 mA quantisation step and a
predicted gain and offset error to confirm or refute.

## Order of checks on a first article

1. **Static resistance and inductance.** Four wire milliohm meter and an LCR
   meter at 1 kHz, all three phase pairs. Confirms R_pp and L_pp, and shows
   whether Ld and Lq differ enough to matter.
2. **Back EMF constant.** Spin the shaft with a drill or a lathe at a measured
   speed, scope the line to line voltage, fit Ke. Compare against the fitted
   flux linkage. Instruments: oscilloscope, tachometer or encoder.
3. **Torque constant.** Locked rotor, DC current through two phases, reaction
   torque on a load cell at a known lever arm. This is the measurement that
   fixes the current convention question, so it comes before any loop tuning.
4. **Cogging.** Slow back drive with a torque sensor and an encoder, one
   revolution, FFT the torque against angle. Gives the real harmonic spectrum
   instead of the assumed 0.5 percent.
5. **Saturation.** Inductance against DC bias current, LCR meter with a bias
   tee, or a step response at several current levels. Gives the real L(i) curve.
6. **Dead time and inverter non linearity.** Low inductance load, sweep
   commanded voltage at zero speed, measure the current distortion against
   commanded voltage. Gives dead time and its asymmetry directly.
7. **Current sensor chain.** Inject known currents with a precision source,
   record ADC counts, fit gain and offset. Then repeat at temperature.
8. **Thermal resistance.** DC current into one phase pair with the rotor locked,
   thermocouple on the winding and on the housing, wait for steady state at
   three power levels. Fits the three node network.
9. **Loop bandwidth and margin.** Swept sine on the q axis current reference
   with a signal analyser or a captured scope record. This is the same
   measurement the simulator produces, so the comparison is direct.
10. **Friction and compliance.** Free decay from several speeds for Coulomb and
    viscous terms, and a hammer tap with an accelerometer for the coupling
    resonance.
11. **Bilateral stability boundary.** Two joints, a variable delay line and a
    variable gain, find the edge by sweep. Compare against the passivity
    prediction from the model.

## What would falsify the model

- Measured current loop bandwidth below 60 percent of the simulated value.
- Measured cogging above 3 percent of continuous torque.
- Torque constant more than 5 percent away from the fitted flux linkage.
- A stability edge more than 20 percent away from the passivity prediction.

Any of those sends the model back for a parameter, not for a tuned fudge
factor. The fudge factor is how these projects usually die.
