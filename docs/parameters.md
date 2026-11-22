# Parameters

Every constant in the simulator traces to a row here. A row is either cited to
a datasheet with a retrieval date, or marked ASSUMED with the reason. There is
no third category.

Motor: maxon EC-i 40, 70 W, 36 V, part 496655. Retrieved 2026-11-22 from
https://www.maxongroup.com/maxon/view/product/496655 and the EC-i 40 catalogue
pages EN-306 and EN-307 (2025 edition). No NRND flag on the page as retrieved.
Re-check the flag before quoting this in an application.

The EC 45 flat 50 W parts 251601 and 651611 were rejected because both are
marked NRND on maxon's site on the same date.

## Datasheet values

| Quantity | Symbol | Value | Source |
|---|---|---|---|
| nominal voltage | V_bus | 36 V | datasheet |
| pole pairs | p | 7 | datasheet |
| terminal resistance, phase to phase | R_pp | 0.582 ohm | datasheet |
| terminal inductance, phase to phase | L_pp | 0.512 mH | datasheet |
| torque constant | Kt | 46.1 mNm/A | datasheet |
| speed constant | Ke | 207 rpm/V | datasheet |
| stall torque | | 1030 mNm | datasheet |
| stall current | | 61.8 A | datasheet |
| continuous torque | | 130 mNm | datasheet |
| continuous current | | 2.74 A | datasheet |
| rotor inertia | J | 23 gcm2 = 2.3e-6 kg m2 | datasheet |
| mechanical time constant | | 0.632 ms | datasheet |
| max speed | | 10000 rpm | datasheet |
| winding to housing resistance | Rth_wh | 2.27 K/W | datasheet |
| housing to ambient resistance | Rth_ha | 8.17 K/W | datasheet |
| winding thermal time constant | | 24.9 s | datasheet |
| motor thermal time constant | | 1020 s | datasheet |
| max winding temperature | | 155 degC | datasheet |
| ambient range | | -40 to +100 degC | datasheet |

## Derived values, with the arithmetic shown

| Quantity | Value | Derivation |
|---|---|---|
| phase resistance | 0.291 ohm | R_pp / 2, wye connection |
| phase inductance | 0.256 mH | L_pp / 2, wye connection |
| electrical time constant | 0.88 ms | L_phase / R_phase, pole at 181 Hz |
| flux linkage | 4.39 mWb | Kt / (1.5 p), peak phase convention |
| winding heat capacity | 11.0 J/K | 24.9 s / 2.27 K/W |
| housing heat capacity | 125 J/K | 1020 s / 8.17 K/W |
| switching ripple, 50 percent duty | 1.76 A peak to peak | V D (1-D) / (L f), f = 20 kHz |

Two cross checks, because a parameter table that disagrees with itself is
worthless:

1. `J R_pp / Kt^2` = 2.3e-6 * 0.582 / 0.002125 = 0.63 ms, against the datasheet
   mechanical time constant of 0.632 ms. Agreement 0.3 percent.
2. Kt in SI is 0.0461 N m/A. The speed constant in SI is 1 / (207 * 2 pi / 60) =
   0.0461 V s/rad. Agreement better than 0.1 percent.

Note on the ripple figure: 1.76 A peak to peak is 64 percent of the 2.74 A
continuous current. That is a real consequence of a 0.256 mH winding at 20 kHz,
not a modelling choice. Drives for this class of motor typically switch at
40 kHz or more. Switching frequency stays a parameter, default 20 kHz, and the
rung 3 report states the ripple and the loss trade off rather than hiding it.

## ASSUMED parameters

Each of these is a line in `docs/model-limits.md` as well, because an assumed
parameter is a limit on what the model can claim.

| Quantity | Value | Why it is assumed |
|---|---|---|
| Ld = Lq | equal | iron core surface magnet machine, saliency not published |
| cogging amplitude | 0.5 percent of continuous torque at order 84, 0.2 percent second harmonic | maxon states low cogging, publishes no spectrum |
| saturation knee | inductance falls 20 percent at 2x continuous current | no saturation curve published |
| Coulomb friction | 8 mNm | not published, typical for this size and bearings |
| viscous friction | 1e-5 N m s/rad | not published, chosen so coast down is observable |
| gate dead time | 500 ns, 40 ns asymmetry | typical for a 36 V half bridge, device not selected |
| MOSFET on resistance | 3 mOhm | device not selected |
| switching energy | 20 uJ per device per event | device not selected |
| bus capacitor | 100 uF, 20 mOhm ESR | not part of the drive model in rung 1 |
| shunt resistance | 0.5 mOhm | board not designed |
| shunt amplifier gain | 50 V/V | board not designed |
| ADC | 12 bit, 3.3 V reference, 4095 counts | STM32G4 class converter |
| encoder | 14 bit, 1.5 us latency, 0.05 deg RMS noise | resolution fixed by the brief, rest assumed |
| coupling stiffness | 0.9 N m/rad | joint not designed, gives a resonance near 100 Hz |
| coupling damping | 0.05 N m s/rad | joint not designed |
| hand inertia | 1.5e-4 kg m2 | hand on a 30 mm knob |
| hand spring and damping | 0.45 N m/rad, 0.02 N m s/rad | hand model, literature range |
| wall stiffness | 1.8 N m/rad | rigid wall on a 30 mm knob |
| network delay, jitter, loss | 2 ms, 1 ms, 0.5 percent | swept, not fixed, in rung 7 |
| junction to case resistance | 1.5 K/W | not published |
| ambient | 25 degC | bench |

## Handling rules

- A parameter may move from ASSUMED to cited only by adding a source to this
  file in the same commit.
- Any figure that depends on an assumed parameter states which one in its
  caption.
- If a number comes out worse than expected, the number is published and the
  expectation is corrected in `docs/results.md`.
