/* thermal.h, the winding and housing as two lumps instead of a current that
 * costs nothing.
 *
 * Why this exists: every run in this repository before day 5 could pull 20 A
 * through a 0.291 ohm winding forever and nothing objected. That is the one
 * thing a real drive never lets you do, and it is the reason a current limit
 * with no thermal model behind it is a number somebody chose and not a limit.
 *
 * The model is the datasheet's own two resistances and two time constants:
 * winding to housing 2.27 K/W with 11.0 J/K, housing to ambient 8.17 K/W with
 * 125 J/K. Copper resistance rises with temperature, 0.00393 per K at 20 degC,
 * so the loss a current produces is itself a function of the temperature that
 * loss produced. That feedback is the whole point: below about 7.5 A there is a
 * steady state and above it there is none, and a model without the temperature
 * coefficient cannot see the difference.
 *
 * Disabled by default, like the sensing chain, so day 1 to day 4 reproduce bit
 * for bit. A disabled network reports the ambient and a resistance multiplier
 * of exactly 1.0, which means the plants do the arithmetic they always did.
 */
#ifndef TSIM_SIM_THERMAL_H
#define TSIM_SIM_THERMAL_H

namespace tsim {

/* Every value here is cited or marked ASSUMED in docs/parameters.md. */
struct ThermalParams {
    bool enabled;                         /* false = a stiff 25 degC world */
    double ambient_c;                     /* bench, degC */
    double winding_to_housing_k_per_w;    /* Rth winding to housing, K/W */
    double housing_to_ambient_k_per_w;    /* Rth housing to ambient, K/W */
    double winding_heat_capacity_j_per_k; /* C winding, J/K */
    double housing_heat_capacity_j_per_k; /* C housing, J/K */
    double copper_alpha_per_k;            /* 0.00393 at 20 degC, 1/K */
    double copper_reference_c;            /* temperature alpha is quoted at, degC */
    double initial_winding_c;             /* starting winding temperature, degC */
    double initial_housing_c;             /* starting housing temperature, degC */
};

/* Resistance multiplier for a winding at a temperature. Pure copper physics,
 * independent of whether the network is enabled: R(T) / R(20 degC). The
 * efficiency map uses it to hold a winding at a stated temperature without
 * running the network, which is why it is not a member. */
double thermal_copper_multiplier(const ThermalParams &params, double winding_c);

/* Winding temperature at which a constant copper loss settles, degC. The
 * steady state of the two node chain is the loss times the sum of the two
 * resistances, which is worth writing down once. */
double thermal_steady_state_winding_c(const ThermalParams &params, double copper_loss_w);

/* Copper loss for a balanced three phase set of peak phase current current_a,
 * W. Three sinusoids of amplitude I carry 1.5 I^2 R between them, not 3 I^2 R:
 * the factor matters by a factor of two and getting it wrong makes every
 * thermal number here half what it should be. */
double thermal_copper_loss_w(double current_a, double phase_resistance_ohm);

/* Steady state winding temperature of a constant current, degC. Above the
 * runaway current there is no steady state and this returns 1e9, because
 * returning the ambient or the limit would hide a motor that is burning. */
double thermal_winding_c_at_current(const ThermalParams &params, double current_a, double ambient_c,
                                    double phase_resistance_ohm);

/* Current that reaches a winding temperature from an ambient, A. The inverse
 * of the line above, and the derating curve. */
double thermal_current_for_winding_c(const ThermalParams &params, double winding_limit_c,
                                     double ambient_c, double phase_resistance_ohm);

/* Current above which the copper loss grows faster with temperature than the
 * housing can dump it, A. Ambient independent: it is where the loss line is
 * tangent to the dissipation line. */
double thermal_runaway_current(const ThermalParams &params, double phase_resistance_ohm);

class ThermalNetwork {
  public:
    ThermalNetwork();

    void configure(const ThermalParams &params);

    /* Back to the initial temperatures, at rest. */
    void reset();

    /* One step. Forward Euler like everything else here; at the 1 us plant step
     * the fastest constant is 25 s, so the step is 2.5e-7 of it. */
    void step(double dt_s, double copper_loss_w);

    double winding_temperature_c() const;
    double housing_temperature_c() const;

    /* 1.0 when the network is disabled, so a plant that never asked for heat
     * does exactly the arithmetic it did before this class existed. */
    double copper_resistance_multiplier() const;

    /* Rth * C for each node, s. The datasheet publishes both, so they are a
     * check on the four numbers rather than four free parameters. */
    double winding_time_constant_s() const;
    double housing_time_constant_s() const;

    const ThermalParams &params() const;

  private:
    ThermalParams params_{};
    double winding_c_ = 0.0;
    double housing_c_ = 0.0;
};

} // namespace tsim

#endif /* TSIM_SIM_THERMAL_H */
