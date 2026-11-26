/* switching_plant.h, rung 3 of the fidelity ladder: the same motor as
 * IdealPlant, driven by an inverter that switches.
 *
 * What is added over the ideal plant, all of it cited or marked ASSUMED in
 * docs/parameters.md:
 *   - the carrier is resolved at the plant step, so the winding sees the
 *     switching waveform and the ripple is a result rather than a formula
 *   - gate dead time, as the standard average voltage error per leg,
 *     reduced near a current zero crossing where the sign is not single
 *     valued inside one switching period
 *   - one conducting device of on resistance in series with each phase
 *   - energy lost per device transition, drawn from the DC link
 *   - a DC link that is a capacitor with ESR fed through a source resistance,
 *     so the bus moves at the switching frequency under load
 *
 * The dead time error is applied as an average over the step rather than as
 * two edges, because a 500 ns dead time cannot be resolved on a 1 us step.
 * The bus current is computed from the resolved leg states, so the ripple in
 * the capacitor voltage is physical at the step size, not a fitted waveform.
 */
#ifndef TSIM_SIM_SWITCHING_PLANT_H
#define TSIM_SIM_SWITCHING_PLANT_H

#include "tsim/plant.h"

namespace tsim {

class SwitchingPlant final : public IPlant {
  public:
    SwitchingPlant(const PlantParams &params, const InverterParams &inverter);

    void configure(const PlantParams &params) override;
    void reset(double theta_rotor_rad) override;
    void latch_inverter(const InverterCommand &command) override;
    void step_one() override;
    std::uint64_t step_count() const override;
    double time_s() const override;
    double winding_temperature_c() const override;
    ElectricalReadout sample_electrical() const override;
    MechanicalState mechanical() const override;
    Telemetry telemetry() const override;
    EnergyAccount energy() const override;
    void set_external_torque_nm(double torque_nm) override;
    void set_rotor_locked(bool locked) override;
    void set_step_ns_for_convergence(std::uint64_t step_ns) override;
    const PlantParams &params() const override;

    /* Inverter constants in use. Read only, for tests and for reports. */
    const InverterParams &inverter() const;

  private:
    PlantParams params_;
    InverterParams inverter_;
    MechanicalSystem mechanics_;
    ThermalNetwork thermal_;
    bool locked_ = false;
    double step_s_ = kPlantStepS;
    InverterCommand command_{};
    double ia_a_ = 0.0;
    double ib_a_ = 0.0;
    double ic_a_ = 0.0;
    double theta_rotor_rad_ = 0.0;
    double omega_rotor_rad_s_ = 0.0;
    double torque_motor_nm_ = 0.0;
    double torque_external_nm_ = 0.0;
    double phase_voltage_a_v_ = 0.0;
    double phase_voltage_b_v_ = 0.0;
    double phase_voltage_c_v_ = 0.0;
    double bus_cap_v_ = 0.0;
    double bus_terminal_v_ = 0.0;
    double bus_current_a_ = 0.0;
    double switching_power_w_ = 0.0;
    double p_electrical_w_ = 0.0;
    double p_copper_w_ = 0.0;
    double p_mechanical_w_ = 0.0;
    std::uint64_t steps_ = 0;
    EnergyAccount energy_{};

    /* Winding resistance at the temperature the network is at right now. */
    double copper_resistance_ohm() const;

    /* Leg states and voltage for this step, filled by step_inverter(). */
    void step_inverter();
};

} // namespace tsim

#endif /* TSIM_SIM_SWITCHING_PLANT_H */
