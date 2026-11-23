/* ideal_plant.h, rung 1 of the fidelity ladder: back EMF, rotor inertia and
 * viscous friction only. No switching, no saturation, no friction torque. */
#ifndef TSIM_SIM_IDEAL_PLANT_H
#define TSIM_SIM_IDEAL_PLANT_H

#include "tsim/plant.h"

namespace tsim {

class IdealPlant final : public IPlant {
  public:
    explicit IdealPlant(const PlantParams &params);

    void configure(const PlantParams &params) override;
    void reset(double theta_rotor_rad) override;
    void latch_inverter(const InverterCommand &command) override;
    void step_one() override;
    std::uint64_t step_count() const override;
    double time_s() const override;
    ElectricalReadout sample_electrical() const override;
    MechanicalState mechanical() const override;
    Telemetry telemetry() const override;
    EnergyAccount energy() const override;
    void set_external_torque_nm(double torque_nm) override;
    void set_rotor_locked(bool locked) override;
    void set_step_ns_for_convergence(std::uint64_t step_ns) override;
    const PlantParams &params() const override;

  private:
    PlantParams params_;
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
    double p_electrical_w_ = 0.0;
    double p_copper_w_ = 0.0;
    double p_mechanical_w_ = 0.0;
    std::uint64_t steps_ = 0;
    EnergyAccount energy_{};
};

} // namespace tsim

#endif /* TSIM_SIM_IDEAL_PLANT_H */
