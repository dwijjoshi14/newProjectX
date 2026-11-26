/* plant.h, the plant interface. Nothing outside this interface advances time.
 * Units are in names. Angles are rad, speeds rad/s, currents A, voltages V,
 * torques N m, time s unless a field says otherwise. */
#ifndef TSIM_SIM_PLANT_H
#define TSIM_SIM_PLANT_H

#include <cstdint>

#include "tsim/mechanics.h"
#include "tsim/thermal.h"

namespace tsim {

inline constexpr std::uint64_t kPlantStepNs = 1000;      /* 1 MHz plant step, ns */
inline constexpr std::uint64_t kControlPeriodNs = 50000; /* 20 kHz control, ns */
inline constexpr std::uint32_t kStepsPerControl = 50;    /* plant steps per tick, count */
inline constexpr std::uint64_t kSampleOffsetNs = 25000;  /* ADC trigger at window centre, ns */
inline constexpr double kPlantStepS = 1.0e-9 * static_cast<double>(kPlantStepNs);
inline constexpr double kControlPeriodS = 1.0e-9 * static_cast<double>(kControlPeriodNs);
inline constexpr double kTwoPi = 6.283185307179586;
inline constexpr double kTwoPiOverThree = 2.0943951023931953;

struct PlantParams {
    double pole_pairs;                    /* pole pairs, count */
    double resistance_phase_ohm;          /* per phase, ohm, phase frame */
    double inductance_phase_h;            /* per phase, H, phase frame */
    double flux_linkage_wb;               /* rotor flux linkage, Wb */
    double inertia_kg_m2;                 /* rotor plus load, kg m^2, rotor frame */
    double viscous_friction_nm_s_per_rad; /* viscous friction, N m s/rad */
    double bus_voltage_v;                 /* DC link, V, inverter frame */
    /* Everything past the shaft: coupling, load, hand, wall, backlash and
     * Coulomb friction. All zero is one rigid body with viscous drag, which is
     * exactly the model day 1 to day 3 used. */
    MechanicalParams mechanics;
    /* The winding and the housing. Disabled by default, which means the
     * resistance stays at the value above and day 1 to day 4 are unchanged. */
    ThermalParams thermal;
};

struct InverterCommand {
    float duty_a;        /* duty, phase frame, 0 to 1 */
    float duty_b;        /* duty, phase frame, 0 to 1 */
    float duty_c;        /* duty, phase frame, 0 to 1 */
    bool bridge_enabled; /* false means the windings are open, bool */
};

/* The inverter, as opposed to the motor. An all zero instance with
 * instantaneous_switching false is the average voltage source of rung 1, which
 * is what makes day 1 and day 2 results reproducible after this struct exists.
 * Values are cited or marked ASSUMED in docs/parameters.md. */
struct InverterParams {
    bool instantaneous_switching;     /* resolve the carrier, or apply the duty average */
    double dead_time_s;               /* gate dead time per edge, s */
    double on_resistance_ohm;         /* one conducting device, ohm */
    double switching_crossover_s;     /* voltage and current overlap per transition, s */
    double bus_capacitance_f;         /* DC link capacitance, 0 means a stiff bus */
    double bus_esr_ohm;               /* capacitor ESR, ohm */
    double bus_source_resistance_ohm; /* supply plus cabling into the capacitor, ohm */
    double dead_time_linear_a;        /* current below which the dead time error fades, A */
};

struct ElectricalReadout {
    double ia_a;  /* phase A current, A, phase frame */
    double ib_a;  /* phase B current, A, phase frame */
    double ic_a;  /* phase C current, A, phase frame */
    double va_v;  /* phase A voltage the winding sees, common mode removed, V */
    double vb_v;  /* phase B voltage the winding sees, common mode removed, V */
    double vc_v;  /* phase C voltage the winding sees, common mode removed, V */
    double bus_v; /* DC link at the inverter terminals, V, inverter frame */
};

struct MechanicalState {
    double theta_rotor_rad;    /* rotor angle, rad, rotor frame */
    double omega_rotor_rad_s;  /* rotor speed, rad/s, rotor frame */
    double torque_motor_nm;    /* torque produced this step, N m, rotor frame */
    double torque_external_nm; /* applied load torque, N m, rotor frame */
    /* Day 4. With a rigid coupling the load fields equal the rotor fields. */
    double theta_load_rad;     /* load side angle, rad */
    double omega_load_rad_s;   /* load side speed, rad/s */
    double torque_coupling_nm; /* rotor into the load, N m */
    double torque_hand_nm;     /* hand reaction on the load, N m */
    double torque_wall_nm;     /* wall reaction on the load, N m */
    double torque_coulomb_nm;  /* shaft friction this step, N m */
};

struct EnergyAccount {
    double electrical_j; /* energy drawn from the DC link, J */
    double copper_j;     /* energy lost in the windings, J */
    double viscous_j;    /* energy lost to viscous friction, J */
    double mechanical_j; /* energy delivered to the rotor by the motor, J */
    double switching_j;  /* energy lost in the transition of the devices, J */
    double coulomb_j;    /* energy lost to shaft friction, J */
    double elastic_j;    /* stored in the coupling and the hand right now, J */
};

struct Telemetry {
    double p_electrical_w; /* instantaneous DC link power, W */
    double p_copper_w;     /* instantaneous winding loss, W */
    double p_mechanical_w; /* instantaneous mechanical power, W */
    double torque_nm;      /* instantaneous motor torque, N m */
};

class IPlant {
  public:
    virtual ~IPlant() = default;

    /* Replace every parameter. Resets nothing, call reset afterwards. */
    virtual void configure(const PlantParams &params) = 0;

    /* Zero the state and the energy accounts at a rotor angle, rad. */
    virtual void reset(double theta_rotor_rad) = 0;

    /* Duty applied from the next plant step, phase and inverter frames. */
    virtual void latch_inverter(const InverterCommand &command) = 0;

    /* Exactly one plant step of kPlantStepNs. The only time advancing call. */
    virtual void step_one() = 0;

    virtual std::uint64_t step_count() const = 0;

    virtual double time_s() const = 0;

    /* Winding temperature, degC. Ambient when the thermal network is off, which
     * is what a board with no thermistor would report and what makes the drive's
     * over temperature logic a no op at 25 degC. */
    virtual double winding_temperature_c() const = 0;

    virtual ElectricalReadout sample_electrical() const = 0;

    virtual MechanicalState mechanical() const = 0;

    virtual Telemetry telemetry() const = 0;

    virtual EnergyAccount energy() const = 0;

    /* Load torque applied to the rotor, N m, rotor frame, positive opposes
     * positive rotation. */
    virtual void set_external_torque_nm(double torque_nm) = 0;

    /* Hold the rotor still, the way a dyno clamp does, so a current loop can be
     * measured at standstill without the speed loop fighting it. The motor
     * torque goes into the clamp: no speed, no angle change, and no mechanical
     * energy leaves the shaft. */
    virtual void set_rotor_locked(bool locked) = 0;

    /* Integration step for a convergence study, ns. The harness never calls
     * this, it runs kPlantStepNs. Halving the step and watching the answer
     * settle is how the first order error gets bounded rather than assumed. */
    virtual void set_step_ns_for_convergence(std::uint64_t step_ns) = 0;

    virtual const PlantParams &params() const = 0;
};

} // namespace tsim

#endif /* TSIM_SIM_PLANT_H */
