/* firmware.h, the drive entry points. Units in names, frames per field. */
#ifndef TSIM_FIRMWARE_H
#define TSIM_FIRMWARE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* What the reference path is doing. A mode change zeroes both integrators,
 * because carrying an integrator across a reference step is how a loop winds
 * up before its first tick. */
typedef enum {
    FIRMWARE_MODE_OFF = 0,       /* gates on, zero vector, no torque */
    FIRMWARE_MODE_CURRENT = 1,   /* q axis current reference, A */
    FIRMWARE_MODE_SPEED = 2,     /* rotor speed reference, rad/s */
    FIRMWARE_MODE_POSITION = 3,  /* rotor position reference, rad, speed cascade */
    FIRMWARE_MODE_IMPEDANCE = 4, /* spring and damper to a position reference */
} firmware_mode_t;

typedef struct {
    uint32_t ticks;              /* control ticks executed since boot, count */
    firmware_mode_t mode;        /* active reference path */
    float theta_e_rad;           /* electrical angle, rad, electrical frame */
    float current_peak_a;        /* largest phase current magnitude in the last tick, A */
    float omega_rad_s;           /* measured rotor speed, rad/s, rotor frame */
    float iq_ref_a;              /* q axis current reference in use, A */
    float id_a;                  /* measured d axis current, A, rotor frame */
    float iq_a;                  /* measured q axis current, A, rotor frame */
    float vd_v;                  /* voltage applied, d axis, V */
    float vq_v;                  /* voltage applied, q axis, V */
    float vd_ff_v;               /* d axis feedforward inside vd_v, V */
    float vq_ff_v;               /* q axis feedforward inside vq_v, V */
    float bus_v;                 /* DC link measured this tick, V */
    bool dead_time_compensation; /* compensation enabled this tick, bool */
    uint32_t fault_bits;         /* last latched fault mask, HAL_FAULT_* bits */
    /* Day 4. The position quantities are rotor frame, rad, and remain valid
     * past one turn because the encoder counts are continuous. */
    float position_rad;          /* measured rotor position, rad */
    float position_ref_rad;      /* position reference in use, rad */
    float position_error_rad;    /* reference minus measured, rad */
    float position_kp_per_s;     /* commissioned position gain, 1/s */
    float speed_crossover_rad_s; /* commissioned speed loop crossover, rad/s */
    float speed_command_rad_s;   /* speed the cascade is asking for, rad/s */
    float torque_ref_nm;         /* impedance torque reference, N m */
    float stiffness_nm_per_rad;  /* rendered stiffness, N m/rad */
    float damping_nm_s_per_rad;  /* rendered damping, N m s/rad */
    /* Day 5. The winding has a thermal limit and the drive has to act on it,
     * so the temperature it reads is used twice: to pull the current envelope
     * down before the limit, and to open the gates at it. */
    float temperature_c;   /* winding temperature the HAL reported, degC */
    float thermal_limit_a; /* current envelope after derating, A */
    /* Time domain passivity, opt in. The observer accumulates the energy the
     * rendered port has taken in; `passivity_excess_j` is what is left after
     * the spring's stored energy is accounted for, so a negative value means
     * the render is generating energy rather than returning it. */
    float passivity_energy_j;             /* port energy accumulated, J */
    float passivity_excess_j;             /* negative means active, J */
    float passivity_damping_nm_s_per_rad; /* damping the controller added, N m s/rad */
    /* Day 6. The other end's port energy, as it arrived over the link and as the
     * observer used it. Zero when this drive is a one port render. */
    float passivity_peer_energy_j; /* peer port energy in use, J */
} firmware_status_t;

/* Bring the drive up: HAL, sensor offsets, PWM off then armed. */
void firmware_init(void);

/* Start the 20 kHz tick. The harness does not call this, it calls the tick. */
void firmware_start(void);

/* Choose the reference path and clear the integrators. */
void firmware_set_mode(firmware_mode_t mode);

/* q axis current reference, A. Used in current mode. Positive produces positive
 * torque. Clipped to the continuous current rating. */
void firmware_set_current_reference_a(float iq_a);

/* Rotor speed reference, rad/s, rotor frame. Used in speed mode. */
void firmware_set_speed_reference_rad_s(float omega_rad_s);

/* Enable or disable gate dead time compensation. Enabled at boot, because a
 * drive that knows its dead time should use it. Disabled only to measure what
 * it buys, which is what docs/results.md day 3 does. */
void firmware_set_dead_time_compensation(bool enable);

/* Rotor position reference, rad, rotor frame, measured from the encoder zero.
 * Used in position and impedance modes. Entering either mode seeds the
 * reference to where the shaft already is, so a mode change is not a move to
 * zero; this call is how the reference then actually moves. */
void firmware_set_position_reference_rad(float theta_rad);

/* Rendered impedance: torque = stiffness * position error - damping * speed.
 * Both terms are clamped at zero, and the torque is limited by stall current.
 * Damping is only as good as the velocity estimate it is multiplied by, which
 * is a 2 kHz ten tick difference and not an observer. */
void firmware_set_impedance(float stiffness_nm_per_rad, float damping_nm_s_per_rad);

/* The passivity controller on the impedance render, Hannaford and Ryu, IEEE
 * Transactions on Robotics and Automation 18(1), 2002. The observer itself
 * always runs, because it is an observation and a drive that renders an
 * impedance should be able to say how much energy it has returned. This call
 * enables the controller, which changes the torque and is therefore off by
 * default: the day 4 render was measured without it.
 */
void firmware_set_passivity_control(bool enable);

/* The other end's accumulated port energy, J, as it arrives over the link. A
 * port that is delivering work has a positive energy in this sign convention,
 * the same one `passivity_energy_j` uses, so a pair that is generating has two
 * positive numbers and a pair that is passing energy back and forth has one of
 * each. The observer adds this to its own before it decides, which is the whole
 * of the difference between a one port observer and a two port one: a drive
 * cannot see the far side's energy by measuring its own port, so the only way
 * it ever learns it is from the network.
 *
 * Zero is the one port behaviour and the default, so every day 5 number stands.
 */
void firmware_set_link_energy_j(float energy_j);

/* One control tick, 20 kHz. Must finish inside 50 us. */
void firmware_tick(void);

/* Latest status snapshot. Never NULL after firmware_init. */
const firmware_status_t *firmware_status(void);

#ifdef __cplusplus
}
#endif

#endif /* TSIM_FIRMWARE_H */
