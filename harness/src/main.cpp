/* Deterministic harness. Three scenarios, all closed loop through the real
 * firmware: a locked rotor current step, a speed step with a load step, and a
 * low speed run for the dead time study. Time is integer, no clock is read, so
 * a repeated run is byte identical on the same compiler and flags.
 *
 * The plant defaults to the switching model, because that is the honest one.
 * `--plant ideal` selects the average voltage source of day 1 and 2, which is
 * how those results are reproduced after day 3. */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>

#include "firmware.h"
#include "hal.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"

namespace {

enum class Scenario { kSpeedStep, kCurrentStep, kDeadTime };

/* Average voltage source, or the inverter that switches. */
enum class PlantModel { kIdeal, kSwitching };

struct ScenarioConfig {
    Scenario scenario = Scenario::kSpeedStep;
    PlantModel plant = PlantModel::kSwitching;
    std::uint64_t ticks = 6000;  /* windows, count */
    std::uint64_t log_every = 5; /* windows between CSV rows, count */
    float speed_reference_rad_s = 0.0f;
    float current_reference_a = 0.0f;
    std::uint64_t load_step_tick = 0; /* window the load torque is applied at */
    double load_torque_nm = 0.0;
    bool locked_rotor = false;
    bool imperfections = true;          /* sensing chain on */
    bool dead_time_compensation = true; /* firmware compensation on */
    double dead_time_ns = 500.0;        /* gate dead time in the model, ns */
    const char *out_path = "";
};

/* 300 ms at 200 rad/s, with 50 mNm applied at 150 ms. The load is 38 percent of
 * continuous torque, which is a disturbance a joint would actually see. */
ScenarioConfig speed_step_config() {
    ScenarioConfig config;
    config.scenario = Scenario::kSpeedStep;
    config.ticks = 6000;
    config.log_every = 5;
    config.speed_reference_rad_s = 200.0f;
    config.load_step_tick = 3000;
    config.load_torque_nm = 0.05;
    config.locked_rotor = false;
    config.out_path = "data/run_latest/day2_speed_step.csv";
    return config;
}

/* 6 ms on a clamped rotor: +2 A, then -2 A, then 0. Steps are far enough apart
 * that each one finishes before the next starts. */
ScenarioConfig current_step_config() {
    ScenarioConfig config;
    config.scenario = Scenario::kCurrentStep;
    config.ticks = 120;
    config.log_every = 1;
    config.current_reference_a = 2.0f;
    config.locked_rotor = true;
    config.out_path = "data/run_latest/day2_current_step.csv";
    return config;
}

/* 300 ms at 29.9216 rad/s, which with seven pole pairs is 33.3333 Hz
 * electrical, and therefore exactly ten electrical cycles in 300 ms at the
 * 20 kHz log rate. That is not a round number for its own sake: it puts the
 * fundamental and every harmonic on an exact FFT bin, so the distortion figure
 * cannot be an artefact of a window leaking into its own harmonics.
 *
 * The load is 90 mNm, so the current is about 1.95 A, and the dead time error
 * of 0.36 V is a large fraction of the 1.7 V of back EMF at this speed.
 *
 * The sensing chain is off by default for this scenario, because the figure is
 * about what the inverter does and a quantised encoder at 30 rad/s adds a
 * velocity ripple four times the size of the distortion being measured. It is
 * a switch, not a quiet edit: --imperfections on runs the same scenario with
 * the converter and the encoder in the loop, which is the floor a real joint
 * would start from, and docs/results.md reports both. */
ScenarioConfig dead_time_config() {
    ScenarioConfig config;
    config.scenario = Scenario::kDeadTime;
    config.ticks = 6000;
    config.log_every = 1;
    config.speed_reference_rad_s = 29.9216f;
    config.load_step_tick = 0;
    config.load_torque_nm = 0.09;
    config.locked_rotor = false;
    config.imperfections = false;
    config.out_path = "data/run_latest/day3_dead_time.csv";
    return config;
}

constexpr std::uint64_t kCurrentStepNegativeAt = 40; /* window, count */
constexpr std::uint64_t kCurrentStepZeroAt = 80;     /* window, count */

float current_reference_at(const ScenarioConfig &config, std::uint64_t tick) {
    if (config.scenario != Scenario::kCurrentStep) {
        return config.current_reference_a;
    }
    if (tick < kCurrentStepNegativeAt) {
        return config.current_reference_a;
    }
    if (tick < kCurrentStepZeroAt) {
        return -config.current_reference_a;
    }
    return 0.0f;
}

double load_torque_at(const ScenarioConfig &config, std::uint64_t tick) {
    return (tick >= config.load_step_tick) ? config.load_torque_nm : 0.0;
}

const char *scenario_name(Scenario scenario) {
    switch (scenario) {
    case Scenario::kSpeedStep:
        return "speed_step";
    case Scenario::kCurrentStep:
        return "current_step";
    case Scenario::kDeadTime:
        return "dead_time";
    }
    return "unknown";
}

bool read_on_off(const char *text, bool *out_value) {
    if (std::strcmp(text, "on") == 0) {
        *out_value = true;
        return true;
    }
    if (std::strcmp(text, "off") == 0) {
        *out_value = false;
        return true;
    }
    return false;
}

void print_usage(const char *program) {
    std::fprintf(
        stderr,
        "usage: %s [--scenario speed_step|current_step|dead_time] [--plant ideal|switching]\n"
        "          [--dead-time-ns N] [--compensation on|off] [--imperfections on|off]\n"
        "          [--ticks N] [--log-every N] [--out PATH]\n",
        program);
}

ScenarioConfig parse_options(int argc, char **argv) {
    ScenarioConfig config = speed_step_config();
    bool scenario_chosen = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            const char *name = argv[++i];
            if (std::strcmp(name, "speed_step") == 0) {
                config = speed_step_config();
            } else if (std::strcmp(name, "current_step") == 0) {
                config = current_step_config();
            } else if (std::strcmp(name, "dead_time") == 0) {
                config = dead_time_config();
            } else {
                std::fprintf(stderr, "unknown scenario: %s\n", name);
                std::exit(2);
            }
            scenario_chosen = true;
        } else if (std::strcmp(argv[i], "--plant") == 0 && i + 1 < argc) {
            const char *name = argv[++i];
            if (std::strcmp(name, "ideal") == 0) {
                config.plant = PlantModel::kIdeal;
            } else if (std::strcmp(name, "switching") == 0) {
                config.plant = PlantModel::kSwitching;
            } else {
                std::fprintf(stderr, "unknown plant: %s\n", name);
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--dead-time-ns") == 0 && i + 1 < argc) {
            config.dead_time_ns = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--compensation") == 0 && i + 1 < argc) {
            if (!read_on_off(argv[++i], &config.dead_time_compensation)) {
                std::fprintf(stderr, "compensation wants on or off\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--imperfections") == 0 && i + 1 < argc) {
            if (!read_on_off(argv[++i], &config.imperfections)) {
                std::fprintf(stderr, "imperfections wants on or off\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--ticks") == 0 && i + 1 < argc) {
            config.ticks = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--log-every") == 0 && i + 1 < argc) {
            config.log_every = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            config.out_path = argv[++i];
        } else {
            print_usage(argv[0]);
            std::exit(2);
        }
    }
    if (!scenario_chosen && config.ticks == 0) {
        config.ticks = 1;
    }
    if (config.log_every == 0) {
        config.log_every = 1;
    }
    if (config.dead_time_ns < 0.0) {
        config.dead_time_ns = 0.0;
    }
    return config;
}

} // namespace

int main(int argc, char **argv) {
    const ScenarioConfig config = parse_options(argc, argv);

    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    std::unique_ptr<tsim::IPlant> plant;
    if (config.plant == PlantModel::kIdeal) {
        plant = std::make_unique<tsim::IdealPlant>(params);
    } else {
        tsim::InverterParams inverter = tsim::inverter_params_realistic();
        inverter.dead_time_s = config.dead_time_ns * 1.0e-9;
        plant = std::make_unique<tsim::SwitchingPlant>(params, inverter);
    }

    /* In the clamped rotor scenario the shaft is parked a quarter of an
     * electrical turn in, where the q axis lines up with phase A. Clamped at
     * zero instead, the q axis sits between B and C, phase A carries only the d
     * axis current and the trace is a flat line at zero. True, and useless. */
    const double initial_angle_rad =
        config.locked_rotor ? -tsim::kTwoPi / (4.0 * params.pole_pairs) : 0.0;
    plant->reset(initial_angle_rad);
    plant->set_rotor_locked(config.locked_rotor);
    sim_hal_attach(*plant);
    sim_hal_set_board_dead_time_s(static_cast<float>(config.dead_time_ns * 1.0e-9));
    sim_hal_set_imperfections(config.imperfections);

    /* A board's converter is running before the bridge is enabled, and the
     * firmware averages that first conversion to find its current offsets. The
     * harness has to publish the same first sample or the drive would run with
     * the raw amplifier offset, which is a bug in the harness rather than a
     * property of the drive. The plant current is zero here because the bridge
     * has never been enabled, so this is the truth and not a spoof. */
    sim_hal_latch_sample(plant->sample_electrical(), 0u);

    firmware_init();
    if (firmware_status()->fault_bits != 0u) {
        std::fprintf(stderr, "firmware latched a fault during boot\n");
        return 1;
    }
    firmware_set_dead_time_compensation(config.dead_time_compensation);
    if (config.scenario == Scenario::kCurrentStep) {
        firmware_set_mode(FIRMWARE_MODE_CURRENT);
        firmware_set_current_reference_a(current_reference_at(config, 0));
    } else {
        firmware_set_mode(FIRMWARE_MODE_SPEED);
        firmware_set_speed_reference_rad_s(config.speed_reference_rad_s);
    }

    const std::filesystem::path out_path(config.out_path);
    try {
        if (!out_path.parent_path().empty()) {
            std::filesystem::create_directories(out_path.parent_path());
        }
    } catch (const std::exception &error) {
        std::fprintf(stderr, "cannot create %s: %s\n", out_path.string().c_str(), error.what());
        return 1;
    }

    std::FILE *csv = std::fopen(config.out_path, "w");
    if (csv == nullptr) {
        std::perror("fopen");
        return 1;
    }
    std::fprintf(csv, "tick,time_ms,omega_ref_rad_s,omega_rad_s,iq_ref_a,iq_a,id_a,ia_a,ia_plant_a,"
                      "vd_v,vq_v,duty_a,torque_nm,load_nm,bus_v\n");

    double peak_duty = 0.0;
    double peak_current_a = 0.0;
    double peak_speed_rad_s = 0.0;
    double lowest_bus_v = 1.0e9;
    double highest_bus_v = 0.0;

    for (std::uint64_t tick = 0; tick < config.ticks; ++tick) {
        if (config.scenario == Scenario::kCurrentStep) {
            firmware_set_current_reference_a(current_reference_at(config, tick));
        }
        const double load_nm = load_torque_at(config, tick);
        plant->set_external_torque_nm(load_nm);

        sim_run_window(*plant, tick);

        const firmware_status_t *status = firmware_status();
        const tsim::MechanicalState mechanical = plant->mechanical();
        /* The phase current the firmware actually acted on, which is the sample
         * the converter latched at the window centre, quantised by the ADC if
         * the sensing chain is on. The plant's own current is logged beside it
         * because the difference is exactly what the sensing chain costs, and
         * a distortion figure drawn from the wrong one would report the
         * switching ripple as controller behaviour. */
        hal_current_sample_t measured;
        (void)hal_current_read(&measured);
        const double plant_current_a = plant->sample_electrical().ia_a;
        const double bus_v = plant->sample_electrical().bus_v;
        const double omega_reference_rad_s = (config.scenario == Scenario::kCurrentStep)
                                                 ? 0.0
                                                 : (double)config.speed_reference_rad_s;

        tsim::InverterCommand command{};
        sim_hal_take_inverter_command(&command);
        peak_duty = (command.duty_a > peak_duty) ? command.duty_a : peak_duty;
        const double speed = mechanical.omega_rotor_rad_s;
        peak_speed_rad_s = (speed > peak_speed_rad_s) ? speed : peak_speed_rad_s;
        const double phase_current_a = std::fabs(static_cast<double>(measured.ia_a));
        peak_current_a = (phase_current_a > peak_current_a) ? phase_current_a : peak_current_a;
        lowest_bus_v = (bus_v < lowest_bus_v) ? bus_v : lowest_bus_v;
        highest_bus_v = (bus_v > highest_bus_v) ? bus_v : highest_bus_v;

        if (tick % config.log_every == 0) {
            std::fprintf(csv,
                         "%llu,%.4f,%.6g,%.9g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6f,%.6g,%.6g,"
                         "%.6g\n",
                         static_cast<unsigned long long>(tick),
                         static_cast<double>(tick + 1u) * tsim::kControlPeriodS * 1.0e3,
                         omega_reference_rad_s, speed, status->iq_ref_a, status->iq_a, status->id_a,
                         measured.ia_a, plant_current_a, status->vd_v, status->vq_v, command.duty_a,
                         mechanical.torque_motor_nm, load_nm, bus_v);
        }
    }

    std::fclose(csv);

    const tsim::MechanicalState mechanical = plant->mechanical();
    const firmware_status_t *status = firmware_status();
    std::printf("deterministic run, %s\n", scenario_name(config.scenario));
    std::printf("  plant            %s\n", (config.plant == PlantModel::kIdeal)
                                               ? "ideal average voltage source"
                                               : "switching");
    std::printf("  dead time        %.0f ns, compensation %s\n", config.dead_time_ns,
                config.dead_time_compensation ? "on" : "off");
    std::printf("  sensing chain    %s\n", config.imperfections ? "on" : "off");
    std::printf("  ticks            %llu (%.4f s)\n",
                static_cast<unsigned long long>(plant->step_count() / tsim::kStepsPerControl),
                plant->time_s());
    std::printf("  rotor speed      %.6f rad/s\n", mechanical.omega_rotor_rad_s);
    std::printf("  peak speed       %.6f rad/s\n", peak_speed_rad_s);
    std::printf("  final iq         %.6f A (reference %.6f A)\n", status->iq_a, status->iq_ref_a);
    std::printf("  final id         %.6f A\n", status->id_a);
    std::printf("  feedforward      vd %.6f V, vq %.6f V\n", status->vd_ff_v, status->vq_ff_v);
    std::printf("  peak duty        %.6f\n", peak_duty);
    std::printf("  peak phase A     %.6f A\n", peak_current_a);
    std::printf("  bus range        %.4f to %.4f V\n", lowest_bus_v, highest_bus_v);
    /* The energy account is what makes the switching loss assumption visible
     * rather than quoted: it is the difference between what the link delivered
     * and what the windings and the rotor used. */
    const tsim::EnergyAccount energy = plant->energy();
    std::printf("  energy           electrical %.3f J, copper %.3f J, switching %.3f J, "
                "mechanical %.3f J\n",
                energy.electrical_j, energy.copper_j, energy.switching_j, energy.mechanical_j);
    std::printf("  firmware ticks   %u\n", status->ticks);
    std::printf("  faults           0x%08x\n", status->fault_bits);
    std::printf("  telemetry drops  %u\n", sim_hal_telemetry_drops());
    std::printf("  csv              %s\n", config.out_path);
    return 0;
}
