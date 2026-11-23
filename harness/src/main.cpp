/* Deterministic harness. Two scenarios, both closed loop through the real
 * firmware: a locked rotor current step, which measures the current loop
 * against a standstill, and a speed step with a load step, which measures the
 * cascade and its disturbance rejection. Time is integer, no clock is read, so
 * a repeated run is byte identical on the same compiler and flags. */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <string>

#include "firmware.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"

namespace {

enum class Scenario { kSpeedStep, kCurrentStep };

struct ScenarioConfig {
    Scenario scenario = Scenario::kSpeedStep;
    std::uint64_t ticks = 6000;  /* windows, count */
    std::uint64_t log_every = 5; /* windows between CSV rows, count */
    float speed_reference_rad_s = 0.0f;
    float current_reference_a = 0.0f;
    std::uint64_t load_step_tick = 0; /* window the load torque is applied at */
    double load_torque_nm = 0.0;
    bool locked_rotor = false;
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
            } else {
                std::fprintf(stderr, "unknown scenario: %s\n", name);
                std::exit(2);
            }
            scenario_chosen = true;
        } else if (std::strcmp(argv[i], "--ticks") == 0 && i + 1 < argc) {
            config.ticks = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--log-every") == 0 && i + 1 < argc) {
            config.log_every = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            config.out_path = argv[++i];
        } else {
            std::fprintf(stderr,
                         "usage: %s [--scenario speed_step|current_step] [--ticks N] "
                         "[--log-every N] [--out PATH]\n",
                         argv[0]);
            std::exit(2);
        }
    }
    if (!scenario_chosen && config.ticks == 0) {
        config.ticks = 1;
    }
    if (config.log_every == 0) {
        config.log_every = 1;
    }
    return config;
}

} // namespace

int main(int argc, char **argv) {
    const ScenarioConfig config = parse_options(argc, argv);

    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    plant.set_rotor_locked(config.locked_rotor);
    sim_hal_attach(plant);

    firmware_init();
    if (firmware_status()->fault_bits != 0u) {
        std::fprintf(stderr, "firmware latched a fault during boot\n");
        return 1;
    }
    if (config.scenario == Scenario::kSpeedStep) {
        firmware_set_mode(FIRMWARE_MODE_SPEED);
        firmware_set_speed_reference_rad_s(config.speed_reference_rad_s);
    } else {
        firmware_set_mode(FIRMWARE_MODE_CURRENT);
        firmware_set_current_reference_a(current_reference_at(config, 0));
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
    std::fprintf(csv, "tick,time_ms,omega_ref_rad_s,omega_rad_s,iq_ref_a,iq_a,id_a,ia_a,vd_v,vq_v,"
                      "duty_a,torque_nm,load_nm\n");

    double peak_duty = 0.0;
    double peak_current_a = 0.0;
    double peak_speed_rad_s = 0.0;

    for (std::uint64_t tick = 0; tick < config.ticks; ++tick) {
        if (config.scenario == Scenario::kCurrentStep) {
            firmware_set_current_reference_a(current_reference_at(config, tick));
        }
        const double load_nm = load_torque_at(config, tick);
        plant.set_external_torque_nm(load_nm);

        sim_run_window(plant, tick);

        const firmware_status_t *status = firmware_status();
        const tsim::MechanicalState mechanical = plant.mechanical();
        const double current_a = plant.sample_electrical().ia_a;
        const double omega_reference_rad_s =
            (config.scenario == Scenario::kSpeedStep) ? (double)config.speed_reference_rad_s : 0.0;

        tsim::InverterCommand command{};
        sim_hal_take_inverter_command(&command);
        peak_duty = (command.duty_a > peak_duty) ? command.duty_a : peak_duty;
        const double speed = mechanical.omega_rotor_rad_s;
        peak_speed_rad_s = (speed > peak_speed_rad_s) ? speed : peak_speed_rad_s;
        const double phase_current_a = (current_a < 0.0) ? -current_a : current_a;
        peak_current_a = (phase_current_a > peak_current_a) ? phase_current_a : peak_current_a;

        if (tick % config.log_every == 0) {
            std::fprintf(csv, "%llu,%.4f,%.6g,%.9g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6f,%.6g,%.6g\n",
                         static_cast<unsigned long long>(tick),
                         static_cast<double>(tick + 1u) * tsim::kControlPeriodS * 1.0e3,
                         omega_reference_rad_s, speed, status->iq_ref_a, status->iq_a, status->id_a,
                         current_a, status->vd_v, status->vq_v, command.duty_a,
                         mechanical.torque_motor_nm, load_nm);
        }
    }

    std::fclose(csv);

    const tsim::MechanicalState mechanical = plant.mechanical();
    const firmware_status_t *status = firmware_status();
    std::printf("deterministic run, %s\n",
                (config.scenario == Scenario::kSpeedStep) ? "speed_step" : "current_step");
    std::printf("  ticks            %llu (%.4f s)\n",
                static_cast<unsigned long long>(plant.step_count() / tsim::kStepsPerControl),
                plant.time_s());
    std::printf("  rotor speed      %.6f rad/s\n", mechanical.omega_rotor_rad_s);
    std::printf("  peak speed       %.6f rad/s\n", peak_speed_rad_s);
    std::printf("  final iq         %.6f A (reference %.6f A)\n", status->iq_a, status->iq_ref_a);
    std::printf("  peak duty        %.6f\n", peak_duty);
    std::printf("  peak phase A     %.6f A\n", peak_current_a);
    std::printf("  firmware ticks   %u\n", status->ticks);
    std::printf("  faults           0x%08x\n", status->fault_bits);
    std::printf("  telemetry drops  %u\n", sim_hal_telemetry_drops());
    std::printf("  csv              %s\n", config.out_path);
    return 0;
}
