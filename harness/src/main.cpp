/* Deterministic harness. Six scenarios, all closed loop through the real
 * firmware: a locked rotor current step, a speed step with a load step, a low
 * speed run for the dead time study, a stepped sine sweep for the position
 * loop's frequency response, a contact against a wall with a rendered
 * impedance, and a backlash run.
 *
 * Time is integer, no clock is read, so a repeated run is byte identical on the
 * same compiler and flags.
 *
 * The plant defaults to the switching model, because that is the honest one.
 * `--plant ideal` selects the average voltage source of day 1 and 2, which is
 * how those results are reproduced after day 3. The mechanics defaults to the
 * rigid body of day 1 to day 3, so those results are reproduced without a flag
 * change either; `--load` is how the joint arrives.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "firmware.h"
#include "hal.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"

namespace {

enum class Scenario { kSpeedStep, kCurrentStep, kDeadTime, kSweptSine, kWallContact, kBacklash };

/* Average voltage source, or the inverter that switches. */
enum class PlantModel { kIdeal, kSwitching };

/* What is attached past the shaft. The rigid preset is day 1 to day 3 and stays
 * the default so published results do not move. */
enum class LoadModel { kRigid, kCompliant, kHand, kWall };

/* Which reference path the firmware runs. Current mode is chosen by the current
 * step scenario; everything else is set from the command line. */
enum class ControlMode { kSpeed, kPosition, kImpedance };

struct SweepPoint {
    double frequency_hz = 0.0;
    std::uint64_t start_tick = 0;
    std::uint64_t measure_start_tick = 0;
    std::uint64_t end_tick = 0;
};

struct ScenarioConfig {
    Scenario scenario = Scenario::kSpeedStep;
    PlantModel plant = PlantModel::kSwitching;
    LoadModel load = LoadModel::kRigid;
    ControlMode mode = ControlMode::kSpeed;
    std::uint64_t ticks = 6000;  /* windows, count */
    std::uint64_t log_every = 5; /* windows between CSV rows, count */
    float speed_reference_rad_s = 0.0f;
    float current_reference_a = 0.0f;
    float position_reference_rad = 0.0f;
    float stiffness_nm_per_rad = 1.0f;
    float damping_nm_s_per_rad = 0.05f;
    double wall_angle_rad = 0.05;
    double backlash_rad = 0.0;
    std::uint64_t load_step_tick = 0; /* window the load torque is applied at */
    double load_torque_nm = 0.0;
    bool locked_rotor = false;
    bool imperfections = true;          /* sensing chain on */
    bool dead_time_compensation = true; /* firmware compensation on */
    double dead_time_ns = 500.0;        /* gate dead time in the model, ns */
    double sweep_amplitude_rad = 0.1;
    double coupling_stiffness_nm_per_rad = 0.0;
    bool coupling_stiffness_set = false;
    double sweep_start_hz = 0.5;
    double encoder_noise_deg = 0.05;
    double sweep_stop_hz = 150.0;
    int sweep_points = 13;
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
    config.imperfections = false;
    config.out_path = "data/run_latest/day3_dead_time.csv";
    return config;
}

/* Stepped sine on the position reference, 2 Hz to 150 Hz in thirteen
 * logarithmic steps, with the hand attached and the sensing chain on. Two
 * seconds of settle and four of measurement at low frequency, shrinking with
 * the period; the whole sweep is about ten seconds of simulated time.
 *
 * Stepped sine and not a chirp because each point is then a steady state
 * measurement at a known frequency, and the analysis can fit a sine and a
 * cosine to it with no window and no assumption about how fast the system is
 * allowed to be swept. A chirp is faster and needs the whole toolbox. */
ScenarioConfig swept_sine_config() {
    ScenarioConfig config;
    config.scenario = Scenario::kSweptSine;
    config.load = LoadModel::kHand;
    config.mode = ControlMode::kPosition;
    config.log_every = 1;
    config.sweep_amplitude_rad = 0.1;
    config.sweep_start_hz = 0.5;
    config.sweep_stop_hz = 150.0;
    config.sweep_points = 13;
    config.ticks = 0; /* filled in from the schedule */
    config.out_path = "data/run_latest/day4_sweep.csv";
    return config;
}

/* The joint pushed into a wall by a rendered spring. 0.02 rad for 50 ms, then
 * 0.15 rad against a wall at 0.05 rad, then back off at 250 ms. The reference
 * stays past the wall for long enough to see the contact settle and for the
 * static force to be readable. */
ScenarioConfig wall_contact_config() {
    ScenarioConfig config;
    config.scenario = Scenario::kWallContact;
    config.load = LoadModel::kWall;
    config.mode = ControlMode::kImpedance;
    config.ticks = 6000; /* 300 ms */
    config.log_every = 1;
    config.wall_angle_rad = 0.05;
    config.stiffness_nm_per_rad = 1.0f;
    config.damping_nm_s_per_rad = 0.05f;
    config.out_path = "data/run_latest/day4_wall.csv";
    return config;
}

/* A 1 Hz triangle of plus or minus 0.1 rad with the hand attached, so the joint
 * reverses twice a second and the lost motion shows at every reversal. */
ScenarioConfig backlash_config() {
    ScenarioConfig config;
    config.scenario = Scenario::kBacklash;
    config.load = LoadModel::kHand;
    config.mode = ControlMode::kPosition;
    config.ticks = 12000; /* 600 ms, 1.2 cycles of the triangle */
    config.log_every = 1;
    config.backlash_rad = 0.0;
    config.out_path = "data/run_latest/day4_backlash.csv";
    return config;
}

constexpr std::uint64_t kCurrentStepNegativeAt = 40; /* window, count */
constexpr std::uint64_t kCurrentStepZeroAt = 80;     /* window, count */
constexpr std::uint64_t kWallApproachTick = 1000;    /* 50 ms */
constexpr std::uint64_t kWallReleaseTick = 5000;     /* 250 ms */
constexpr double kBacklashAmplitudeRad = 0.1;
constexpr double kBacklashPeriodS = 1.0;

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

/* Frequency schedule for the sweep. Each point gets two cycles of settle and
 * four of measurement, with a floor of 60 ms of simulated time so that the
 * upper decade is not measured over eight hundred samples of an imperfectly
 * settled transient. Two and four rather than more because the transient at
 * each frequency step is a decay at the damping of the loop, not at the
 * frequency being measured, and it dies well inside two cycles. */
std::vector<SweepPoint> sweep_schedule(const ScenarioConfig &config) {
    std::vector<SweepPoint> schedule;
    std::uint64_t tick = 0;
    const double ratio = (config.sweep_points > 1)
                             ? std::pow(config.sweep_stop_hz / config.sweep_start_hz,
                                        1.0 / static_cast<double>(config.sweep_points - 1))
                             : 1.0;
    for (int i = 0; i < config.sweep_points; ++i) {
        const double frequency_hz = config.sweep_start_hz * std::pow(ratio, static_cast<double>(i));
        const double period_s = 1.0 / frequency_hz;
        const double floor_s = 0.06;
        const std::uint64_t settle_ticks = static_cast<std::uint64_t>(
            std::ceil(std::max(2.0 * period_s, 0.5 * floor_s) / tsim::kControlPeriodS));
        const std::uint64_t measure_ticks = static_cast<std::uint64_t>(
            std::ceil(std::max(4.0 * period_s, 0.5 * floor_s) / tsim::kControlPeriodS));
        SweepPoint point;
        point.frequency_hz = frequency_hz;
        point.start_tick = tick;
        point.measure_start_tick = tick + settle_ticks;
        point.end_tick = point.measure_start_tick + measure_ticks;
        tick = point.end_tick;
        schedule.push_back(point);
    }
    return schedule;
}

struct Reference {
    float position_rad = 0.0f;
    double sweep_hz = 0.0;
    /* One inside the measurement window of a sweep point and zero during its
     * settle. The analysis needs this and should not have to rederive the
     * schedule to get it, which is how two copies of a timing rule drift. */
    int measuring = 0;
};

/* The position reference as a function of time, for the scenarios that drive
 * one. The sweep and the backlash run both start from zero, which is where the
 * harness resets the joint, so the reference is absolute and does not need the
 * angle the mode was entered at. */
Reference position_reference_at(const ScenarioConfig &config, std::uint64_t tick,
                                const std::vector<SweepPoint> &schedule) {
    Reference reference;
    const double time_s = static_cast<double>(tick) * tsim::kControlPeriodS;
    switch (config.scenario) {
    case Scenario::kSweptSine: {
        const auto point =
            std::find_if(schedule.begin(), schedule.end(),
                         [tick](const SweepPoint &candidate) { return tick < candidate.end_tick; });
        if (point != schedule.end()) {
            const double phase =
                tsim::kTwoPi * point->frequency_hz *
                (static_cast<double>(tick - point->start_tick) * tsim::kControlPeriodS);
            reference.position_rad =
                static_cast<float>(config.sweep_amplitude_rad * std::sin(phase));
            reference.sweep_hz = point->frequency_hz;
            reference.measuring = (tick >= point->measure_start_tick) ? 1 : 0;
        }
        break;
    }
    case Scenario::kWallContact: {
        const bool approaching = tick >= kWallApproachTick;
        const bool released = tick >= kWallReleaseTick;
        if (released) {
            reference.position_rad = 0.03f;
        } else if (approaching) {
            reference.position_rad = 0.15f;
        } else {
            reference.position_rad = 0.02f;
        }
        break;
    }
    case Scenario::kBacklash: {
        /* Triangle wave, so the reference reverses at constant speed and the
         * lost motion shows as a flat spot at each reversal rather than as a
         * different amplitude. */
        const double cycles = time_s / kBacklashPeriodS;
        const double fraction = cycles - std::floor(cycles);
        const double triangle = (fraction < 0.5) ? (4.0 * fraction - 1.0) : (3.0 - 4.0 * fraction);
        reference.position_rad = static_cast<float>(kBacklashAmplitudeRad * triangle);
        break;
    }
    default:
        reference.position_rad = config.position_reference_rad;
        break;
    }
    return reference;
}

const char *scenario_name(Scenario scenario) {
    switch (scenario) {
    case Scenario::kSpeedStep:
        return "speed_step";
    case Scenario::kCurrentStep:
        return "current_step";
    case Scenario::kDeadTime:
        return "dead_time";
    case Scenario::kSweptSine:
        return "swept_sine";
    case Scenario::kWallContact:
        return "wall_contact";
    case Scenario::kBacklash:
        return "backlash";
    }
    return "unknown";
}

const char *load_name(LoadModel load) {
    switch (load) {
    case LoadModel::kRigid:
        return "rigid";
    case LoadModel::kCompliant:
        return "compliant";
    case LoadModel::kHand:
        return "hand";
    case LoadModel::kWall:
        return "wall";
    }
    return "unknown";
}

const char *mode_name(ControlMode mode) {
    switch (mode) {
    case ControlMode::kSpeed:
        return "speed";
    case ControlMode::kPosition:
        return "position";
    case ControlMode::kImpedance:
        return "impedance";
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
    std::fprintf(stderr,
                 "usage: %s [--scenario "
                 "speed_step|current_step|dead_time|swept_sine|wall_contact|backlash]\n"
                 "          [--plant ideal|switching] [--load rigid|compliant|hand|wall]\n"
                 "          [--mode speed|position|impedance]\n"
                 "          [--dead-time-ns N] [--compensation on|off] [--imperfections on|off]\n"
                 "          [--backlash-deg X] [--wall-angle-rad X] [--stiffness X] [--damping X]\n"
                 "          [--coupling-stiffness X] [--encoder-noise-deg X]\n"
                 "          [--position-ref-rad X] [--sweep-amplitude-rad X] [--ticks N]\n"
                 "          [--log-every N] [--out PATH]\n",
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
            } else if (std::strcmp(name, "swept_sine") == 0) {
                config = swept_sine_config();
            } else if (std::strcmp(name, "wall_contact") == 0) {
                config = wall_contact_config();
            } else if (std::strcmp(name, "backlash") == 0) {
                config = backlash_config();
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
        } else if (std::strcmp(argv[i], "--load") == 0 && i + 1 < argc) {
            const char *name = argv[++i];
            if (std::strcmp(name, "rigid") == 0) {
                config.load = LoadModel::kRigid;
            } else if (std::strcmp(name, "compliant") == 0) {
                config.load = LoadModel::kCompliant;
            } else if (std::strcmp(name, "hand") == 0) {
                config.load = LoadModel::kHand;
            } else if (std::strcmp(name, "wall") == 0) {
                config.load = LoadModel::kWall;
            } else {
                std::fprintf(stderr, "unknown load: %s\n", name);
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            const char *name = argv[++i];
            if (std::strcmp(name, "speed") == 0) {
                config.mode = ControlMode::kSpeed;
            } else if (std::strcmp(name, "position") == 0) {
                config.mode = ControlMode::kPosition;
            } else if (std::strcmp(name, "impedance") == 0) {
                config.mode = ControlMode::kImpedance;
            } else {
                std::fprintf(stderr, "unknown mode: %s\n", name);
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
        } else if (std::strcmp(argv[i], "--backlash-deg") == 0 && i + 1 < argc) {
            config.backlash_rad = std::strtod(argv[++i], nullptr) * tsim::kTwoPi / 360.0;
        } else if (std::strcmp(argv[i], "--wall-angle-rad") == 0 && i + 1 < argc) {
            config.wall_angle_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--stiffness") == 0 && i + 1 < argc) {
            config.stiffness_nm_per_rad = static_cast<float>(std::strtod(argv[++i], nullptr));
        } else if (std::strcmp(argv[i], "--damping") == 0 && i + 1 < argc) {
            config.damping_nm_s_per_rad = static_cast<float>(std::strtod(argv[++i], nullptr));
        } else if (std::strcmp(argv[i], "--position-ref-rad") == 0 && i + 1 < argc) {
            config.position_reference_rad = static_cast<float>(std::strtod(argv[++i], nullptr));
        } else if (std::strcmp(argv[i], "--sweep-amplitude-rad") == 0 && i + 1 < argc) {
            config.sweep_amplitude_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--encoder-noise-deg") == 0 && i + 1 < argc) {
            config.encoder_noise_deg = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--coupling-stiffness") == 0 && i + 1 < argc) {
            config.coupling_stiffness_nm_per_rad = std::strtod(argv[++i], nullptr);
            config.coupling_stiffness_set = true;
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
    if (config.backlash_rad < 0.0) {
        config.backlash_rad = 0.0;
    }
    return config;
}

tsim::MechanicalParams mechanics_for(const ScenarioConfig &config) {
    tsim::MechanicalParams mechanics;
    switch (config.load) {
    case LoadModel::kRigid:
        mechanics = tsim::mechanics_params_rigid();
        break;
    case LoadModel::kCompliant:
        mechanics = tsim::mechanics_params_compliant_load();
        break;
    case LoadModel::kHand:
        mechanics = tsim::mechanics_params_hand();
        break;
    case LoadModel::kWall:
        mechanics = tsim::mechanics_params_wall(config.wall_angle_rad);
        break;
    }
    if (config.coupling_stiffness_set) {
        mechanics.coupling_stiffness_nm_per_rad = config.coupling_stiffness_nm_per_rad;
    }
    return mechanics;
}

} // namespace

int main(int argc, char **argv) {
    const ScenarioConfig config = parse_options(argc, argv);
    const std::vector<SweepPoint> schedule = sweep_schedule(config);
    const std::uint64_t ticks =
        (config.scenario == Scenario::kSweptSine) ? schedule.back().end_tick : config.ticks;

    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = mechanics_for(config);
    params.mechanics.backlash_rad = config.backlash_rad;

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
    sim_hal_set_encoder_noise_rad(
        static_cast<float>(config.encoder_noise_deg * tsim::kTwoPi / 360.0));
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
    } else if (config.mode == ControlMode::kPosition) {
        firmware_set_mode(FIRMWARE_MODE_POSITION);
        firmware_set_position_reference_rad(
            position_reference_at(config, 0, schedule).position_rad);
    } else if (config.mode == ControlMode::kImpedance) {
        firmware_set_impedance(config.stiffness_nm_per_rad, config.damping_nm_s_per_rad);
        firmware_set_mode(FIRMWARE_MODE_IMPEDANCE);
        firmware_set_position_reference_rad(
            position_reference_at(config, 0, schedule).position_rad);
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
                      "vd_v,vq_v,duty_a,torque_nm,load_nm,bus_v,theta_rad,theta_ref_rad,"
                      "theta_load_rad,omega_load_rad_s,torque_coupling_nm,torque_wall_nm,sweep_hz,"
                      "position_error_rad,sweep_measure\n");

    double peak_duty = 0.0;
    double peak_current_a = 0.0;
    double peak_speed_rad_s = 0.0;
    double lowest_bus_v = 1.0e9;
    double highest_bus_v = 0.0;
    double largest_position_error_rad = 0.0;

    for (std::uint64_t tick = 0; tick < ticks; ++tick) {
        Reference reference = position_reference_at(config, tick, schedule);
        if (config.scenario == Scenario::kCurrentStep) {
            firmware_set_current_reference_a(current_reference_at(config, tick));
        } else if (config.mode == ControlMode::kPosition ||
                   config.mode == ControlMode::kImpedance) {
            firmware_set_position_reference_rad(reference.position_rad);
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
        const double omega_reference_rad_s =
            (config.mode == ControlMode::kSpeed) ? (double)config.speed_reference_rad_s : 0.0;

        tsim::InverterCommand command{};
        sim_hal_take_inverter_command(&command);
        peak_duty = (command.duty_a > peak_duty) ? command.duty_a : peak_duty;
        const double speed = mechanical.omega_rotor_rad_s;
        peak_speed_rad_s = (speed > peak_speed_rad_s) ? speed : peak_speed_rad_s;
        const double phase_current_a = std::fabs(static_cast<double>(measured.ia_a));
        peak_current_a = (phase_current_a > peak_current_a) ? phase_current_a : peak_current_a;
        lowest_bus_v = (bus_v < lowest_bus_v) ? bus_v : lowest_bus_v;
        highest_bus_v = (bus_v > highest_bus_v) ? bus_v : highest_bus_v;
        largest_position_error_rad = std::max(
            largest_position_error_rad, std::fabs(static_cast<double>(status->position_error_rad)));

        if (tick % config.log_every == 0) {
            std::fprintf(csv,
                         "%llu,%.4f,%.6g,%.9g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6f,%.6g,%.6g,"
                         "%.6g,%.9g,%.9g,%.9g,%.9g,%.6g,%.6g,%.6g,%.9g,%d\n",
                         static_cast<unsigned long long>(tick),
                         static_cast<double>(tick + 1u) * tsim::kControlPeriodS * 1.0e3,
                         omega_reference_rad_s, speed, status->iq_ref_a, status->iq_a, status->id_a,
                         measured.ia_a, plant_current_a, status->vd_v, status->vq_v, command.duty_a,
                         mechanical.torque_motor_nm, load_nm, bus_v, mechanical.theta_rotor_rad,
                         reference.position_rad, mechanical.theta_load_rad,
                         mechanical.omega_load_rad_s, mechanical.torque_coupling_nm,
                         mechanical.torque_wall_nm, reference.sweep_hz, status->position_error_rad,
                         reference.measuring);
        }
    }

    std::fclose(csv);

    const tsim::MechanicalState mechanical = plant->mechanical();
    const firmware_status_t *status = firmware_status();
    std::printf("deterministic run, %s\n", scenario_name(config.scenario));
    std::printf("  plant            %s\n", (config.plant == PlantModel::kIdeal)
                                               ? "ideal average voltage source"
                                               : "switching");
    std::printf("  load             %s, backlash %.2f deg, coupling %.1f N m/rad\n",
                load_name(config.load), params.mechanics.backlash_rad * 360.0 / tsim::kTwoPi,
                params.mechanics.coupling_stiffness_nm_per_rad);
    std::printf("  mode             %s\n", mode_name(config.mode));
    std::printf("  commissioned     speed crossover %.1f rad/s, position gain %.1f 1/s\n",
                status->speed_crossover_rad_s, status->position_kp_per_s);
    std::printf("  dead time        %.0f ns, compensation %s\n", config.dead_time_ns,
                config.dead_time_compensation ? "on" : "off");
    std::printf("  sensing chain    %s, encoder noise %.3f deg\n",
                config.imperfections ? "on" : "off", config.encoder_noise_deg);
    std::printf("  ticks            %llu (%.4f s)\n",
                static_cast<unsigned long long>(plant->step_count() / tsim::kStepsPerControl),
                plant->time_s());
    std::printf("  rotor speed      %.6f rad/s\n", mechanical.omega_rotor_rad_s);
    std::printf("  peak speed       %.6f rad/s\n", peak_speed_rad_s);
    std::printf("  rotor angle      %.6f rad\n", mechanical.theta_rotor_rad);
    std::printf("  load angle       %.6f rad\n", mechanical.theta_load_rad);
    std::printf("  position error   %.6f rad peak\n", largest_position_error_rad);
    std::printf("  coupling torque  %.6f N m\n", mechanical.torque_coupling_nm);
    std::printf("  wall torque      %.6f N m\n", mechanical.torque_wall_nm);
    std::printf("  hand torque      %.6f N m\n", mechanical.torque_hand_nm);
    std::printf("  coulomb torque   %.6f N m\n", mechanical.torque_coulomb_nm);
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
    std::printf("  mechanical loss  viscous %.4f J, coulomb %.4f J, stored %.4f J\n",
                energy.viscous_j, energy.coulomb_j, energy.elastic_j);
    std::printf("  firmware ticks   %u\n", status->ticks);
    std::printf("  faults           0x%08x\n", status->fault_bits);
    std::printf("  telemetry drops  %u\n", sim_hal_telemetry_drops());
    std::printf("  csv              %s\n", config.out_path);
    return 0;
}
