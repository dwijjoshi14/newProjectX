/* Efficiency over a torque and speed grid, measured from the plant's own energy
 * account rather than from a formula.
 *
 * Why a grid and not one operating point: day 3 reported 92 percent on a 200
 * rad/s speed step and said not to quote it, because one point is not an
 * efficiency. A map is, as long as the temperature it was taken at is stated.
 *
 * The winding temperature is held at a fixed value by scaling the winding
 * resistance before the run, not by running the thermal network. That is a
 * deliberate simplification and it is stated in the output: a map is a property
 * of a hot motor, a hot motor is at whatever temperature its own losses put it
 * at, and closing that loop for every grid point would mean soaking each one for
 * 1020 s. What is measured here is the electrical to mechanical conversion at a
 * stated temperature, with the copper loss rising as it should.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <vector>

#include "firmware.h"
#include "hal.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"
#include "tsim/thermal.h"

namespace {

constexpr std::uint64_t kSettleTicks = 2000;  /* 100 ms, ten speed loop times */
constexpr std::uint64_t kMeasureTicks = 2000; /* 100 ms of measured energy */
constexpr double kEncoderNoiseRad = 8.7266e-4;

struct Point {
    double speed_rad_s;
    double torque_nm;
};

struct PointResult {
    double efficiency = 0.0;
    double electrical_w = 0.0;
    double mechanical_w = 0.0;
    double copper_w = 0.0;
    double switching_w = 0.0;
    double friction_w = 0.0;
    /* Electrical in minus mechanical out minus the two losses the account
     * separates. It should be a small positive number (device conduction and
     * dead time); when it is negative the model is returning energy it never
     * received and the row is not a measurement of anything. At low current it
     * goes negative because the link current is resolved on a 1 us grid against
     * a 1.76 A ripple, and that is reported rather than hidden. */
    double unaccounted_w = 0.0;
    double iq_a = 0.0;
    double speed_rad_s = 0.0;
};

PointResult run_point(tsim::IPlant &plant, const Point &point) {
    firmware_set_speed_reference_rad_s(static_cast<float>(point.speed_rad_s));
    plant.set_external_torque_nm(point.torque_nm);

    for (std::uint64_t tick = 0; tick < kSettleTicks; ++tick) {
        sim_run_window(plant, tick);
    }
    const tsim::EnergyAccount before = plant.energy();

    double iq_sum_a = 0.0;
    for (std::uint64_t tick = 0; tick < kMeasureTicks; ++tick) {
        sim_run_window(plant, kSettleTicks + tick);
        iq_sum_a += std::fabs(static_cast<double>(firmware_status()->iq_a));
    }
    const tsim::EnergyAccount after = plant.energy();

    const double seconds = static_cast<double>(kMeasureTicks) * tsim::kControlPeriodS;
    PointResult result;
    result.electrical_w = (after.electrical_j - before.electrical_j) / seconds;
    result.mechanical_w = (after.mechanical_j - before.mechanical_j) / seconds;
    result.copper_w = (after.copper_j - before.copper_j) / seconds;
    result.switching_w = (after.switching_j - before.switching_j) / seconds;
    result.friction_w =
        ((after.viscous_j - before.viscous_j) + (after.coulomb_j - before.coulomb_j)) / seconds;
    result.unaccounted_w =
        result.electrical_w - result.mechanical_w - result.copper_w - result.switching_w;
    result.iq_a = iq_sum_a / static_cast<double>(kMeasureTicks);
    result.speed_rad_s = plant.mechanical().omega_rotor_rad_s;
    /* The efficiency is the shaft work over the link energy. It is below one for
     * every operating point including the ones where the inverter is nearly
     * free, because the copper and the drag are in the same account. */
    result.efficiency =
        (result.electrical_w > 1.0e-6) ? (result.mechanical_w / result.electrical_w) : 0.0;
    return result;
}

std::vector<double> parse_list(const char *text) {
    std::vector<double> values;
    const char *cursor = text;
    while (*cursor != '\0') {
        char *end = nullptr;
        const double value = std::strtod(cursor, &end);
        if (end == cursor) {
            break;
        }
        values.push_back(value);
        cursor = end;
        while (*cursor == ',' || *cursor == ' ') {
            ++cursor;
        }
    }
    return values;
}

void print_usage(const char *program) {
    std::fprintf(stderr,
                 "usage: %s [--winding-temperature-c X] [--speeds X,X,...] "
                 "[--torques X,X,...] [--out PATH]\n",
                 program);
}

} // namespace

int main(int argc, char **argv) {
    double winding_c = 100.0;
    std::vector<double> speeds{20.0, 50.0, 100.0, 200.0, 300.0, 400.0};
    /* 0.126 N m is the continuous torque rating, 2.74 A times 46.1 mNm/A. The
     * grid stops below it because the speed loop's own output limit is the
     * continuous current: a point above that is not a point the drive can hold,
     * and the first version of this grid had 0.13 in it and measured the joint
     * being back driven. */
    std::vector<double> torques{0.02, 0.05, 0.08, 0.10, 0.12};
    const char *out_path = "data/run_latest/day5_efficiency.csv";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--winding-temperature-c") == 0 && i + 1 < argc) {
            winding_c = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--speeds") == 0 && i + 1 < argc) {
            speeds = parse_list(argv[++i]);
        } else if (std::strcmp(argv[i], "--torques") == 0 && i + 1 < argc) {
            torques = parse_list(argv[++i]);
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    /* The winding at the stated temperature, with the network left off: the
     * thermal model is a different study and a map that ran it would take a
     * quarter of an hour per row. */
    tsim::ThermalParams thermal = tsim::thermal_params_maxon();
    const double multiplier = tsim::thermal_copper_multiplier(thermal, winding_c);
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_rigid();
    params.resistance_phase_ohm *= multiplier;

    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    plant.reset(0.0);
    sim_hal_attach(plant);
    sim_hal_set_board_dead_time_s(5.0e-7f);
    sim_hal_set_encoder_noise_rad(static_cast<float>(kEncoderNoiseRad));
    sim_hal_set_imperfections(true);
    sim_hal_latch_sample(plant.sample_electrical(), 0u);

    firmware_init();
    firmware_set_mode(FIRMWARE_MODE_SPEED);

    const std::filesystem::path path(out_path);
    try {
        if (!path.parent_path().empty()) {
            std::filesystem::create_directories(path.parent_path());
        }
    } catch (const std::exception &error) {
        std::fprintf(stderr, "cannot create %s: %s\n", path.string().c_str(), error.what());
        return 1;
    }
    std::FILE *csv = std::fopen(out_path, "w");
    if (csv == nullptr) {
        std::perror("fopen");
        return 1;
    }
    std::fprintf(csv, "speed_rad_s,torque_nm,efficiency,electrical_w,mechanical_w,copper_w,"
                      "switching_w,friction_w,unaccounted_w,iq_a,measured_speed_rad_s,"
                      "winding_c\n");

    std::printf("efficiency map, switching plant, %s\n",
                "winding held at a stated temperature, not soaked");
    std::printf("  winding        %.1f degC, resistance x %.4f (%.4f ohm per phase)\n", winding_c,
                multiplier, params.resistance_phase_ohm);
    std::printf("  commissioned   speed crossover %.1f rad/s\n",
                firmware_status()->speed_crossover_rad_s);

    for (const double speed : speeds) {
        for (const double torque : torques) {
            const Point point{speed, torque};
            const PointResult result = run_point(plant, point);
            std::fprintf(csv, "%.3f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f\n",
                         speed, torque, result.efficiency, result.electrical_w, result.mechanical_w,
                         result.copper_w, result.switching_w, result.friction_w,
                         result.unaccounted_w, result.iq_a, result.speed_rad_s, winding_c);
            std::printf("  %5.0f rad/s  %6.3f N m  %.4f  in %7.3f W  out %7.3f W  "
                        "copper %6.3f W  unaccounted %+7.3f W  iq %5.3f A\n",
                        speed, torque, result.efficiency, result.electrical_w, result.mechanical_w,
                        result.copper_w, result.unaccounted_w, result.iq_a);
        }
    }
    std::fclose(csv);
    std::printf("  wrote %s\n", out_path);
    return 0;
}
