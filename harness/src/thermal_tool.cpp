/* The thermal network on its own: the derating curve, a steady state for a
 * current, and a soak.
 *
 * Why not through the plant: the housing's time constant is 1020 s, and a 1 MHz
 * plant would have to run a hundred billion steps to soak one. The network is
 * integrated here at a step of its own, with the same class the plant uses, so
 * this is the same arithmetic and not a second model. The plant path is checked
 * separately by the closed loop over temperature run in the harness, which is
 * short because a 20 A stall heats a winding in seconds.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>

#include "tsim/plant_params.h"
#include "tsim/thermal.h"

namespace {

constexpr double kPhaseResistanceOhm = 0.291; /* docs/parameters.md */
constexpr double kContinuousCurrentA = 2.74;  /* datasheet */
constexpr double kWindingLimitC = 155.0;      /* datasheet insulation limit */

void print_usage(const char *program) {
    std::fprintf(stderr,
                 "usage: %s --derating [--out PATH]\n"
                 "       %s --soak --current A [--ambient C] [--seconds S] [--out PATH]\n",
                 program, program);
}

bool make_parent(const char *path) {
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (parent.empty()) {
        return true;
    }
    try {
        std::filesystem::create_directories(parent);
    } catch (const std::exception &error) {
        std::fprintf(stderr, "cannot create %s: %s\n", parent.string().c_str(), error.what());
        return false;
    }
    return true;
}

/* Ambient at which the datasheet continuous current reaches the winding limit,
 * by bisection on the closed form. It is a single number and it says more than
 * the curve does when it lands near the motor's rated ambient maximum. */
double ambient_reaching_limit(const tsim::ThermalParams &params, double current_a) {
    double low_c = 0.0;
    double high_c = kWindingLimitC;
    for (int i = 0; i < 80; ++i) {
        const double middle_c = 0.5 * (low_c + high_c);
        if (tsim::thermal_winding_c_at_current(params, current_a, middle_c, kPhaseResistanceOhm) <
            kWindingLimitC) {
            low_c = middle_c;
        } else {
            high_c = middle_c;
        }
    }
    return 0.5 * (low_c + high_c);
}

} // namespace

int main(int argc, char **argv) {
    bool derating = false;
    bool soak = false;
    double current_a = kContinuousCurrentA;
    double ambient_c = 25.0;
    double seconds = 1200.0;
    double step_s = 0.01;
    const char *out_path = nullptr;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--derating") == 0) {
            derating = true;
        } else if (std::strcmp(argv[i], "--soak") == 0) {
            soak = true;
        } else if (std::strcmp(argv[i], "--current") == 0 && i + 1 < argc) {
            current_a = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--ambient") == 0 && i + 1 < argc) {
            ambient_c = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            seconds = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--step") == 0 && i + 1 < argc) {
            step_s = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }
    if (!(step_s > 0.0) || !(seconds > 0.0)) {
        print_usage(argv[0]);
        return 2;
    }

    tsim::ThermalParams params = tsim::thermal_params_maxon();
    params.enabled = true;
    params.ambient_c = ambient_c;
    params.initial_winding_c = ambient_c;
    params.initial_housing_c = ambient_c;

    if (derating || !soak) {
        const char *path = (out_path != nullptr) ? out_path : "data/run_latest/day5_derating.csv";
        if (!make_parent(path)) {
            return 1;
        }
        std::FILE *csv = std::fopen(path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::fprintf(csv, "ambient_c,limit_current_a,winding_c_at_datasheet_a,runaway_current_a\n");

        std::printf("derating curve, winding limit %.0f degC, %s\n", kWindingLimitC,
                    "constant current, steady state");
        std::printf("  network        Rth %.2f + %.2f K/W, C %.1f + %.1f J/K\n",
                    params.winding_to_housing_k_per_w, params.housing_to_ambient_k_per_w,
                    params.winding_heat_capacity_j_per_k, params.housing_heat_capacity_j_per_k);
        for (int row = 0; row < 9; ++row) {
            const double row_ambient_c = 20.0 + 10.0 * static_cast<double>(row);
            const double limit_a = tsim::thermal_current_for_winding_c(
                params, kWindingLimitC, row_ambient_c, kPhaseResistanceOhm);
            const double at_datasheet_c = tsim::thermal_winding_c_at_current(
                params, kContinuousCurrentA, row_ambient_c, kPhaseResistanceOhm);
            std::fprintf(csv, "%.1f,%.4f,%.4f,%.4f\n", row_ambient_c, limit_a, at_datasheet_c,
                         tsim::thermal_runaway_current(params, kPhaseResistanceOhm));
        }
        std::fclose(csv);
        std::printf(
            "  at 25 degC     %.3f A reaches %.0f degC, and the datasheet %.2f A reaches "
            "%.1f degC\n",
            tsim::thermal_current_for_winding_c(params, kWindingLimitC, 25.0, kPhaseResistanceOhm),
            kWindingLimitC, kContinuousCurrentA,
            tsim::thermal_winding_c_at_current(params, kContinuousCurrentA, 25.0,
                                               kPhaseResistanceOhm));
        std::printf("  datasheet amp  reaches %.0f degC at %.1f degC ambient (rated max 100)\n",
                    kWindingLimitC, ambient_reaching_limit(params, kContinuousCurrentA));
        std::printf("  runaway        above %.3f A no steady state exists at any ambient\n",
                    tsim::thermal_runaway_current(params, kPhaseResistanceOhm));
        std::printf("  wrote %s\n", path);
    }

    if (soak) {
        const char *path =
            (out_path != nullptr) ? out_path : "data/run_latest/day5_thermal_soak.csv";
        if (!make_parent(path)) {
            return 1;
        }
        std::FILE *csv = std::fopen(path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::fprintf(csv, "time_s,winding_c,housing_c,current_a,loss_w\n");

        tsim::ThermalNetwork network;
        network.configure(params);
        network.reset();

        const std::uint64_t steps = static_cast<std::uint64_t>(std::llround(seconds / step_s));
        const std::uint64_t log_every = std::max<std::uint64_t>(steps / 2000u, 1u);
        double time_to_limit_s = -1.0;
        double time_to_derate_s = -1.0;
        for (std::uint64_t step = 1; step <= steps; ++step) {
            const double resistance_ohm =
                kPhaseResistanceOhm * network.copper_resistance_multiplier();
            const double loss_w = tsim::thermal_copper_loss_w(current_a, resistance_ohm);
            network.step(step_s, loss_w);
            const double time_s = static_cast<double>(step) * step_s;
            if (time_to_derate_s < 0.0 && network.winding_temperature_c() >= 100.0) {
                time_to_derate_s = time_s;
            }
            if (time_to_limit_s < 0.0 && network.winding_temperature_c() >= kWindingLimitC) {
                time_to_limit_s = time_s;
            }
            if ((step % log_every) == 0u) {
                std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f\n", time_s,
                             network.winding_temperature_c(), network.housing_temperature_c(),
                             current_a, loss_w);
            }
        }
        std::fclose(csv);
        std::printf("soak at %.3f A from %.1f degC ambient\n", current_a, ambient_c);
        /* The network after the run, and what it is heading for, because the
         * housing's 1020 s constant means an hour is not a soak either. */
        const double steady_c =
            tsim::thermal_winding_c_at_current(params, current_a, ambient_c, kPhaseResistanceOhm);
        std::printf("  after %.0f s     winding %.2f degC, housing %.2f degC\n", seconds,
                    network.winding_temperature_c(), network.housing_temperature_c());
        if (steady_c > 1.0e8) {
            std::printf("  steady state   none, %.3f A is above the runaway current\n", current_a);
        } else {
            std::printf("  steady state   winding %.2f degC\n", steady_c);
        }
        std::printf("  reaches       100 degC at %s, %.0f degC at %s\n",
                    (time_to_derate_s >= 0.0) ? "a finite time" : "never", kWindingLimitC,
                    (time_to_limit_s >= 0.0) ? "a finite time" : "never");
        if (time_to_derate_s >= 0.0) {
            std::printf("  time to 100    %.3f s\n", time_to_derate_s);
        }
        if (time_to_limit_s >= 0.0) {
            std::printf("  time to 155    %.3f s\n", time_to_limit_s);
        }
        std::printf("  wrote %s\n", path);
    }
    return 0;
}
