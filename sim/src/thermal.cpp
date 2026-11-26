#include "tsim/thermal.h"

#include <cmath>

namespace tsim {
namespace {

/* The value returned when the model has no steady state. A number large enough
 * that no comparison against a limit can be satisfied by accident, rather than
 * a quiet clamp that would present a runaway as a settled winding. */
constexpr double kNoSteadyStateC = 1.0e9;

} // namespace

double thermal_copper_multiplier(const ThermalParams &params, double winding_c) {
    if (!(params.copper_alpha_per_k > 0.0)) {
        return 1.0;
    }
    return 1.0 + params.copper_alpha_per_k * (winding_c - params.copper_reference_c);
}

double thermal_steady_state_winding_c(const ThermalParams &params, double copper_loss_w) {
    return params.ambient_c +
           copper_loss_w * (params.winding_to_housing_k_per_w + params.housing_to_ambient_k_per_w);
}

double thermal_copper_loss_w(double current_a, double phase_resistance_ohm) {
    return 1.5 * current_a * current_a * phase_resistance_ohm;
}

double thermal_winding_c_at_current(const ThermalParams &params, double current_a, double ambient_c,
                                    double phase_resistance_ohm) {
    /* Loss(T) = A (1 + alpha (T - 20)) with A = 1.5 I^2 R, dissipation is
     * (T - Ta) / Rsum. Setting them equal and collecting T gives a linear
     * equation, so there is no need to iterate. */
    const double loss_at_reference_w = thermal_copper_loss_w(current_a, phase_resistance_ohm);
    const double resistance_sum_k_per_w =
        params.winding_to_housing_k_per_w + params.housing_to_ambient_k_per_w;
    if (!(resistance_sum_k_per_w > 0.0) || !(params.copper_alpha_per_k > 0.0)) {
        return thermal_steady_state_winding_c(params, loss_at_reference_w);
    }
    const double slope_w_per_k =
        loss_at_reference_w * params.copper_alpha_per_k - 1.0 / resistance_sum_k_per_w;
    if (!(slope_w_per_k < 0.0)) {
        return kNoSteadyStateC;
    }
    const double numerator_w =
        params.copper_alpha_per_k * params.copper_reference_c * loss_at_reference_w -
        loss_at_reference_w - ambient_c / resistance_sum_k_per_w;
    const double winding_c = numerator_w / slope_w_per_k;
    return (winding_c > ambient_c) ? winding_c : ambient_c;
}

double thermal_current_for_winding_c(const ThermalParams &params, double winding_limit_c,
                                     double ambient_c, double phase_resistance_ohm) {
    const double resistance_sum_k_per_w =
        params.winding_to_housing_k_per_w + params.housing_to_ambient_k_per_w;
    const double hot_resistance_ohm =
        phase_resistance_ohm * thermal_copper_multiplier(params, winding_limit_c);
    const double loss_w = (winding_limit_c - ambient_c) / resistance_sum_k_per_w;
    if (!(loss_w > 0.0) || !(hot_resistance_ohm > 0.0)) {
        return 0.0;
    }
    return std::sqrt(loss_w / (1.5 * hot_resistance_ohm));
}

double thermal_runaway_current(const ThermalParams &params, double phase_resistance_ohm) {
    const double resistance_sum_k_per_w =
        params.winding_to_housing_k_per_w + params.housing_to_ambient_k_per_w;
    const double denominator =
        resistance_sum_k_per_w * 1.5 * phase_resistance_ohm * params.copper_alpha_per_k;
    if (!(denominator > 0.0)) {
        return kNoSteadyStateC;
    }
    return std::sqrt(1.0 / denominator);
}

ThermalNetwork::ThermalNetwork() {
    ThermalParams params{};
    params.ambient_c = 25.0;
    params.initial_winding_c = 25.0;
    params.initial_housing_c = 25.0;
    configure(params);
}

void ThermalNetwork::configure(const ThermalParams &params) {
    params_ = params;
    reset();
}

void ThermalNetwork::reset() {
    winding_c_ = params_.initial_winding_c;
    housing_c_ = params_.initial_housing_c;
}

void ThermalNetwork::step(double dt_s, double copper_loss_w) {
    if (!params_.enabled) {
        return;
    }
    if (!(params_.winding_to_housing_k_per_w > 0.0) ||
        !(params_.housing_to_ambient_k_per_w > 0.0) ||
        !(params_.winding_heat_capacity_j_per_k > 0.0) ||
        !(params_.housing_heat_capacity_j_per_k > 0.0)) {
        return;
    }
    const double to_housing_w = (winding_c_ - housing_c_) / params_.winding_to_housing_k_per_w;
    const double to_ambient_w =
        (housing_c_ - params_.ambient_c) / params_.housing_to_ambient_k_per_w;
    winding_c_ += (copper_loss_w - to_housing_w) / params_.winding_heat_capacity_j_per_k * dt_s;
    housing_c_ += (to_housing_w - to_ambient_w) / params_.housing_heat_capacity_j_per_k * dt_s;
}

double ThermalNetwork::winding_temperature_c() const {
    return winding_c_;
}

double ThermalNetwork::housing_temperature_c() const {
    return housing_c_;
}

double ThermalNetwork::copper_resistance_multiplier() const {
    if (!params_.enabled) {
        return 1.0;
    }
    return thermal_copper_multiplier(params_, winding_c_);
}

double ThermalNetwork::winding_time_constant_s() const {
    return params_.winding_to_housing_k_per_w * params_.winding_heat_capacity_j_per_k;
}

double ThermalNetwork::housing_time_constant_s() const {
    return params_.housing_to_ambient_k_per_w * params_.housing_heat_capacity_j_per_k;
}

const ThermalParams &ThermalNetwork::params() const {
    return params_;
}

} // namespace tsim
