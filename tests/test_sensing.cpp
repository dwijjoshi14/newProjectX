/* Tests for the sensing chain. The plant currents are doubles and the encoder
 * has no noise, which is what made day 1 and day 2 clean. A drive never sees
 * that: it sees a shunt, an amplifier, a 12 bit converter, 14 bits of encoder
 * and some noise. These tests pin down what the chain does to a known input so
 * that a later result can be blamed on the controller or on the sensor. */

#include <cmath>

#include "firmware.h"
#include "hal.h"
#include "sim_hal.h"
#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim_test.h"

namespace {

tsim::ElectricalReadout readout(double ia_a, double ib_a, double ic_a, double bus_v) {
    return tsim::ElectricalReadout{ia_a, ib_a, ic_a, 0.0, 0.0, 0.0, bus_v};
}

/* One phase current path: the same conversion the HAL applies, reimplemented
 * here from the parameter table so the test does not simply call the thing it
 * is checking twice. */
double expected_lsb_a(const hal_board_config_t *board) {
    const double lsb_v = static_cast<double>(board->adc_reference_v) /
                         static_cast<double>(board->adc_full_scale_counts);
    return lsb_v / (static_cast<double>(board->shunt_resistance_ohm) *
                    static_cast<double>(board->current_amp_gain_v_per_v));
}

} // namespace

TSIM_TEST(the_converter_quantises_a_current_to_its_least_significant_bit) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    sim_hal_attach(plant);
    sim_hal_set_imperfections(true);

    const double lsb_a = expected_lsb_a(hal_board_config());
    TSIM_CHECK(lsb_a > 0.03);
    TSIM_CHECK(lsb_a < 0.034);

    /* Walk the input across a fifth of an amp in steps a tenth of an LSB wide
     * and look at the output. A converter quantises, so every step in the
     * reading is either nothing or exactly one count, and over this span there
     * has to be a handful of them. Checking where the grid sits instead would
     * only test the amplifier bias, which is not what this is about. */
    double previous_a = 0.0;
    int levels = 0;
    bool on_the_grid = true;
    for (int i = 0; i <= 100; ++i) {
        const double current_a = 1.0 + 0.2 * static_cast<double>(i) / 100.0;
        sim_hal_latch_sample(readout(current_a, -current_a, 0.0, 36.0), 0u);
        hal_current_sample_t sample;
        TSIM_CHECK(hal_current_read(&sample) == HAL_OK);
        TSIM_CHECK(sample.valid);
        if (i > 0) {
            const double step_a = static_cast<double>(sample.ia_a) - previous_a;
            if (std::fabs(step_a) > 1.0e-9) {
                levels += 1;
                if (std::fabs(std::fabs(step_a) - lsb_a) > 1.0e-6) {
                    on_the_grid = false;
                }
            }
        }
        previous_a = sample.ia_a;
    }
    TSIM_CHECK(on_the_grid);
    TSIM_CHECK(levels > 3);
    TSIM_CHECK(levels < 12);

    /* One count of quantisation plus one percent of shunt gain error plus the
     * 1.5 mV amplifier offset, which is 0.06 A on its own. The offset is what
     * the boot calibration exists to remove, and this is the raw chain. */
    sim_hal_latch_sample(readout(1.4, -1.4, 0.0, 36.0), 0u);
    hal_current_sample_t sample;
    TSIM_CHECK(hal_current_read(&sample) == HAL_OK);
    TSIM_CHECK(std::fabs(static_cast<double>(sample.ia_a) - 1.4) < 0.1);

    /* Symmetric about the mid scale bias, so a negative current survives. */
    TSIM_CHECK(sample.ib_a < -1.25);
    TSIM_CHECK(sample.ib_a > -1.5);

    sim_hal_set_imperfections(false);
    sim_hal_latch_sample(readout(1.4, -1.4, 0.0, 36.0), 0u);
    TSIM_CHECK(hal_current_read(&sample) == HAL_OK);
    TSIM_CHECK_NEAR(sample.ia_a, 1.4, 1.0e-6);
}

TSIM_TEST(boot_calibration_removes_the_amplifier_offset) {
    /* A converter that is already running when the bridge is still off is the
     * board sequence, and it is the only way the boot calibration has anything
     * to average. sim_hal_attach clears the latch, so it is published here
     * before firmware_init, the way a free running ADC would. */
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    sim_hal_attach(plant);
    sim_hal_set_imperfections(true);
    sim_hal_latch_sample(readout(0.0, 0.0, 0.0, 36.0), 0u);

    firmware_init();
    TSIM_CHECK(firmware_status()->fault_bits == 0u);

    /* Zero current in, and the offset and bias out. The encoder is at angle
     * zero, so the d axis carries whatever is left. */
    sim_hal_latch_sample(readout(0.0, 0.0, 0.0, 36.0), 0u);
    firmware_tick();
    TSIM_CHECK(std::fabs(static_cast<double>(firmware_status()->id_a)) < 0.02);
    TSIM_CHECK(std::fabs(static_cast<double>(firmware_status()->iq_a)) < 0.02);

    /* 1 A on phase A at electrical angle zero is all d axis. The residual is
     * the quantisation of the converter, not the offset, which was removed. */
    sim_hal_latch_sample(readout(1.0, -0.5, -0.5, 36.0), 0u);
    firmware_tick();
    TSIM_CHECK_NEAR(firmware_status()->id_a, 1.0, 0.03);
    TSIM_CHECK(std::fabs(static_cast<double>(firmware_status()->iq_a)) < 0.03);
}

TSIM_TEST(the_encoder_quantises_and_lags_by_its_stated_latency) {
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::IdealPlant plant(params);
    plant.reset(0.0);
    sim_hal_attach(plant);
    sim_hal_set_imperfections(true);

    /* Spin the shaft with a constant external torque until the encoder latency
     * is worth more than the noise. At 200 rad/s the stated 1.5 us is 0.8
     * counts, which is below the floor, so the lag can only be measured at a
     * speed this joint will never see. That is a fact about the encoder worth
     * having: the latency does not matter at this resolution. */
    plant.set_external_torque_nm(-0.02);
    /* Viscous friction caps this at 0.02 / 1e-5 = 2000 rad/s, and the inertia
     * takes 0.23 s to get there, so 0.25 s lands comfortably above 1000. */
    for (int i = 0; i < 250000; ++i) {
        plant.step_one();
    }
    const tsim::MechanicalState mechanical = plant.mechanical();
    TSIM_CHECK(mechanical.omega_rotor_rad_s > 1000.0);

    const hal_board_config_t *board = hal_board_config();
    const double counts_per_rad = static_cast<double>(board->encoder_counts_per_rev) / tsim::kTwoPi;

    double lagged_error = 0.0;
    double unlagged_error = 0.0;
    for (int i = 0; i < 64; ++i) {
        hal_encoder_sample_t sample;
        TSIM_CHECK(hal_encoder_read(&sample) == HAL_OK);
        TSIM_CHECK(static_cast<double>(sample.position_counts) ==
                   std::round(static_cast<double>(sample.position_counts)));
        const tsim::MechanicalState now = plant.mechanical();
        lagged_error += static_cast<double>(sample.position_counts) -
                        (now.theta_rotor_rad - now.omega_rotor_rad_s * 1.5e-6) * counts_per_rad;
        unlagged_error +=
            static_cast<double>(sample.position_counts) - now.theta_rotor_rad * counts_per_rad;
        /* The rotor moves between reads, so the noise draw is not the same
         * twice; averaging 64 of them brings the mean down to a count or so. */
        for (int step = 0; step < 100; ++step) {
            plant.step_one();
        }
    }
    lagged_error /= 64.0;
    unlagged_error /= 64.0;
    TSIM_CHECK(std::fabs(lagged_error) < 2.0);
    TSIM_CHECK(std::fabs(lagged_error) < std::fabs(unlagged_error) * 0.5);
}

TSIM_TEST(the_encoder_noise_is_the_same_on_a_rerun) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.75);
    sim_hal_attach(plant);
    sim_hal_set_imperfections(true);
    hal_encoder_sample_t first[8];
    hal_current_sample_t current;
    for (int i = 0; i < 8; ++i) {
        TSIM_CHECK(hal_encoder_read(&first[i]) == HAL_OK);
        TSIM_CHECK(hal_current_read(&current) == HAL_OK);
    }

    plant.reset(0.75);
    sim_hal_attach(plant);
    sim_hal_set_imperfections(true);
    for (int i = 0; i < 8; ++i) {
        hal_encoder_sample_t again;
        TSIM_CHECK(hal_encoder_read(&again) == HAL_OK);
        TSIM_CHECK(again.position_counts == first[i].position_counts);
    }
}
