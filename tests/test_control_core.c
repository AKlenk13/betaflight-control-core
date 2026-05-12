#include "bfcore/control_core.h"

#include <math.h>
#include <stdio.h>

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < 1.0e-5f;
}

static int test_disarmed_outputs_zero(void)
{
    bfcore_config_t config;
    bfcore_state_t state;
    bfcore_output_t output;

    bfcore_default_config(&config);
    bfcore_reset(&state);

    const bfcore_input_t input = {
        .stick = { 1.0f, 1.0f, 1.0f },
        .throttle = 1.0f,
        .gyro_dps = { 0.0f, 0.0f, 0.0f },
        .dt_seconds = 0.001f,
        .armed = 0,
    };

    if (bfcore_step(&state, &config, &input, &output) != 0) {
        return 1;
    }

    for (int i = 0; i < BFCORE_MOTOR_COUNT_QUADX; i++) {
        if (!nearly_equal(output.motor[i], 0.0f)) {
            return 1;
        }
    }

    return 0;
}

static int test_hover_throttle_equal_motors(void)
{
    bfcore_config_t config;
    bfcore_state_t state;
    bfcore_output_t output;

    bfcore_default_config(&config);
    bfcore_reset(&state);

    const bfcore_input_t input = {
        .stick = { 0.0f, 0.0f, 0.0f },
        .throttle = 0.42f,
        .gyro_dps = { 0.0f, 0.0f, 0.0f },
        .dt_seconds = 0.001f,
        .armed = 1,
    };

    if (bfcore_step(&state, &config, &input, &output) != 0) {
        return 1;
    }

    for (int i = 0; i < BFCORE_MOTOR_COUNT_QUADX; i++) {
        if (!nearly_equal(output.motor[i], 0.42f)) {
            return 1;
        }
    }

    return 0;
}

int main(void)
{
    if (test_disarmed_outputs_zero() != 0) {
        fprintf(stderr, "test_disarmed_outputs_zero failed\n");
        return 1;
    }

    if (test_hover_throttle_equal_motors() != 0) {
        fprintf(stderr, "test_hover_throttle_equal_motors failed\n");
        return 1;
    }

    return 0;
}
