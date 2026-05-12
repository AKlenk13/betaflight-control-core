#include "bfcore/control_core.h"

#include <stdio.h>

int main(void)
{
    bfcore_config_t config;
    bfcore_state_t state;
    bfcore_output_t output;

    bfcore_default_config(&config);
    bfcore_reset(&state);

    const bfcore_input_t input = {
        .stick = { 0.15f, -0.05f, 0.10f },
        .throttle = 0.35f,
        .gyro_dps = { 18.0f, -6.0f, 4.0f },
        .dt_seconds = 0.000125f,
        .armed = 1,
    };

    if (bfcore_step(&state, &config, &input, &output) != 0) {
        fprintf(stderr, "bfcore_step failed\n");
        return 1;
    }

    printf("motors:");
    for (int i = 0; i < BFCORE_MOTOR_COUNT_QUADX; i++) {
        printf(" %.6f", output.motor[i]);
    }
    printf("\n");

    for (int axis = 0; axis < BFCORE_AXIS_COUNT; axis++) {
        printf("axis %d setpoint %.3f gyro_error %.3f pid_sum %.3f\n",
            axis,
            output.axis[axis].setpoint_dps,
            output.axis[axis].error_dps,
            output.axis[axis].sum);
    }

    return 0;
}
