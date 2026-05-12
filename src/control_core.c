#include "bfcore/control_core.h"
#include "bfcore/rates.h"

#include <math.h>
#include <string.h>

typedef struct motor_mixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motor_mixer_t;

static const motor_mixer_t QUADX_MIXER[BFCORE_MOTOR_COUNT_QUADX] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f,  1.0f },
    { 1.0f,  1.0f,  1.0f,  1.0f },
    { 1.0f,  1.0f, -1.0f, -1.0f },
};

static float constrainf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float constrain_abs(float value, float limit)
{
    return constrainf_local(value, -limit, limit);
}

void bfcore_default_config(bfcore_config_t *config)
{
    if (!config) {
        return;
    }

    memset(config, 0, sizeof(*config));

    bfcore_rates_default_config(&config->rates);

    config->pid[BFCORE_ROLL] = (bfcore_pid_gains_t){ .p = 0.06f, .i = 0.30f, .d = 0.0015f, .f = 0.0f };
    config->pid[BFCORE_PITCH] = (bfcore_pid_gains_t){ .p = 0.06f, .i = 0.30f, .d = 0.0015f, .f = 0.0f };
    config->pid[BFCORE_YAW] = (bfcore_pid_gains_t){ .p = 0.06f, .i = 0.20f, .d = 0.0f, .f = 0.0f };

    config->pid_sum_limit = 500.0f;
    config->pid_sum_limit_yaw = 400.0f;
    config->motor_output_min = 0.0f;
    config->motor_output_max = 1.0f;
    config->yaw_motors_reversed = 0;
}

void bfcore_reset(bfcore_state_t *state)
{
    if (!state) {
        return;
    }

    memset(state, 0, sizeof(*state));
}

int bfcore_step(
    bfcore_state_t *state,
    const bfcore_config_t *config,
    const bfcore_input_t *input,
    bfcore_output_t *output)
{
    if (!state || !config || !input || !output || input->dt_seconds <= 0.0f) {
        return -1;
    }

    memset(output, 0, sizeof(*output));

    if (!state->initialized) {
        for (int axis = 0; axis < BFCORE_AXIS_COUNT; axis++) {
            state->previous_gyro_dps[axis] = input->gyro_dps[axis];
        }
        state->initialized = 1;
    }

    float pid_sum[BFCORE_AXIS_COUNT];

    for (int axis = 0; axis < BFCORE_AXIS_COUNT; axis++) {
        const float stick = constrainf_local(input->stick[axis], -1.0f, 1.0f);
        const float setpoint = bfcore_apply_rates(&config->rates, axis, stick);
        const float gyro = input->gyro_dps[axis];
        const float error = setpoint - gyro;
        const bfcore_pid_gains_t gains = config->pid[axis];

        state->iterm[axis] += gains.i * error * input->dt_seconds;

        const float sum_limit = axis == BFCORE_YAW ? config->pid_sum_limit_yaw : config->pid_sum_limit;
        state->iterm[axis] = constrain_abs(state->iterm[axis], sum_limit);

        const float gyro_delta = (gyro - state->previous_gyro_dps[axis]) / input->dt_seconds;
        const float pterm = gains.p * error;
        const float dterm = -gains.d * gyro_delta;
        const float fterm = gains.f * setpoint;
        const float sum = constrain_abs(pterm + state->iterm[axis] + dterm + fterm, sum_limit);

        output->axis[axis].setpoint_dps = setpoint;
        output->axis[axis].error_dps = error;
        output->axis[axis].p = pterm;
        output->axis[axis].i = state->iterm[axis];
        output->axis[axis].d = dterm;
        output->axis[axis].f = fterm;
        output->axis[axis].sum = sum;

        pid_sum[axis] = sum;
        state->previous_gyro_dps[axis] = gyro;
    }

    if (!input->armed) {
        return 0;
    }

    const float throttle = constrainf_local(input->throttle, 0.0f, 1.0f);
    const float roll = pid_sum[BFCORE_ROLL] / config->pid_sum_limit;
    const float pitch = pid_sum[BFCORE_PITCH] / config->pid_sum_limit;
    float yaw = pid_sum[BFCORE_YAW] / config->pid_sum_limit_yaw;

    if (!config->yaw_motors_reversed) {
        yaw = -yaw;
    }

    float mix[BFCORE_MOTOR_COUNT_QUADX];
    float min_mix = 0.0f;
    float max_mix = 0.0f;

    for (int i = 0; i < BFCORE_MOTOR_COUNT_QUADX; i++) {
        mix[i] = roll * QUADX_MIXER[i].roll + pitch * QUADX_MIXER[i].pitch + yaw * QUADX_MIXER[i].yaw;
        if (mix[i] < min_mix) {
            min_mix = mix[i];
        }
        if (mix[i] > max_mix) {
            max_mix = mix[i];
        }
    }

    const float mix_range = max_mix - min_mix;
    const float normalization = mix_range > 1.0f ? 1.0f / mix_range : 1.0f;
    const float normalized_min = min_mix * normalization;
    const float normalized_max = max_mix * normalization;
    const float adjusted_throttle = constrainf_local(throttle, -normalized_min, 1.0f - normalized_max);
    const float output_range = config->motor_output_max - config->motor_output_min;

    for (int i = 0; i < BFCORE_MOTOR_COUNT_QUADX; i++) {
        const float normalized_motor = adjusted_throttle * QUADX_MIXER[i].throttle + mix[i] * normalization;
        output->motor[i] = config->motor_output_min + output_range * constrainf_local(normalized_motor, 0.0f, 1.0f);
    }

    return 0;
}
