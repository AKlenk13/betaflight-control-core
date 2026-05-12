#ifndef BFCORE_CONTROL_CORE_H
#define BFCORE_CONTROL_CORE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BFCORE_AXIS_COUNT 3
#define BFCORE_MOTOR_COUNT_QUADX 4

typedef enum bfcore_axis_e {
    BFCORE_ROLL = 0,
    BFCORE_PITCH = 1,
    BFCORE_YAW = 2,
} bfcore_axis_t;

typedef struct bfcore_pid_gains_s {
    float p;
    float i;
    float d;
    float f;
} bfcore_pid_gains_t;

typedef struct bfcore_config_s {
    bfcore_pid_gains_t pid[BFCORE_AXIS_COUNT];
    float max_rate_dps[BFCORE_AXIS_COUNT];
    float pid_sum_limit;
    float pid_sum_limit_yaw;
    float motor_output_min;
    float motor_output_max;
    int yaw_motors_reversed;
} bfcore_config_t;

typedef struct bfcore_input_s {
    float stick[BFCORE_AXIS_COUNT];
    float throttle;
    float gyro_dps[BFCORE_AXIS_COUNT];
    float dt_seconds;
    int armed;
} bfcore_input_t;

typedef struct bfcore_axis_debug_s {
    float setpoint_dps;
    float error_dps;
    float p;
    float i;
    float d;
    float f;
    float sum;
} bfcore_axis_debug_t;

typedef struct bfcore_output_s {
    float motor[BFCORE_MOTOR_COUNT_QUADX];
    bfcore_axis_debug_t axis[BFCORE_AXIS_COUNT];
} bfcore_output_t;

typedef struct bfcore_state_s {
    float iterm[BFCORE_AXIS_COUNT];
    float previous_gyro_dps[BFCORE_AXIS_COUNT];
    int initialized;
} bfcore_state_t;

void bfcore_default_config(bfcore_config_t *config);
void bfcore_reset(bfcore_state_t *state);
int bfcore_step(
    bfcore_state_t *state,
    const bfcore_config_t *config,
    const bfcore_input_t *input,
    bfcore_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
