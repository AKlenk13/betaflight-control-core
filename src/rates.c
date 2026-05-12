#include "bfcore/rates.h"

#include <math.h>
#include <string.h>

#define BFCORE_RC_RATE_INCREMENTAL 14.54f

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

static float power3(float value)
{
    return value * value * value;
}

static float power5(float value)
{
    return power3(value) * value * value;
}

void bfcore_rates_default_config(bfcore_rates_config_t *rates)
{
    if (!rates) {
        return;
    }

    memset(rates, 0, sizeof(*rates));
    rates->type = BFCORE_RATES_ACTUAL;

    for (int axis = 0; axis < BFCORE_AXIS_COUNT; axis++) {
        rates->rc_rates[axis] = 7;
        rates->rc_expo[axis] = 0;
        rates->rates[axis] = 67;
        rates->rate_limit[axis] = (unsigned short)BFCORE_SETPOINT_RATE_LIMIT_MAX;
    }
}

static float apply_betaflight_rates(const bfcore_rates_config_t *rates, int axis, float rc_command, float rc_command_abs)
{
    if (rates->rc_expo[axis]) {
        const float expo = rates->rc_expo[axis] / 100.0f;
        rc_command = rc_command * power3(rc_command_abs) * expo + rc_command * (1.0f - expo);
    }

    float rc_rate = rates->rc_rates[axis] / 100.0f;
    if (rc_rate > 2.0f) {
        rc_rate += BFCORE_RC_RATE_INCREMENTAL * (rc_rate - 2.0f);
    }

    float angle_rate = 200.0f * rc_rate * rc_command;
    if (rates->rates[axis]) {
        const float super_rate = rates->rates[axis] / 100.0f;
        const float super_factor = 1.0f / constrainf_local(1.0f - rc_command_abs * super_rate, 0.01f, 1.0f);
        angle_rate *= super_factor;
    }

    return angle_rate;
}

static float apply_raceflight_rates(const bfcore_rates_config_t *rates, int axis, float rc_command, float rc_command_abs)
{
    rc_command = (1.0f + 0.01f * rates->rc_expo[axis] * (rc_command * rc_command - 1.0f)) * rc_command;

    float angle_rate = 10.0f * rates->rc_rates[axis] * rc_command;
    angle_rate *= 1.0f + rc_command_abs * rates->rates[axis] * 0.01f;

    return angle_rate;
}

static float apply_kiss_rates(const bfcore_rates_config_t *rates, int axis, float rc_command, float rc_command_abs)
{
    const float curve = rates->rc_expo[axis] / 100.0f;
    const float super_factor = 1.0f / constrainf_local(1.0f - rc_command_abs * (rates->rates[axis] / 100.0f), 0.01f, 1.0f);
    const float kiss_command = (power3(rc_command) * curve + rc_command * (1.0f - curve)) * (rates->rc_rates[axis] / 1000.0f);

    return constrainf_local(2000.0f * super_factor * kiss_command, BFCORE_SETPOINT_RATE_LIMIT_MIN, BFCORE_SETPOINT_RATE_LIMIT_MAX);
}

static float apply_actual_rates(const bfcore_rates_config_t *rates, int axis, float rc_command, float rc_command_abs)
{
    float expo = rates->rc_expo[axis] / 100.0f;
    expo = rc_command_abs * (power5(rc_command) * expo + rc_command * (1.0f - expo));

    const float center_sensitivity = rates->rc_rates[axis] * 10.0f;
    const float stick_movement = fmaxf(0.0f, rates->rates[axis] * 10.0f - center_sensitivity);

    return rc_command * center_sensitivity + stick_movement * expo;
}

static float apply_quick_rates(const bfcore_rates_config_t *rates, int axis, float rc_command, float rc_command_abs)
{
    const unsigned short rc_rate = rates->rc_rates[axis] * 2;
    if (rc_rate == 0) {
        return 0.0f;
    }

    const unsigned short max_dps = (unsigned short)fmaxf(rates->rates[axis] * 10.0f, rc_rate);
    const float expo = rates->rc_expo[axis] / 100.0f;
    const float super_factor_config = ((float)max_dps / rc_rate - 1.0f) / ((float)max_dps / rc_rate);

    float curve;
    float super_factor;
    float angle_rate;

    if (rates->quick_rates_rc_expo) {
        curve = power3(rc_command) * expo + rc_command * (1.0f - expo);
        super_factor = 1.0f / constrainf_local(1.0f - rc_command_abs * super_factor_config, 0.01f, 1.0f);
        angle_rate = constrainf_local(curve * rc_rate * super_factor, BFCORE_SETPOINT_RATE_LIMIT_MIN, BFCORE_SETPOINT_RATE_LIMIT_MAX);
    } else {
        curve = power3(rc_command_abs) * expo + rc_command_abs * (1.0f - expo);
        super_factor = 1.0f / constrainf_local(1.0f - curve * super_factor_config, 0.01f, 1.0f);
        angle_rate = constrainf_local(rc_command * rc_rate * super_factor, BFCORE_SETPOINT_RATE_LIMIT_MIN, BFCORE_SETPOINT_RATE_LIMIT_MAX);
    }

    return angle_rate;
}

float bfcore_apply_rates(const bfcore_rates_config_t *rates, int axis, float stick)
{
    if (!rates || axis < 0 || axis >= BFCORE_AXIS_COUNT) {
        return 0.0f;
    }

    const float rc_command = constrainf_local(stick, -1.0f, 1.0f);
    const float rc_command_abs = fabsf(rc_command);
    float angle_rate;

    switch (rates->type) {
    case BFCORE_RATES_RACEFLIGHT:
        angle_rate = apply_raceflight_rates(rates, axis, rc_command, rc_command_abs);
        break;
    case BFCORE_RATES_KISS:
        angle_rate = apply_kiss_rates(rates, axis, rc_command, rc_command_abs);
        break;
    case BFCORE_RATES_ACTUAL:
        angle_rate = apply_actual_rates(rates, axis, rc_command, rc_command_abs);
        break;
    case BFCORE_RATES_QUICK:
        angle_rate = apply_quick_rates(rates, axis, rc_command, rc_command_abs);
        break;
    case BFCORE_RATES_BETAFLIGHT:
    default:
        angle_rate = apply_betaflight_rates(rates, axis, rc_command, rc_command_abs);
        break;
    }

    const float rate_limit = rates->rate_limit[axis] > 0
        ? (float)rates->rate_limit[axis]
        : BFCORE_SETPOINT_RATE_LIMIT_MAX;

    return constrainf_local(angle_rate, -rate_limit, rate_limit);
}
