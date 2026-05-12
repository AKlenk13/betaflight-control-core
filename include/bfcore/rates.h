#ifndef BFCORE_RATES_H
#define BFCORE_RATES_H

#include "bfcore/control_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BFCORE_SETPOINT_RATE_LIMIT_MIN (-1998.0f)
#define BFCORE_SETPOINT_RATE_LIMIT_MAX (1998.0f)

void bfcore_rates_default_config(bfcore_rates_config_t *rates);
float bfcore_apply_rates(const bfcore_rates_config_t *rates, int axis, float stick);

#ifdef __cplusplus
}
#endif

#endif
