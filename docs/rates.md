# Betaflight Rate Logic

The rate module converts normalized stick deflection into angular-rate setpoints in degrees per second.

```c
float setpoint = bfcore_apply_rates(&config.rates, BFCORE_ROLL, stick);
```

Input `stick` is constrained to `[-1, 1]`.

## Supported Rate Types

`bfcore_rates_type_t` maps to the upstream Betaflight `ratesType_e` choices:

| bfcore type | Upstream behavior |
| --- | --- |
| `BFCORE_RATES_BETAFLIGHT` | `applyBetaflightRates()` |
| `BFCORE_RATES_RACEFLIGHT` | `applyRaceFlightRates()` |
| `BFCORE_RATES_KISS` | `applyKissRates()` |
| `BFCORE_RATES_ACTUAL` | `applyActualRates()` |
| `BFCORE_RATES_QUICK` | `applyQuickRates()` |

The formulas are ported from `AKlenk13/betaflight` commit `09bb4fcda`, file `src/main/fc/rc.c`.

## Config Fields

- `rc_rates[axis]`: upstream `rcRates[axis]`
- `rc_expo[axis]`: upstream `rcExpo[axis]`
- `rates[axis]`: upstream `rates[axis]`
- `rate_limit[axis]`: upstream `rate_limit[axis]`
- `quick_rates_rc_expo`: upstream `quickRatesRcExpo`

## Current Boundary

This module starts after receiver normalization. It does not yet model:

- `rcData[]` PWM/channel conversion
- deadband
- yaw reversal
- RC smoothing
- feedforward smoothing
- FPV camera angle mixing

Those pieces can be added later as a setpoint pipeline in front of `bfcore_apply_rates()`.
