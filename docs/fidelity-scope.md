# Fidelity Scope

This project keeps the simulator-facing API small while progressively replacing simplified internals with Betaflight-faithful behavior.

## Must Match Betaflight

### Rate Logic

Status: implemented for the standalone core.

The `bfcore_apply_rates()` implementation mirrors the upstream formulas from `src/main/fc/rc.c` for:

- Betaflight rates
- RaceFlight rates
- KISS rates
- Actual rates
- Quick rates

The API takes normalized stick input in `[-1, 1]` and returns an angular-rate setpoint in degrees per second. This corresponds to Betaflight after raw receiver values have been converted into `rcCommand` deflection.

The current implementation does not yet include RC packet timing, RC smoothing, feedforward smoothing, or throttle lookup generation. Those belong to the larger RC/setpoint pipeline and should be added without changing the public `bfcore_step()` API.

### Full PID Behavior

Status: planned.

The current PID loop is still a small placeholder. The target is to replace it with Betaflight-faithful runtime behavior from:

- `src/main/flight/pid.c`
- `src/main/flight/pid_init.c`
- required filter/math helpers from `src/main/common`

The implementation should preserve the current simulator boundary:

```text
setpoint rates + gyro rates + dt + config
=> PID terms and axis sums
```

Features that must be represented when the PID import is complete include D-term filtering, I-term relax, anti-gravity, TPA, D-max, feedforward, integrated yaw, absolute control, and PID sum limiting. Features can be disabled by config, but their enabled behavior should match upstream Betaflight.

### Gyro / Filtering Pipeline

Status: planned.

The core must support both paths:

```text
raw gyro samples -> Betaflight filter pipeline -> PID
```

and:

```text
already-filtered gyro rates -> PID
```

The bypass path is important for Simulink and model-in-the-loop use cases where the plant or estimator already provides filtered angular rates.

## Set Aside For Now

The following are intentionally out of scope for the current phase:

- Real receiver protocols
- Arming/failsafe state machine
- Angle/horizon modes and accelerometer attitude logic
- GPS/autopilot/altitude hold
- Hardware motor drivers
- Non-Quad-X mixers
- CLI/MSP/OSD/telemetry/blackbox
- Real scheduler and target hardware abstraction

## Integration Rule

When replacing placeholder code with upstream behavior, prefer small modules with tests over broad source drops. Each imported behavior should document:

- upstream file/function reference
- simulator-facing inputs and outputs
- deliberately stubbed dependencies
- test cases that lock the behavior down
