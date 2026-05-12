# Betaflight Control Core

Standalone control-core extraction work for Betaflight-based model-in-the-loop simulation.

The near-term goal is an acro-only Quad X controller that can be called from a host simulator:

```text
pilot sticks + throttle
gyro rates
loop time
configuration
=> motor outputs
```

This repository starts with a small, buildable C interface so we can stabilize the simulator boundary before deciding how much upstream Betaflight code to retain directly.

## Current Scope

- Acro/rate-mode Quad X controller.
- Normalized pilot input for roll, pitch, yaw, and throttle.
- Gyro feedback in degrees per second.
- Per-axis PID state.
- Quad X motor mix using Betaflight mixer signs.
- Example command-line harness.

## Upstream Reference

Initial source mapping was done against `AKlenk13/betaflight` at commit `09bb4fcda`.

Key upstream files are documented in [docs/upstream-control-path.md](docs/upstream-control-path.md).

## Build

```sh
cmake -S . -B build
cmake --build build
./build/bfcore_step_demo
```

## License

This project is intended to remain compatible with Betaflight's GPL licensing. See [LICENSE](LICENSE).
