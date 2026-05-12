# Upstream Control Path

Mapped from `AKlenk13/betaflight` commit `09bb4fcda`.

## Runtime Loop

The firmware PID loop is coordinated by `src/main/fc/core.c`:

```text
taskMainPidLoop()
  -> subTaskRcCommand()
       -> processRcCommand()
  -> subTaskPidController()
       -> pidController()
  -> subTaskMotorUpdate()
       -> mixTable()
       -> writeMotors()
```

## Critical Upstream Files

- `src/main/fc/core.c`
  - `taskMainPidLoop()`
  - `subTaskRcCommand()`
  - `subTaskPidController()`
  - `subTaskMotorUpdate()`
- `src/main/fc/rc.c`
  - `updateRcCommands()`
  - `processRcCommand()`
  - `applyBetaflightRates()`
  - `applyActualRates()`
- `src/main/flight/pid.c`
  - `pidController()`
  - consumes `getSetpointRate()` and `gyro.gyroADCf[]`
  - produces `pidData[axis].Sum`
- `src/main/flight/pid_init.c`
  - `pidInit()`
  - `pidInitConfig()`
  - `pidInitFilters()`
- `src/main/flight/mixer.c`
  - `mixTable()`
  - `applyMixerAdjustment()`
  - `applyMixToMotors()`
  - `writeMotors()`
- `src/main/flight/mixer_init.c`
  - Quad X mixer coefficients
  - `mixerInit()`
  - `mixerInitProfile()`
- `src/main/sensors/gyro.c`
  - `gyroUpdate()`
  - `gyroFiltering()`
- `src/main/drivers/motor.c`
  - `motorWriteAll()`

## First Extraction Boundary

For the first simulator-facing target, bypass real receiver protocols and motor hardware:

```text
sim input
  -> normalized sticks
  -> gyro rates in deg/s
  -> acro PID and Quad X mixer
  -> captured motor outputs
```

Later extraction passes can replace the simplified implementation with progressively more upstream Betaflight code while preserving the same external API.
