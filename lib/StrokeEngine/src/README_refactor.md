# StrokeEngine Phase 1 + Phase 2 Refactor

This package contains a practical refactor of the uploaded StrokeEngine code.

## Files

- `StrokeEngine.h`
- `StrokeEngine.cpp`
- `MotionDriver.h`
- `FastAccelStepperMotionDriver.h`
- `pattern.h`
- `PatternMath.h`

## Phase 1: smoother stopping

Added:

```cpp
void requestStop(float decelerationMmPerSecondSquared = 0.0f, bool emergency = false);
void stopMotion(float decelerationMmPerSecondSquared = 0.0f, bool emergency = false);
void setDefaultStopAcceleration(float decelerationMmPerSecondSquared);
float getDefaultStopAcceleration();
```

Behavior:

- `requestStop()` is non-blocking and is intended for UI/user stop actions while a pattern is running.
- `stopMotion()` remains blocking so existing calls before homing/manual moves stay safe.
- A new `STOPPING` state was added.
- Smooth stop uses configurable deceleration instead of always slamming max acceleration.
- Blocking stop loops now call `vTaskDelay()` instead of spinning forever like a tiny heater.

## Phase 2: driver abstraction

Added:

```cpp
MotionDriver.h
FastAccelStepperMotionDriver.h
```

`StrokeEngine` now stores:

```cpp
MotionDriver *_driver;
```

instead of directly using a global `FastAccelStepper *servo`.

Default usage still works:

```cpp
StrokeEngine engine;
engine.begin(&geometry, &motor);
```

To inject another driver later:

```cpp
MyOtherMotionDriver driver;
StrokeEngine engine;
engine.begin(&geometry, &motor, &driver);
```

## Small fixes included

- Fixed `getPatternName()` off-by-one bounds check.
- Fixed `setMaxSpeed()` and `setMaxAcceleration()` so they use their function arguments.
- Fixed a likely stale-speed bug in `Insist::_updateStrokeTiming()`.
- Fixed several telemetry casts to avoid precision loss.
- Fixed setup-depth speed telemetry conversion from milliHz.

## Integration notes

Put all headers where your Arduino/PlatformIO include path can find them. If your project uses `src/` and `include/`, a typical layout is:

```text
include/StrokeEngine.h
include/MotionDriver.h
include/FastAccelStepperMotionDriver.h
include/pattern.h
include/PatternMath.h
src/StrokeEngine.cpp
```

The new abstraction keeps FastAccelStepper as the default backend, so you do not have to change your sketch/application code unless you want to inject a different driver.

## Compatibility hotfix

Some FastAccelStepper versions do not expose `stepsToStop()`. The adapter now returns `0` from `stepsToStop()` so StrokeEngine knows predictive stop-distance checking is unavailable. Smooth stopping still works through `setAcceleration()`, `applySpeedAcceleration()`, and `stopMove()`.

The sensorless-homing debug timestamp is now only compiled when `DEBUG_TALKATIVE` is enabled, avoiding the unused-variable warning in normal builds.

## Backlash smoothing additions

This version also includes gearbox/rack backlash smoothing controls:

```cpp
strokeEngine.setMotionAccelerationFactor(0.75f);
strokeEngine.setReversalAccelerationFactor(0.25f);
strokeEngine.setEndpointDwellMillis(10);
strokeEngine.setDefaultStopAcceleration(300.0f);
```

- `setMotionAccelerationFactor()` globally softens pattern acceleration.
- `setReversalAccelerationFactor()` additionally softens direction changes.
- `setEndpointDwellMillis()` adds a short pause between completed pattern moves.

For a NEMA 24 with a 20:1 gearbox on rack and pinion, start with 0.75, 0.25, and 10 ms. If the knock persists, try 0.60, 0.20, and 20 ms.
