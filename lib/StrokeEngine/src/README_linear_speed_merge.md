# Linear speed merge for PR #239 concept

This package factors the useful part of KinkyMakers/OSSM-hardware PR #239 into the local backlash-safe StrokeEngine refactor.

## What changed

### StrokeEngine speed model

`StrokeEngine::setSpeed()` now accepts **linear actuator speed in mm/s** instead of strokes per minute.

The engine clamps the command to `0..motor.maxSpeed`, converts it to steps/second, then derives `_timeOfStroke` for the existing pattern classes:

```cpp
fullStrokeTime = 3.0f * strokeSteps / linearSpeedStepsPerSecond;
```

That keeps the normal trapezoid patterns from requesting a peak speed above the commanded linear speed.

### Legacy compatibility

A helper remains for old code:

```cpp
Stroker.setSpeedFpm(strokesPerMinute, applyNow);
float fpm = Stroker.getSpeedFpm();
```

But new firmware code should call:

```cpp
Stroker.setSpeed(speedMmPerSecond, applyNow);
```

### Main firmware speed commands

`main.cpp` now treats speed as mm/s all the way through:

```cpp
float speedlimit = min(USER_SPEEDLIMIT, MAX_SPEED);
```

All remote and local speed commands go through the existing smoothing task, now renamed to mm/s:

```cpp
requestSpeedMmPerSecond(speed);
```

The speed task ramps toward the target and calls:

```cpp
Stroker.setSpeed(appliedSpeedMmPerSecond, false);
```

That means speed changes are clamped, slewed, and applied on the next pattern segment rather than rewriting the active motion profile mid-stroke.

## Why this helps the knocking

The old FPM model made high speed inputs collapse `_timeOfStroke` toward the minimum. Patterns then generated unrealistic speed/acceleration values, and StrokeEngine clipped them afterward. That can still cause nasty reversal behavior on a gearbox with backlash.

The new model clamps at the command boundary and derives the pattern timing from a physical linear speed.

## Recommended gearbox-safe values

Start with:

```cpp
#define SPEED_SLEW_MM_PER_SECOND_PER_SECOND 80.0f
#define SPEED_DEADBAND_MM_PER_SECOND 0.5f

Stroker.setMotionAccelerationFactor(0.60f);
Stroker.setReversalAccelerationFactor(0.20f);
Stroker.setEndpointDwellMillis(20);
Stroker.setDefaultStopAcceleration(250.0f);
```

Then tune upward only if the rack/pinion behaves quietly.

## Files included

- `main.cpp`
- `StrokeEngine.h`
- `StrokeEngine.cpp`
- `pattern.h`
- `MotionDriver.h`
- `FastAccelStepperMotionDriver.h`
- `PatternMath.h`



## v2 compile hotfix

Some ESP32/PlatformIO toolchains expose `isfinite()` only as `std::isfinite()`.
This v2 package adds `<cmath>` and uses `std::isfinite()` in `StrokeEngine.cpp` and `main.cpp`.
The change is compile-only; it does not alter the speed-control behavior.
