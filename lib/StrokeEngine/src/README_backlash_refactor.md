# StrokeEngine backlash smoothing refactor

This package builds on the Phase 1 + Phase 2 refactor and adds tuning for geared rack-and-pinion systems where backlash causes knocking at stroke reversal.

## New controls

### 1. Global pattern acceleration factor

Softens every pattern move without changing the machine's configured hard safety limit.

```cpp
strokeEngine.setMotionAccelerationFactor(0.75f);
```

Recommended starting values:

```text
1.00 = original behavior
0.75 = mild softening
0.60 = noticeable softening
0.50 = strong softening
```

### 2. Reversal acceleration factor

Applies an additional acceleration reduction only when the next commanded move reverses direction.

```cpp
strokeEngine.setReversalAccelerationFactor(0.25f);
```

Recommended starting values for geared rack and pinion:

```text
0.20 = very soft reversal
0.25 = good first test value for backlash
0.35 = default in this package
0.50 = moderate
1.00 = original behavior
```

### 3. Endpoint dwell

Adds a small pause after each completed pattern move before the next move is commanded. This lets gearbox backlash and rack/pinion compliance settle before reversal.

```cpp
strokeEngine.setEndpointDwellMillis(10);
```

Recommended starting values:

```text
0 ms  = disabled
10 ms = default in this package
20 ms = stronger settling
50 ms = visible pause
```

## Suggested initial tuning for a NEMA 24 + 20:1 gearbox + rack/pinion

```cpp
strokeEngine.setMotionAccelerationFactor(0.75f);
strokeEngine.setReversalAccelerationFactor(0.25f);
strokeEngine.setEndpointDwellMillis(10);
strokeEngine.setDefaultStopAcceleration(300.0f); // mm/s^2
```

If the knock is still obvious, try:

```cpp
strokeEngine.setMotionAccelerationFactor(0.60f);
strokeEngine.setReversalAccelerationFactor(0.20f);
strokeEngine.setEndpointDwellMillis(20);
```

## Changed files

- `StrokeEngine.h`
- `StrokeEngine.cpp`
- `pattern.h`

The driver abstraction files from the earlier refactor are unchanged except for being included in this package.
