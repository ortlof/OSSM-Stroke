#pragma once

#include <Arduino.h>

/**
 * Driver-independent STEP/DIR configuration.
 * This intentionally mirrors the existing motorProperties fields without
 * including StrokeEngine.h, because circular includes are how firmware grows fangs.
 */
typedef struct {
    int stepPin;
    int directionPin;
    int enablePin;
    bool invertDirection;
    bool enableActiveLow;
} StepperDriverConfig;

/**
 * Minimal motion-driver interface used by StrokeEngine.
 *
 * The method names intentionally stay close to FastAccelStepper so that the
 * current engine code can be moved behind an adapter with minimal risk.
 */
class MotionDriver {
public:
    virtual ~MotionDriver() {}

    virtual bool begin(const StepperDriverConfig& config) = 0;
    virtual bool isReady() const = 0;

    virtual void enableOutputs() = 0;
    virtual void disableOutputs() = 0;

    virtual void setCurrentPosition(int32_t position) = 0;
    virtual int32_t getCurrentPosition() const = 0;

    virtual void setSpeedInHz(uint32_t speedHz) = 0;
    virtual uint32_t getSpeedInMilliHz() const = 0;

    virtual void setAcceleration(uint32_t acceleration) = 0;
    virtual uint32_t getAcceleration() const = 0;
    virtual void applySpeedAcceleration() = 0;

    virtual void moveTo(int32_t absolutePosition) = 0;
    virtual void move(int32_t relativePosition) = 0;
    virtual void runForward() = 0;
    virtual void runBackward() = 0;

    virtual void stopMove() = 0;
    virtual void forceStop() = 0;
    virtual void forceStopAndNewPosition(int32_t position) = 0;

    virtual bool isRunning() const = 0;

    /**
     * Return predicted stopping distance in steps when the driver supports it.
     * Drivers that cannot calculate this should return 0.
     */
    virtual uint32_t stepsToStop() const = 0;
};
