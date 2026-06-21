#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>
#include "MotionDriver.h"

/**
 * FastAccelStepper adapter for StrokeEngine.
 *
 * Drop another adapter beside this file if you want to test ESP-FlexyStepper or
 * another STEP/DIR backend later. StrokeEngine should not care. That is the
 * whole point, despite the universe's ongoing preference for coupling.
 */
class FastAccelStepperMotionDriver : public MotionDriver {
public:
    FastAccelStepperMotionDriver() = default;

    bool begin(const StepperDriverConfig& config) override {
        _engine.init();
        _stepper = _engine.stepperConnectToPin(config.stepPin);

        if (_stepper == nullptr) {
            return false;
        }

        _stepper->setDirectionPin(config.directionPin, config.invertDirection);
        _stepper->setEnablePin(config.enablePin, config.enableActiveLow);
        _stepper->setAutoEnable(false);
        _stepper->disableOutputs();
        return true;
    }

    bool isReady() const override {
        return _stepper != nullptr;
    }

    void enableOutputs() override {
        if (_stepper) _stepper->enableOutputs();
    }

    void disableOutputs() override {
        if (_stepper) _stepper->disableOutputs();
    }

    void setCurrentPosition(int32_t position) override {
        if (_stepper) _stepper->setCurrentPosition(position);
    }

    int32_t getCurrentPosition() const override {
        return _stepper ? _stepper->getCurrentPosition() : 0;
    }

    void setSpeedInHz(uint32_t speedHz) override {
        if (_stepper) _stepper->setSpeedInHz(speedHz);
    }

    uint32_t getSpeedInMilliHz() const override {
        return _stepper ? _stepper->getSpeedInMilliHz() : 0;
    }

    void setAcceleration(uint32_t acceleration) override {
        if (_stepper) _stepper->setAcceleration(acceleration);
    }

    uint32_t getAcceleration() const override {
        return _stepper ? _stepper->getAcceleration() : 0;
    }

    void applySpeedAcceleration() override {
        if (_stepper) _stepper->applySpeedAcceleration();
    }

    void moveTo(int32_t absolutePosition) override {
        if (_stepper) _stepper->moveTo(absolutePosition);
    }

    void move(int32_t relativePosition) override {
        if (_stepper) _stepper->move(relativePosition);
    }

    void runForward() override {
        if (_stepper) _stepper->runForward();
    }

    void runBackward() override {
        if (_stepper) _stepper->runBackward();
    }

    void stopMove() override {
        if (_stepper) _stepper->stopMove();
    }

    void forceStop() override {
        if (_stepper) {
            // forceStopAndNewPosition() exists in the original code path already.
            // Using it here avoids depending on optional forceStop() availability.
            _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
        }
    }

    void forceStopAndNewPosition(int32_t position) override {
        if (_stepper) _stepper->forceStopAndNewPosition(position);
    }

    bool isRunning() const override {
        return _stepper ? _stepper->isRunning() : false;
    }

    uint32_t stepsToStop() const override {
        // Some FastAccelStepper releases do not expose stepsToStop().
        // Return 0 to tell StrokeEngine that predictive stop-distance safety
        // is unavailable. The actual smooth stop still works via stopMove().
        return 0;
    }

private:
    FastAccelStepperEngine _engine;
    FastAccelStepper *_stepper = nullptr;
};
