#include <Arduino.h>
#include <cmath>
#include <math.h>
#include <StrokeEngine.h>
#include "FastAccelStepperMotionDriver.h"
#include <pattern.h>

void StrokeEngine::begin(machineGeometry *physics, motorProperties *motor) {
    begin(physics, motor, NULL);
}

void StrokeEngine::begin(machineGeometry *physics, motorProperties *motor, MotionDriver *driver) {
    // Store the machine geometry and motor properties pointer
    _physics = physics;
    _motor = motor;

    // Derived Machine Geometry & Motor Limits in steps:
    _travel = (_physics->physicalTravel - (2 * _physics->keepoutBoundary));
    _minStep = 0;
    _maxStep = int(0.5f + _travel * _motor->stepsPerMillimeter);
    _maxStepPerSecond = int(0.5f + _motor->maxSpeed * _motor->stepsPerMillimeter);
    _maxStepAcceleration = int(0.5f + _motor->maxAcceleration * _motor->stepsPerMillimeter);
    _defaultStopAcceleration = max(1, _maxStepAcceleration / 4);

    // Initialize with default values
    _state = UNDEFINED;
    _isHomed = false;
    _patternIndex = 0;
    _index = 0;
    _depth = _maxStep;
    _previousDepth = _maxStep;
    _stroke = _maxStep / 3;
    _previousStroke = _maxStep / 3;
    _timeOfStroke = 1.0f;
    _sensation = 0.0f;
    _applyUpdate = false;
    _stopRequested = false;
    _stopRequestEmergency = false;
    _requestedStopAcceleration = _defaultStopAcceleration;
    _motionAccelerationFactor = 1.0f;
    _reversalAccelerationFactor = 0.35f;
    _endpointDwellMillis = 10;
    _lastMoveDirection = 0;

    if (_ownsDriver && _driver != NULL) {
        delete _driver;
        _driver = NULL;
        _ownsDriver = false;
    }

    if (driver != NULL) {
        _driver = driver;
        _ownsDriver = false;
    } else {
        _driver = new FastAccelStepperMotionDriver();
        _ownsDriver = true;
    }

    StepperDriverConfig driverConfig;
    driverConfig.stepPin = _motor->stepPin;
    driverConfig.directionPin = _motor->directionPin;
    driverConfig.enablePin = _motor->enablePin;
    driverConfig.invertDirection = _motor->invertDirection;
    driverConfig.enableActiveLow = _motor->enableActiveLow;

    if ((_driver == NULL) || !_driver->begin(driverConfig)) {
        Serial.println("Servo initialization failed");
        _state = UNDEFINED;
        return;
    }

    Serial.println("Servo initialized");

#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

StrokeEngine::~StrokeEngine() {
    if (_ownsDriver && _driver != NULL) {
        delete _driver;
        _driver = NULL;
        _ownsDriver = false;
    }
}

void StrokeEngine::setSpeed(float speedMmPerSecond, bool applyNow = false) {

    // Update pattern with a LINEAR actuator speed in mm/s.
    // The old API interpreted this number as strokes per minute, which made
    // an over-limit command collapse _timeOfStroke toward 10 ms and forced
    // the engine to rely on clipping after the pattern had already generated
    // an ugly motion profile. That is how gearboxes become percussion.
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {

        if (!std::isfinite(speedMmPerSecond)) {
            speedMmPerSecond = 0.0f;
        }

        _linearSpeedMmPerSecond = constrain(speedMmPerSecond, 0.0f, _motor->maxSpeed);
        _linearSpeedStepsPerSecond = max(1, int(0.5f + _linearSpeedMmPerSecond * _motor->stepsPerMillimeter));

        _updatePatternTimeFromLinearSpeed();
        patternTable[_patternIndex]->setTimeOfStroke(_timeOfStroke);

#ifdef DEBUG_TALKATIVE
    Serial.println("setLinearSpeed: " + String(_linearSpeedMmPerSecond, 2) + "mm/s");
    Serial.println("setTimeOfStroke: " + String(_timeOfStroke, 3));
#endif

        // When running a pattern and immediate update requested:
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

#ifdef DEBUG_TALKATIVE
        Serial.println("Apply New Settings Now");
#endif
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }
}

void StrokeEngine::setSpeedFpm(float strokesPerMinute, bool applyNow = false) {
    if (!std::isfinite(strokesPerMinute) || strokesPerMinute <= 0.0f) {
        setSpeed(0.0f, applyNow);
        return;
    }

    float strokeMm = getStroke();
    float peakLinearSpeedMmPerSecond = 0.05f * strokeMm * strokesPerMinute;
    setSpeed(peakLinearSpeedMmPerSecond, applyNow);
}

float StrokeEngine::getSpeed() {
    return _linearSpeedMmPerSecond;
}

float StrokeEngine::getSpeedFpm() {
    float strokeMm = getStroke();
    if (strokeMm <= 0.0f) {
        return 0.0f;
    }

    return _linearSpeedMmPerSecond / (0.05f * strokeMm);
}

void StrokeEngine::setDepth(float depth, bool applyNow = false) {

    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        // Convert depth from mm into steps
        // Constrain depth between minStep and maxStep
        _depth = constrain(int(depth * _motor->stepsPerMillimeter), _minStep, _maxStep); 

        patternTable[_patternIndex]->setDepth(_depth);

#ifdef DEBUG_TALKATIVE
        Serial.println("setDepth: " + String(_depth));
#endif
        // When running a pattern and immediate update requested: 
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

#ifdef DEBUG_TALKATIVE
        Serial.println("Apply New Settings Now");
#endif
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }

    // if in state SETUPDEPTH then adjust
    if (_state == SETUPDEPTH) {
        _setupDepths();
    }
}

float StrokeEngine::getDepth() {
    // Convert depth from steps into mm
    return _depth / _motor->stepsPerMillimeter;
}

void StrokeEngine::setStroke(float stroke, bool applyNow = false) {
    // Update pattern with new stroke, will be used with the next stroke or on update request
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {

        // Convert stroke from mm into steps
        // Constrain stroke between minStep and maxStep
        _stroke = constrain(int(stroke * _motor->stepsPerMillimeter), _minStep, _maxStep); 
        _updatePatternTimeFromLinearSpeed();

        patternTable[_patternIndex]->setStroke(_stroke);
        patternTable[_patternIndex]->setTimeOfStroke(_timeOfStroke);

#ifdef DEBUG_TALKATIVE
        Serial.println("setStroke: " + String(_stroke));
#endif
    
        // When running a pattern and immediate update requested: 
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

#ifdef DEBUG_TALKATIVE
        Serial.println("Apply New Settings Now");
#endif
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }

    // if in state SETUPDEPTH then adjust
    if (_state == SETUPDEPTH) {
        _setupDepths();
    }
}

float StrokeEngine::getStroke() {
    // Convert stroke from steps into mm
    return _stroke / _motor->stepsPerMillimeter;
}

void StrokeEngine::setSensation(float sensation, bool applyNow = false) {

    // Update pattern with new sensation, will be used with the next stroke or on update request
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {

        // Constrain sensation between -100 and 100
        _sensation = constrain(sensation, -100.0f, 100.0f); 

        patternTable[_patternIndex]->setSensation(_sensation);

#ifdef DEBUG_TALKATIVE
        Serial.println("setSensation: " + String(_sensation));
#endif

        // When running a pattern and immediate update requested: 
        if ((_state == PATTERN) && (applyNow == true)) {
            // set flag to apply update from stroking thread
            _applyUpdate = true;

#ifdef DEBUG_TALKATIVE
        Serial.println("Apply New Settings Now");
#endif
        }

        // give back mutex
        xSemaphoreGive(_patternMutex);
    }
    
    // if in state SETUPDEPTH then adjust
    if (_state == SETUPDEPTH) {
        _setupDepths();
    }
}

float StrokeEngine::getSensation() {
    return _sensation;
}

bool StrokeEngine::setPattern(int patternIndex, bool applyNow = false) {
    // Check wether pattern Index is in range
    if ((patternIndex < patternTableSize) && (patternIndex >= 0)) {
        _patternIndex = patternIndex;

        // Inject current motion parameters into new pattern
        if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
            _injectCurrentPatternSettings();

            // When running a pattern and immediate update requested: 
            if ((_state == PATTERN) && (applyNow == true)) {
                // set flag to apply update from stroking thread
                _applyUpdate = true;

#ifdef DEBUG_TALKATIVE
            Serial.println("Apply New Settings Now");
#endif
            }

            // Reset index counter
            _index = 0; 

            // give back mutex
            xSemaphoreGive(_patternMutex);
        }

#ifdef DEBUG_TALKATIVE
    Serial.println("setPattern: [" + String(_patternIndex) + "] " + patternTable[_patternIndex]->getName());
    Serial.println("setTimeOfStroke: " + String(_timeOfStroke, 2));
    Serial.println("setDepth: " + String(_depth));
    Serial.println("setStroke: " + String(_stroke));
    Serial.println("setSensation: " + String(_sensation));
#endif
        return true;
    }

    // Return false on no match
#ifdef DEBUG_TALKATIVE
    Serial.println("Failed to set pattern: " + String(_patternIndex));
#endif
    return false;   
}

int StrokeEngine::getPattern() {
    return _patternIndex;
}

int StrokeEngine::getPhysicalTravel() {
    return _physics->physicalTravel;
}

bool StrokeEngine::startPattern() {
    // Only valid if state is ready
    if (_state == READY || _state == SETUPDEPTH) {

        // Stop current move, should one be pending (moveToMax or moveToMin)
        if (_driver->isRunning()) {
            // Stop servo motor as fast as legally allowed
            _driver->setAcceleration(_maxStepAcceleration);
            _driver->applySpeedAcceleration();
            _driver->stopMove();
        }

        // Set state to PATTERN
        _state = PATTERN;

        // Reset Stroke and Motion parameters
        _index = -1;
        if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
            _injectCurrentPatternSettings();            
            xSemaphoreGive(_patternMutex);
        }

        
#ifdef DEBUG_TALKATIVE
        Serial.print(" _timeOfStroke: " + String(_timeOfStroke));
        Serial.print(" | _depth: " + String(_depth));
        Serial.print(" | _stroke: " + String(_stroke));
        Serial.println(" | _sensation: " + String(_sensation));
#endif

        if (_taskStrokingHandle == NULL) {
            // Create Stroke Task
            xTaskCreatePinnedToCore(
                this->_strokingImpl,    // Function that should be called
                "Stroking",             // Name of the task (for debugging)
                4096,                   // Stack size (bytes)
                this,                   // Pass reference to this class instance
                24,                     // Pretty high task priority
                &_taskStrokingHandle,   // Task handle
                1                       // Pin to application core
            ); 
        } else {
            // Resume task, if it already exists
            vTaskResume(_taskStrokingHandle);
        }

#ifdef DEBUG_TALKATIVE
        Serial.println("Started motion task");
        Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

        return true;

    } else {

#ifdef DEBUG_TALKATIVE
        Serial.println("Failed to start motion");
#endif
        return false;

    }
}

void StrokeEngine::requestStop(float decelerationMmPerSecondSquared, bool emergency) {
    if (_driver == NULL) {
        return;
    }

    if (_state == SETUPDEPTH) {
        // SETUPDEPTH is not guaranteed to have the stroking task active, so keep it safe and blocking.
        stopMotion(decelerationMmPerSecondSquared, emergency);
        return;
    }

    if (_state != PATTERN) {
        return;
    }

    _requestedStopAcceleration = _resolveStopAcceleration(decelerationMmPerSecondSquared);
    _stopRequestEmergency = emergency;
    _stopRequested = true;
}

void StrokeEngine::stopMotion(float decelerationMmPerSecondSquared, bool emergency) {
    if (_driver == NULL) {
        return;
    }

    if (_state == PATTERN || _state == SETUPDEPTH || _state == STOPPING) {
        _beginStop(_resolveStopAcceleration(decelerationMmPerSecondSquared), emergency);

        // Blocking compatibility path for existing calls before homing/manual moves.
        while (_driver->isRunning()) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        _completeStop();
    }

#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

void StrokeEngine::setDefaultStopAcceleration(float decelerationMmPerSecondSquared) {
    _defaultStopAcceleration = _resolveStopAcceleration(decelerationMmPerSecondSquared);
}

float StrokeEngine::getDefaultStopAcceleration() {
    return float(_defaultStopAcceleration) / _motor->stepsPerMillimeter;
}

void StrokeEngine::setMotionAccelerationFactor(float factor) {
    _motionAccelerationFactor = constrain(factor, 0.05f, 1.0f);
}

float StrokeEngine::getMotionAccelerationFactor() {
    return _motionAccelerationFactor;
}

void StrokeEngine::setReversalAccelerationFactor(float factor) {
    _reversalAccelerationFactor = constrain(factor, 0.05f, 1.0f);
}

float StrokeEngine::getReversalAccelerationFactor() {
    return _reversalAccelerationFactor;
}

void StrokeEngine::setEndpointDwellMillis(int dwellMillis) {
    _endpointDwellMillis = constrain(dwellMillis, 0, 500);

    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        patternTable[_patternIndex]->setEndpointDwellMillis(_endpointDwellMillis);
        xSemaphoreGive(_patternMutex);
    }
}

int StrokeEngine::getEndpointDwellMillis() {
    return _endpointDwellMillis;
}

void StrokeEngine::_updatePatternTimeFromLinearSpeed() {
    // Patterns still use _timeOfStroke internally. For the common trapezoid
    // profile, peak speed = 3 * strokeSteps / fullStrokeTime. Therefore:
    // fullStrokeTime = 3 * strokeSteps / desiredPeakSpeedStepsPerSecond.
    // This preserves existing pattern behavior while making the public speed
    // command a physical linear speed instead of a stroke-rate side quest.
    int safeStrokeSteps = max(1, _stroke);
    int safeSpeedStepsPerSecond = max(1, _linearSpeedStepsPerSecond);

    if (_linearSpeedMmPerSecond <= 0.0f) {
        _timeOfStroke = 120.0f;
        return;
    }

    _timeOfStroke = constrain(3.0f * float(safeStrokeSteps) / float(safeSpeedStepsPerSecond), 0.01f, 120.0f);
}

void StrokeEngine::_injectCurrentPatternSettings() {
    _updatePatternTimeFromLinearSpeed();
    patternTable[_patternIndex]->setSpeedLimit(_maxStepPerSecond, _maxStepAcceleration, _motor->stepsPerMillimeter);
    patternTable[_patternIndex]->setTimeOfStroke(_timeOfStroke);
    patternTable[_patternIndex]->setStroke(_stroke);
    patternTable[_patternIndex]->setDepth(_depth);
    patternTable[_patternIndex]->setSensation(_sensation);
    patternTable[_patternIndex]->setEndpointDwellMillis(_endpointDwellMillis);
}

int StrokeEngine::_resolveStopAcceleration(float decelerationMmPerSecondSquared) {
    int requestedAcceleration = _defaultStopAcceleration;

    if (decelerationMmPerSecondSquared > 0.0f) {
        requestedAcceleration = int(0.5f + decelerationMmPerSecondSquared * _motor->stepsPerMillimeter);
    }

    requestedAcceleration = constrain(requestedAcceleration, 1, _maxStepAcceleration);
    return _limitStopAccelerationForTravel(requestedAcceleration);
}

int StrokeEngine::_limitStopAccelerationForTravel(int requestedAcceleration) {
    if (_driver == NULL) {
        return requestedAcceleration;
    }

    uint32_t stopDistance = _driver->stepsToStop();

    // If the active driver cannot predict stopping distance, keep the requested acceleration.
    if (stopDistance == 0) {
        return requestedAcceleration;
    }

    int32_t currentPosition = _driver->getCurrentPosition();
    int32_t distanceToMin = abs(currentPosition - _minStep);
    int32_t distanceToMax = abs(_maxStep - currentPosition);
    int32_t nearestLimitDistance = max(1, min(distanceToMin, distanceToMax));

    // Conservative safety: if a gentle stop might overrun the nearest software limit,
    // use max acceleration. Smooth is nice. Crashing is less nice. Apparently this must be said.
    if (stopDistance >= uint32_t(nearestLimitDistance)) {
        return _maxStepAcceleration;
    }

    return requestedAcceleration;
}

void StrokeEngine::_beginStop(int stopAcceleration, bool emergency) {
    if (_driver == NULL) {
        return;
    }

    _state = STOPPING;

    if (emergency) {
        _driver->forceStop();
        return;
    }

    stopAcceleration = constrain(stopAcceleration, 1, _maxStepAcceleration);
    _driver->setAcceleration(stopAcceleration);
    _driver->applySpeedAcceleration();
    _driver->stopMove();
}

void StrokeEngine::_completeStop() {
    _state = READY;
    _stopRequested = false;
    _stopRequestEmergency = false;
    _lastMoveDirection = 0;

#ifdef DEBUG_TALKATIVE
    Serial.println("Motion stopped");
#endif

    if (_callbackTelemetry != NULL && _driver != NULL) {
        _callbackTelemetry(float(_driver->getCurrentPosition()) / _motor->stepsPerMillimeter, 0.0f, false);
    }
}

void StrokeEngine::enableAndHome(endstopProperties *endstop, void(*callBackHoming)(bool), float speed) {
    // Store callback
    _callBackHomeing = callBackHoming;

    // enable and home
    enableAndHome(endstop, speed);
}

void StrokeEngine::enableAndHome(endstopProperties *endstop, float speed) {
    // set homing pin as input
    _sensorlessHomeing = false;
    _homeingPin = endstop->endstopPin;
    pinMode(_homeingPin, endstop->pinMode);
    _homeingActiveLow = endstop->activeLow;
    _homeingSpeed = speed * _motor->stepsPerMillimeter;

    // set homing direction so sign can be multiplied
    if (endstop->homeToBack == true) {
        _homeingToBack = 1;
    } else {
        _homeingToBack = -1;
    }

    // first stop current motion and delete stroke task
    stopMotion();

    // Enable Servo
    _driver->enableOutputs();

    // Create homing task
    xTaskCreatePinnedToCore(
        this->_homingProcedureImpl,     // Function that should be called
        "Homing",                       // Name of the task (for debugging)
        2048,                           // Stack size (bytes)
        this,                           // Pass reference to this class instance
        20,                             // Pretty high task priority
        &_taskHomingHandle,             // Task handle
        1                               // Have it on application core
    ); 
#ifdef DEBUG_TALKATIVE
    Serial.println("Homing task started");
#endif

}

void StrokeEngine::enableAndSensorlessHome(sensorlessHomeProperties *sensorless, void(*callBackHoming)(bool), float speed) {
    // Store callback
    _callBackHomeing = callBackHoming;

    // enable and sensorless home
    enableAndSensorlessHome(sensorless, speed);
}

void StrokeEngine::enableAndSensorlessHome(sensorlessHomeProperties *sensorless, float speed) {
    _sensorlessHomeing = true;
    _homeingSpeed = speed * _motor->stepsPerMillimeter;
    _sensorlessHomeingCurrentPin = sensorless->currentPin;
    pinMode(_sensorlessHomeingCurrentPin, INPUT);
    _sensorlessHomeingCurrentLimit = sensorless->currentLimit;

    // first stop current motion and delete stroke task
    stopMotion();

    // Create homing task
    xTaskCreatePinnedToCore(
        this->_homingProcedureImpl,     // Function that should be called
        "SensorlessHoming",             // Name of the task (for debugging)
        2048,                           // Stack size (bytes)
        this,                           // Pass reference to this class instance
        20,                             // Pretty high task priority
        &_taskHomingHandle,             // Task handle
        1                               // Have it on application core
    ); 
#ifdef DEBUG_TALKATIVE
    Serial.println("Sensorless homing task started");
#endif

}

void StrokeEngine::thisIsHome(float speed) {
    // set homeing speed
    _homeingSpeed = speed * _motor->stepsPerMillimeter;

    if (_state == UNDEFINED) {
        // Enable Servo
        _driver->enableOutputs();

        // Stet current position as home
        _driver->setCurrentPosition(-_motor->stepsPerMillimeter * _physics->keepoutBoundary);

        // Set feedrate for homing
        _driver->setSpeedInHz(_homeingSpeed);       
        _driver->setAcceleration(_maxStepAcceleration / 10);

        // drive free of switch and set axis to 0
        _driver->moveTo(_minStep);
        
        // Change state
        _isHomed = true;
        _state = READY;

#ifdef DEBUG_TALKATIVE
        Serial.println("This is Home now");
#endif

        return;
    }

#ifdef DEBUG_TALKATIVE
    Serial.println("Manual homing failed. Not in state UNDEFINED");
#endif

}

bool StrokeEngine::moveToMax(float speed) {

#ifdef DEBUG_TALKATIVE
    Serial.println("Move to max");
#endif

    if (_isHomed) {
        // Stop motion immediately
        stopMotion();

        // Set feedrate for safe move 
        // Constrain speed between 1 step/sec and _maxStepPerSecond
        _driver->setSpeedInHz(constrain(int(0.5f + speed * _motor->stepsPerMillimeter), 1, _maxStepPerSecond));
        _driver->setAcceleration(_maxStepAcceleration / 10);
        _driver->moveTo(_maxStep);

        // Send telemetry data
        if (_callbackTelemetry != NULL) {
            _callbackTelemetry(float(_maxStep) / _motor->stepsPerMillimeter, speed, false);
        } 

#ifdef DEBUG_TALKATIVE
        Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

        // Return success
        return true;

    } else {
        // Return failure
        return false;
    }
}

bool StrokeEngine::moveToMin(float speed) {

#ifdef DEBUG_TALKATIVE
    Serial.println("Move to min");
#endif

    if (_isHomed) {
        // Stop motion immediately
        stopMotion();

        // Set feedrate for safe move 
        // Constrain speed between 1 step/sec and _maxStepPerSecond
        _driver->setSpeedInHz(constrain(int(0.5f + speed * _motor->stepsPerMillimeter), 1, _maxStepPerSecond));
        _driver->setAcceleration(_maxStepAcceleration / 10);
        _driver->moveTo(_minStep);

        // Send telemetry data
        if (_callbackTelemetry != NULL) {
            _callbackTelemetry(float(_minStep) / _motor->stepsPerMillimeter, speed, false);
        } 

#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

        // Return success
        return true;

    } else {
        // Return failure
        return false;
    }
}

bool StrokeEngine::setupDepth(float speed, bool fancy) {
#ifdef DEBUG_TALKATIVE
    Serial.println("Move to Depth");
#endif
    // store fanciness
    _fancyAdjustment = fancy;

    // returns true on success, and false if in wrong state
    bool allowed = false;

    // isHomed is only true in states READY, PATTERN and SETUPDEPTH
    if (_isHomed) {
        // Stop motion immediately
        stopMotion();

        // Set feedrate for safe move 
        // Constrain speed between 1 step/sec and _maxStepPerSecond
        _driver->setSpeedInHz(constrain(int(0.5f + speed * _motor->stepsPerMillimeter), 1, _maxStepPerSecond));
        _driver->setAcceleration(_maxStepAcceleration / 10);

        // Set new state
        _state = SETUPDEPTH;

        // move to current depth position
        _setupDepths();

        // set return value to true
        allowed = true;
    }
#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
    return allowed;
}

ServoState StrokeEngine::getState() {
    return _state;
}

void StrokeEngine::disable() {
    // Delete homing Task
    _abortHoming = true;
    while (_taskHomingHandle != NULL) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    _abortHoming = false;

    _state = UNDEFINED;
    _isHomed = false;

    // Disable servo motor
    _driver->disableOutputs();

#ifdef DEBUG_TALKATIVE
    Serial.println("Servo disabled. Call home to continue.");
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

}

String StrokeEngine::getPatternName(int index) {
    if (index >= 0 && index < patternTableSize) {
        return String(patternTable[index]->getName());
    } else {
        return String("Invalid");
    }
    
}

void StrokeEngine::setMaxSpeed(float maxSpeed){
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        _motor->maxSpeed = max(0.0f, maxSpeed);
        _maxStepPerSecond = int(0.5f + _motor->maxSpeed * _motor->stepsPerMillimeter);
        _linearSpeedMmPerSecond = constrain(_linearSpeedMmPerSecond, 0.0f, _motor->maxSpeed);
        _linearSpeedStepsPerSecond = max(1, int(0.5f + _linearSpeedMmPerSecond * _motor->stepsPerMillimeter));
        _updatePatternTimeFromLinearSpeed();
        patternTable[_patternIndex]->setSpeedLimit(_maxStepPerSecond, _maxStepAcceleration, _motor->stepsPerMillimeter);
        patternTable[_patternIndex]->setTimeOfStroke(_timeOfStroke);
        xSemaphoreGive(_patternMutex);
    }
}

float StrokeEngine::getMaxSpeed() {
    return float(_maxStepPerSecond / _motor->stepsPerMillimeter);
}

void StrokeEngine::setMaxAcceleration(float maxAcceleration) {
    if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
        _motor->maxAcceleration = max(0.0f, maxAcceleration);
        _maxStepAcceleration = int(0.5f + _motor->maxAcceleration * _motor->stepsPerMillimeter);
        _defaultStopAcceleration = constrain(_defaultStopAcceleration, 1, _maxStepAcceleration);
        patternTable[_patternIndex]->setSpeedLimit(_maxStepPerSecond, _maxStepAcceleration, _motor->stepsPerMillimeter);
        xSemaphoreGive(_patternMutex);
    }
}

float StrokeEngine::getMaxAcceleration() {
    return float(_maxStepAcceleration / _motor->stepsPerMillimeter);
}

void StrokeEngine::registerTelemetryCallback(void(*callbackTelemetry)(float, float, bool)) {
    _callbackTelemetry = callbackTelemetry;
}

float StrokeEngine::_getAnalogAveragePercent(int pinNumber, int samples) {
    float sum = 0;
    float average = 0;
    float percentage = 0;
    for (int i = 0; i < samples; i++)
    {
        // TODO: Possibly use fancier filters?
        sum += analogRead(pinNumber);
    }
    average = sum / samples;
    // TODO: Might want to add a deadband
    percentage = 100.0 * average / 4096.0; // 12 bit resolution
    return percentage;
}

void StrokeEngine::_homingProcedure() {
    if(_sensorlessHomeing) {
        _sensorlessHomingProcedure();
    } else {
        _sensorHomingProcedure();
    }

    _taskHomingHandle = NULL;
    vTaskDelete(NULL);
}

void StrokeEngine::_sensorlessHomingProcedure() {
#ifdef DEBUG_TALKATIVE
    Serial.println("Finding Home Sensorless");
#endif
    float currentSensorOffset = (_getAnalogAveragePercent(_sensorlessHomeingCurrentPin, 1000));
    float current = _getAnalogAveragePercent(_sensorlessHomeingCurrentPin, 200) - currentSensorOffset;
#ifdef DEBUG_TALKATIVE
    long lastMillisMessage = 0;
#endif

    // Set feedrate for homing
    _driver->setSpeedInHz(_homeingSpeed);
    _driver->setAcceleration(_maxStepAcceleration / 10);

    // disable motor briefly in case we are against a hard stop.
    _driver->disableOutputs();
    vTaskDelay(600 / portTICK_PERIOD_MS);
    if(_abortHoming) return;
    _driver->enableOutputs();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if(_abortHoming) return;

#ifdef DEBUG_TALKATIVE
    Serial.print(_getAnalogAveragePercent(_sensorlessHomeingCurrentPin, 500) - currentSensorOffset);
    Serial.print(",");
    Serial.println(_driver->getCurrentPosition() / _motor->stepsPerMillimeter);
    Serial.println(_motor->stepsPerMillimeter);
    Serial.println("Sensorless homing move");
#endif
    _driver->runForward();

    current = _getAnalogAveragePercent(_sensorlessHomeingCurrentPin, 200) - currentSensorOffset;
    while (current < _sensorlessHomeingCurrentLimit)
    {
        if(_abortHoming) return;
        current = _getAnalogAveragePercent(_sensorlessHomeingCurrentPin, 25) - currentSensorOffset;
#ifdef DEBUG_TALKATIVE
        if(millis() - lastMillisMessage > 200) {
            Serial.print(current);
            Serial.print(",");
            Serial.println(_driver->getCurrentPosition());
            lastMillisMessage = millis();
        }
#endif

        // Let other tasks run
        vTaskDelay(0);
    }

    _driver->forceStopAndNewPosition(0);
#ifdef DEBUG_TALKATIVE
    Serial.println("Sensorless found max");
#endif

    _driver->runBackward();

    vTaskDelay(300 / portTICK_PERIOD_MS);

    current = _getAnalogAveragePercent(_sensorlessHomeingCurrentPin, 200) - currentSensorOffset;
    while (current < _sensorlessHomeingCurrentLimit)
    {
        if(_abortHoming) return;
        current = _getAnalogAveragePercent(_sensorlessHomeingCurrentPin, 25) - currentSensorOffset;
#ifdef DEBUG_TALKATIVE
        if(millis() - lastMillisMessage > 200) {
            Serial.print(current);
            Serial.print(",");
            Serial.println(_driver->getCurrentPosition());
            lastMillisMessage = millis();
        }
#endif
        // Let other tasks run
        vTaskDelay(0);
    }

    if(_abortHoming) return;

    _physics->physicalTravel = abs(_driver->getCurrentPosition()) / _motor->stepsPerMillimeter;
    _travel = (_physics->physicalTravel - (2 * _physics->keepoutBoundary));
    _driver->forceStopAndNewPosition(-_motor->stepsPerMillimeter * _physics->keepoutBoundary);

#ifdef DEBUG_TALKATIVE
    Serial.printf("Found rail length: %f\n", _physics->physicalTravel);
#endif

    _driver->moveTo(0);

    _isHomed = true;
    _state = READY;

#ifdef DEBUG_TALKATIVE
        Serial.println("Homing succeeded");
#endif

    // Call notification callback, if it was defined.
    if (_callBackHomeing != NULL) {
        _callBackHomeing(_isHomed);
    }

    // Set first point for telemetry
    if (_callbackTelemetry != NULL) {
        _callbackTelemetry(0.0, 0.0, false);
    }

#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

void StrokeEngine::_sensorHomingProcedure() {
    // Set feedrate for homing
    _driver->setSpeedInHz(_homeingSpeed);       
    _driver->setAcceleration(_maxStepAcceleration / 10);    

    // Check if we are already at the homing switch
    if (digitalRead(_homeingPin) == !_homeingActiveLow) {
        //back off 5 mm from switch
        _driver->move(_motor->stepsPerMillimeter * 2 * _physics->keepoutBoundary * _homeingToBack);

        // wait for move to complete
        while (_driver->isRunning()) {
            if(_abortHoming) return;
            // Pause the task for 100ms while waiting for move to complete
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        // move back towards endstop
        _driver->move(-_motor->stepsPerMillimeter * 4 * _physics->keepoutBoundary * _homeingToBack);

    } else {
        // Move MAX_TRAVEL towards the homing switch
        _driver->move(-_motor->stepsPerMillimeter * _physics->physicalTravel * _homeingToBack);
    }

    // Poll homing switch
    while (_driver->isRunning()) {
        if(_abortHoming) return;
        // Switch is active low
        if (digitalRead(_homeingPin) == !_homeingActiveLow) {

            // Set home position
            if (_homeingToBack == 1) {
                //Switch is at -KEEPOUT_BOUNDARY
                _driver->forceStopAndNewPosition(-_motor->stepsPerMillimeter * _physics->keepoutBoundary);

                // drive free of switch and set axis to lower end
                _driver->moveTo(_minStep);

            } else {
                _driver->forceStopAndNewPosition(_motor->stepsPerMillimeter * (_physics->physicalTravel - _physics->keepoutBoundary));

                // drive free of switch and set axis to front end
                _driver->moveTo(_maxStep);
            }
            _isHomed = true;

            // drive free of switch and set axis to 0
            _driver->moveTo(0);
            
            // Break loop, home was found
            break;
        }

        // Pause the task for 20ms to allow other tasks
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    
    // disable Servo if homing has not found the homing switch
    if (!_isHomed) {
        _driver->disableOutputs();
        _state = UNDEFINED;

#ifdef DEBUG_TALKATIVE
        Serial.println("Homing failed");
#endif

    } else {
        // Set state to ready
        _state = READY;

#ifdef DEBUG_TALKATIVE
        Serial.println("Homing succeeded");
#endif
    }

    // Call notification callback, if it was defined.
    if (_callBackHomeing != NULL) {
        _callBackHomeing(_isHomed);
    }

    // Set first point for telemetry
    if (_callbackTelemetry != NULL) {
        _callbackTelemetry(0.0, 0.0, false);
    }

#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

void StrokeEngine::_stroking() {
    motionParameter currentMotion;

    while(1) { // infinite loop

        // Suspend task, if not in a state owned by this task.
        if (_state != PATTERN && _state != STOPPING) {
            vTaskSuspend(_taskStrokingHandle);
        }

        if (_stopRequested && _state == PATTERN) {
            _beginStop(_requestedStopAcceleration, _stopRequestEmergency);
            _stopRequested = false;
        }

        if (_state == STOPPING) {
            if (_driver != NULL && !_driver->isRunning()) {
                _completeStop();
                vTaskSuspend(_taskStrokingHandle);
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // Take mutex to ensure no interference / race condition with communication thread on other core
        if (xSemaphoreTake(_patternMutex, 0) == pdTRUE) {

            if (_applyUpdate == true) {
                // Ask pattern for update on motion parameters
                currentMotion = patternTable[_patternIndex]->nextTarget(_index);

                // Increase deceleration if required to avoid crash
                if (_driver != NULL && _driver->getAcceleration() > currentMotion.acceleration) {
#ifdef DEBUG_CLIPPING
                    Serial.print("Crash avoidance! Set Acceleration from " + String(currentMotion.acceleration));
                    Serial.println(" to " + String(_driver->getAcceleration()));
#endif
                    currentMotion.acceleration = _driver->getAcceleration();
                }

                // Apply new trapezoidal motion profile to servo
                _applyMotionProfile(&currentMotion);

                // clear update flag
                _applyUpdate = false;
            }

            // If motor has stopped issue moveTo command to next position
            else if (_driver != NULL && _driver->isRunning() == false) {

                // Increment index for pattern
                _index++;

                // Query new set of pattern parameters
                currentMotion = patternTable[_patternIndex]->nextTarget(_index);

                // Pattern may introduce pauses between strokes
                if (currentMotion.skip == false) {

#ifdef DEBUG_STROKE
                    Serial.println("Stroking Index: " + String(_index));
#endif
                    // Apply new trapezoidal motion profile to servo
                    _applyMotionProfile(&currentMotion);

                } else {
                    // decrement _index so that it stays the same until the next valid stroke parameters are delivered
                    _index--;
                }
            }

            // give back mutex
            xSemaphoreGive(_patternMutex);
        }

        // Delay 10ms
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void StrokeEngine::_streaming() {

    while(1) { // infinite loop

        // Suspend task, if not in STREAMING state
        if (_state != STREAMING) {
            vTaskSuspend(_taskStreamingHandle);
        }
        
        // Delay 10ms 
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void StrokeEngine::_applyMotionProfile(motionParameter* motion) {

    if (_driver == NULL) {
        return;
    }

    bool clipping = false;
    float speed = 0.0;
    float position = 0.0;

    // Apply new trapezoidal motion profile to servo if pattern does not skip
    if (motion->skip == false) {

        // Constrain stroke to motion envelope early so reversal detection uses
        // the actual target position that will be sent to the driver.
        int pos = constrain((motion->stroke), _minStep, _maxStep);

        // Globally soften pattern acceleration if requested. This is separate
        // from the physical max acceleration limit, so safety clipping still
        // has the final word.
        motion->acceleration = max(1, int(float(motion->acceleration) * _motionAccelerationFactor));

        // Rack/pinion plus gearbox backlash gets noisy when a full acceleration
        // reversal slams the gear train. Soften only direction changes.
        int currentPosition = _driver->getCurrentPosition();
        int moveDelta = pos - currentPosition;
        int moveDirection = 0;

        if (moveDelta > 0) {
            moveDirection = 1;
        } else if (moveDelta < 0) {
            moveDirection = -1;
        }

        bool directionChanged = (_lastMoveDirection != 0)
                             && (moveDirection != 0)
                             && (moveDirection != _lastMoveDirection);

        if (directionChanged) {
            motion->acceleration = max(1, int(float(motion->acceleration) * _reversalAccelerationFactor));
        }

        if (moveDirection != 0) {
            _lastMoveDirection = moveDirection;
        }

        // Constrain speed to below _maxStepPerSecond
        if (motion->speed > _maxStepPerSecond) {
#ifdef DEBUG_CLIPPING
        Serial.println("Max Speed Exceeded: " + String(float(motion->speed) / _motor->stepsPerMillimeter, 2)
                + "mm/s --> Limit: " + String(float(_maxStepPerSecond) / _motor->stepsPerMillimeter, 2) + "mm/s");
#endif
            motion->speed = _maxStepPerSecond;
            clipping = true;
        } 

        // Constrain acceleration between 1 step/sec^2 and _maxStepAcceleration
        if (motion->acceleration > _maxStepAcceleration) {
#ifdef DEBUG_CLIPPING
        Serial.println("Max Acceleration Exceeded: " + String(float(motion->acceleration) / _motor->stepsPerMillimeter, 2)
                + "mm/s² --> Limit: " + String(float(_maxStepAcceleration) / _motor->stepsPerMillimeter, 2) + "mm/s²");
#endif
            motion->acceleration = _maxStepAcceleration;
            clipping = true;
        } 

        // write values to servo
        _driver->setSpeedInHz(motion->speed);
        _driver->setAcceleration(motion->acceleration);
        _driver->moveTo(pos);

        // Compile speed telemetry data
        speed = float(motion->speed) / _motor->stepsPerMillimeter;
        position = float(pos) / _motor->stepsPerMillimeter;

#ifdef DEBUG_STROKE
    Serial.println("motion.stroke: " + String(position, 2) + "mm");
    Serial.println("motion.speed: " + String(speed, 2) + "mm/s");
    Serial.println("motion.acceleration: " + String(float(motion->acceleration) / _motor->stepsPerMillimeter, 2) + "mm/s²");
#endif

        // Send telemetry data
        if (_callbackTelemetry != NULL) {
            _callbackTelemetry(position, speed, clipping);
        }
    }
}

void StrokeEngine::_setupDepths() {
    // set depth to _depth
    int depth = _depth;

    // in fancy mode we need to calculate exact position based on sensation, stroke & depth
    if (_fancyAdjustment == true) {
        // map sensation into the interval [depth-stroke, depth]
        depth = map(_sensation, -100, 100, _depth - _stroke, _depth);

#ifdef DEBUG_TALKATIVE
        Serial.println("map sensation " + String(_sensation)
            + " to interval [" + String(_depth - _stroke)
            + ", " + String(_depth) 
            + "] = " + String(depth));
#endif
    } 

    // move servo to desired position
    _driver->moveTo(depth);

    // Send telemetry data
    if (_callbackTelemetry != NULL) {
        _callbackTelemetry(float(depth) / _motor->stepsPerMillimeter, 
            float(_driver->getSpeedInMilliHz()) / 1000.0f / _motor->stepsPerMillimeter, 
            false);
    } 

#ifdef DEBUG_TALKATIVE
    Serial.println("setup new depth: " + String(depth));
#endif
}

