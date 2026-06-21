/**
 *   StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine 
 *
 * Copyright (C) 2022 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#pragma once

#include <Arduino.h>
#include "MotionDriver.h"
#include <pattern.h>

// Debug Levels
//#define DEBUG_TALKATIVE             // Show debug messages from the StrokeEngine on Serial
#define DEBUG_STROKE                // Show debug messaged for each individual stroke on Serial
#define DEBUG_CLIPPING              // Show debug messages when motions violating the machine 
                                    // physics are commanded

/**************************************************************************/
/*!
  @brief  Struct defining the physical properties of the stroking machine.
*/
/**************************************************************************/
typedef struct {
  float physicalTravel;       /*> What is the maximum physical travel in mm */
  float keepoutBoundary;      /*> Soft endstop preventing hard crashes in mm. Will be 
                               *  subtracted twice from physicalTravel. Should be 
                               *  sufficiently to completley drive clear from 
                               *  homing switch */
} machineGeometry;

/**************************************************************************/
/*!
  @brief  Struct defining the motor (stepper or servo with STEP/DIR 
  interface) and the motion system translating the rotation into a 
  linear motion.
*/
/**************************************************************************/
typedef struct {
  float maxSpeed;             /*> What is the maximum speed in mm/s */
  float maxAcceleration;      /*> Maximum acceleration in mm/s^2 */
  float stepsPerMillimeter;   /*> Number of steps per millimeter */
  bool invertDirection;       /*> Set to true to invert the direction signal
                               *  The firmware expects the home switch to be located at the 
                               *  end of an retraction move. That way the machine homes 
                               *  itself away from the body. Home position is -KEEPOUTBOUNDARY */
  bool enableActiveLow;       /*> Polarity of the enable signal. True for active low. */
  int stepPin;                /*> Pin connected to the STEP input */
  int directionPin;           /*> Pin connected to the DIR input */
  int enablePin;              /*> Pin connected to the ENA input */
} motorProperties;

/**************************************************************************/
/*!
  @brief  Struct defining the endstop properties like pin, pinmode, polarity 
  and homing direction.
*/
/**************************************************************************/
typedef struct {
  bool homeToBack;            /*> Set to true to home to the back of the machine
                               *  Set to false to home to the front of the machine */
  bool activeLow;             /*> Polarity of the homing signal. True for active low. */
  int endstopPin;             /*> Pin connected to home switch */
  uint8_t pinMode;            /*> Pinmode of the switch INPUT, INPUT_PULLUP, INPUT_PULLDOWN */
} endstopProperties;

typedef struct {
  int currentPin; /*> Pin connected to current sensor */
  float currentLimit; /*> Current limit */
} sensorlessHomeProperties;

/**************************************************************************/
/*!
  @brief  Enum containing the states of the state machine
*/
/**************************************************************************/
typedef enum {
  UNDEFINED,          //!< No power to the servo. We don't know its position
  READY,             //!< Servo is energized and knows its position. Not running.
  PATTERN,           //!< Stroke Engine is running and servo is moving according to defined pattern.
  SETUPDEPTH,        //!< Interactive adjustment mode to setup depth and stroke
  STREAMING,         //!< Tracks the depth-position whenever depth is updated.
  STOPPING           //!< Servo is decelerating to a controlled stop.
} ServoState;

// Verbose strings of states for debugging purposes
static String verboseState[] = {
  "[0] Servo disabled",
  "[1] Servo ready",
  "[2] Servo pattern running",
  "[3] Servo setup depth",
  "[4] Servo position streaming",
  "[5] Servo stopping"
};

/**************************************************************************/
/*!
  @brief  Stroke Engine provides a convenient package for stroking motions
  created by stepper or servo motors. It's internal states are handled by a 
  finite state machine. A pattern generator allows to creat a variety of 
  motion profiles. Under the hood FastAccelStepper is used for interfacing
  a stepper or servo motor vie a STEP/DIR interface.  
*/
/**************************************************************************/
class StrokeEngine {
    public:

        /**************************************************************************/
        /*!
          @brief  Initializes FastAccelStepper and configures all pins and outputs
          accordingly. StrokeEngine is in state UNDEFINED
        */
        /**************************************************************************/
        void begin(machineGeometry *physics, motorProperties *motor);
        void begin(machineGeometry *physics, motorProperties *motor, MotionDriver *driver);
        ~StrokeEngine();

        /**************************************************************************/
        /*!
          @brief  Set the commanded linear actuator speed in mm/s.
                  Internally this is converted to a pattern stroke time so that
                  normal trapezoid patterns do not request a peak speed above
                  the configured motor max speed. This replaces the old
                  stroke-per-minute speed model, because gearboxes dislike
                  surprise math violence.
          @param speedMmPerSecond Linear actuator speed in mm/s. Clamped to
                                  0..motor.maxSpeed.
          @param applyNow Set to true if changes should take effect immediately.
        */
        /**************************************************************************/
        void setSpeed(float speedMmPerSecond, bool applyNow);

        /**************************************************************************/
        /*!
          @brief  Compatibility helper for callers that still think in strokes
                  per minute. Converts FPM to a linear peak speed using the
                  current stroke length, then calls setSpeed().
          @param strokesPerMinute Stroke rate in full strokes per minute.
          @param applyNow Set to true if changes should take effect immediately.
        */
        /**************************************************************************/
        void setSpeedFpm(float strokesPerMinute, bool applyNow);

        /**************************************************************************/
        /*!
          @brief  Get the currently commanded linear speed in mm/s.
          @return Linear actuator speed in mm/s.
        */
        /**************************************************************************/
        float getSpeed();

        /**************************************************************************/
        /*!
          @brief  Get the equivalent legacy stroke rate for the current stroke
                  length. Returns 0 if the stroke length is 0.
          @return Strokes per minute.
        */
        /**************************************************************************/
        float getSpeedFpm();

        /**************************************************************************/
        /*!
          @brief  Set the depth of a stroke. Settings tale effect with next stroke, 
          or after calling applyNewSettingsNow().
          @param depth Depth in [mm]. Is constrained from 0 to TRAVEL 
          @param applyNow Set to true if changes should take effect immediately 
        */
        /**************************************************************************/
        void setDepth(float depth, bool applyNow);

        /**************************************************************************/
        /*!
          @brief  Returns the depth of a stroke. 
          @return Depth in [mm]. Is constrained from 0 to TRAVEL 
        */
        /**************************************************************************/
        float getDepth();

        /**************************************************************************/
        /*!
          @brief  Set the stroke length of a stroke. Settings take effect with next 
          stroke, or after calling applyNewSettingsNow().
          @param stroke Stroke length in [mm]. Is constrained from 0 to TRAVEL 
          @param applyNow Set to true if changes should take effect immediately 
        */
        /**************************************************************************/
        void setStroke(float stroke, bool applyNow);

        /**************************************************************************/
        /*!
          @brief  Get the stroke length of a stroke.
          @return Stroke length in [mm].
        */
        /**************************************************************************/
        float getStroke();

        /**************************************************************************/
        /*!
          @brief  Set the sensation of a pattern. Sensation is an additional 
          parameter a pattern may use to alter its behaviour. Settings takes 
          effect with next stroke, or after calling applyNewSettingsNow().
          @param sensation  Sensation in [a.u.]. Is constrained from -100 to 100  
                        with 0 beeing assumed as neutral.
          @param applyNow Set to true if changes should take effect immediately 
        */
        /**************************************************************************/
        void setSensation(float sensation, bool applyNow);

        /**************************************************************************/
        /*!
          @brief  Get the sensation of a pattern. Sensation is an additional 
          parameter a pattern may use to alter its behaviour.
          @return Sensation in [a.u.]. Is constrained from -100 to 100  
                        with 0 beeing assumed as neutral.
        */
        /**************************************************************************/
        float getSensation();

        /**************************************************************************/
        /*!
          @brief  Choose a pattern for the StrokeEngine. Settings take effect with 
          next stroke, or after calling applyNewSettingsNow(). 
          @param patternIndex  Index of a pattern
          @param applyNow Set to true if changes should take effect immediately 
          @return TRUE on success, FALSE, if patternIndex is invalid. Previous 
                        pattern will be retained.
        */
        /**************************************************************************/
        bool setPattern(int patternIndex, bool applyNow);

        /**************************************************************************/
        /*!
          @brief  Get the pattern index for the StrokeEngine.
          @return Index of a pattern
        */
        /**************************************************************************/
        int getPattern();

        /**************************************************************************/
        /*!
          @brief  Creates a FreeRTOS task to run a stroking pattern. Only valid in
          state READY. Pattern is initialized with the values from the set 
          functions. If the task is running, state is PATTERN.
          @return TRUE when task was created and motion starts, FALSE on failure.
        */
        /**************************************************************************/
        int getPhysicalTravel();

        /**************************************************************************/
        /*!
          @brief  Creates a FreeRTOS task to run a stroking pattern. Only valid in
          state READY. Pattern is initialized with the values from the set 
          functions. If the task is running, state is PATTERN.
          @return TRUE when task was created and motion starts, FALSE on failure.
        */
        /**************************************************************************/
        bool startPattern();

        /**************************************************************************/
        /*!
          @brief  Stops the motion with MAX_ACCEL and deletes the stroking task. Is
          in state READY afterwards.
        */
        /**************************************************************************/
        void stopMotion(float decelerationMmPerSecondSquared = 0.0f, bool emergency = false);

        /**************************************************************************/
        /*!
          @brief  Requests a non-blocking controlled stop. In PATTERN state the
                  stroking task applies the stop and transitions through STOPPING
                  to READY. If deceleration is <= 0, the configured default stop
                  acceleration is used.
        */
        /**************************************************************************/
        void requestStop(float decelerationMmPerSecondSquared = 0.0f, bool emergency = false);

        void setDefaultStopAcceleration(float decelerationMmPerSecondSquared);
        float getDefaultStopAcceleration();

        /**************************************************************************/
        /*!
          @brief  Scales acceleration for every pattern move. Use this to globally
                  calm the machine without changing the configured physical safety
                  limit. 1.0 keeps existing behavior, 0.5 halves pattern acceleration.
        */
        /**************************************************************************/
        void setMotionAccelerationFactor(float factor);
        float getMotionAccelerationFactor();

        /**************************************************************************/
        /*!
          @brief  Extra acceleration reduction applied only when the next move
                  reverses direction. Useful for geared rack and pinion systems
                  with backlash. 1.0 keeps existing behavior, lower values soften
                  reversal.
        */
        /**************************************************************************/
        void setReversalAccelerationFactor(float factor);
        float getReversalAccelerationFactor();

        /**************************************************************************/
        /*!
          @brief  Adds a small pause after each completed pattern move before
                  commanding the next move. This lets backlash and drivetrain
                  compliance settle before reversing direction.
        */
        /**************************************************************************/
        void setEndpointDwellMillis(int dwellMillis);
        int getEndpointDwellMillis();

        /**************************************************************************/
        /*!
          @brief  Enable the servo/stepper and do the homing procedure. Drives towards
          the endstop with HOMING_SPEED. Function is non-blocking and backed by a task.
          Optionally a callback can be given to receive feedback if homing succeeded 
          going in state READY. If homing switch is not found after traveling 
          MAX_TRAVEL it times out, disables the servo and goes into UNDEFINED. 
          @param endstop Pointer to a endstopProperties struct defining all relevant 
                        properties like pin, pinmode, homing direction & signal  
                        polarity.
          @param speed  Speed in mm/s used for finding the homing switch. 
                        Defaults to 5.0 mm/s
          @param callBackHoming Callback function is called after homing is done. 
                        Function parametere holds a bool containing the success (TRUE)
                        or failure (FALSE) of homing.
        */
        /**************************************************************************/
        void enableAndHome(endstopProperties *endstop, float speed = 5.0);
        void enableAndHome(endstopProperties *endstop, void(*callBackHoming)(bool), float speed = 5.0);

        void enableAndSensorlessHome(sensorlessHomeProperties *sensorless, float speed = 5.0);
        void enableAndSensorlessHome(sensorlessHomeProperties *sensorless, void(*callBackHoming)(bool), float speed = 5.0);

        /**************************************************************************/
        /*!
          @brief  If no homing switch is present homing can be done manually. Push 
          the endeffector all the way in and call thisIsHome(). This enables the
          the servo and sets the position to -KEEPOUT_BOUNDARY
          @param speed  Speed in mm/s used for finding the homing switch. 
                        Defaults to 5.0 mm/s
        */
        /**************************************************************************/
        void thisIsHome(float speed = 5.0);

        /**************************************************************************/
        /*!
          @brief  In state PATTERN, SETUPDEPTH and READY this 
          moves the endeffector to TRAVEL. Can be used for adjustments. Stops any 
          running pattern and ends in state READY.
          @param speed  Speed in mm/s used for driving to max. 
                        Defaults to 10.0 mm/s
          @return TRUE on success, FALSE if state does not allow this.
        */
        /**************************************************************************/
        bool moveToMax(float speed = 10.0);

        /**************************************************************************/
        /*!
          @brief  In state PATTERN, SETUPDEPTH and READY this 
          moves the endeffector to 0. Can be used for adjustments. Stops any running
          pattern and ends in state READY.
          @param speed  Speed in mm/s used for driving to min. 
                        Defaults to 10.0 mm/s
          @return TRUE on success, FALSE if state does not allow this.
        */
        /**************************************************************************/
        bool moveToMin(float speed = 10.0);

        /**************************************************************************/
        /*!
          @brief  In state PATTERN and READY this moves the endeffector
          to DEPTH and enters state SETUPDEPTH. Follows the DEPTH postion 
          whenever setDepth() is called. Can be used for adjustments. Stops any running
          pattern. 
          @param speed  Speed in mm/s used for driving to min. 
                        Defaults to 10.0 mm/s
          @param fancy  In fancy mode sensation allows to adjust both, depth and 
                        stroke. +100 adjusts the depth position, -100 adjusts the
                        stroke position. 0 adjusts the midpoint depth-stroke/2.
          @return TRUE on success, FALSE if state does not allow this.
        */
        /**************************************************************************/
        bool setupDepth(float speed = 10.0, bool fancy = false);

        /**************************************************************************/
        /*!
          @brief  Retrieves the current servo state from the internal state machine.
          @return Current state of the state machine
        */
        /**************************************************************************/
        ServoState getState();

        /**************************************************************************/
        /*!
          @brief  Disables the servo motor instantly and deletes any motion task. 
          Sets state machine to UNDEFINED. Must be followed by homing to enable
          servo again. 
        */
        /**************************************************************************/
        void disable();

        /**************************************************************************/
        /*!
          @brief  Makes the pattern list available for the main program to retreive 
          informations like pattern names.
          @param index index of a pattern.
          @return String holding a pattern name with a certain index. If index is 
                        out of range it returns "Invalid"
        */
        /**************************************************************************/
        String getPatternName(int index);

        /**************************************************************************/
        /*!
          @brief  Makes the pattern list available for the main program to retreive 
          informations like pattern names.
          @return The number of pattern available.
        */
        /**************************************************************************/
        unsigned int getNumberOfPattern() { 
          return patternTableSize; 
        };

        /**************************************************************************/
        /*!
          @brief  Updates the maximum speed number of StrokeEngine. This value is 
          used to keep alle motions in check and as a safeguard. 
          @param maxSpeed maximum Speed in mm/s
        */
        /**************************************************************************/
        void setMaxSpeed(float maxSpeed);

        /**************************************************************************/
        /*!
          @brief  Get the current set maximum speed
          @return maximum speed in mm/s
        */
        /**************************************************************************/
        float getMaxSpeed();

        /**************************************************************************/
        /*!
          @brief   Updates the maximum acceleration number of StrokeEngine. This value 
          is used to keep alle motions in check and as a safeguard.
          @param maxAcceleration maximum acceleration in mm/s²
        */
        /**************************************************************************/
        void setMaxAcceleration(float maxAcceleration);

        /**************************************************************************/
        /*!
          @brief  Get the current set maximum acceleration
          @return maximum acceleration in mm/s²
        */
        /**************************************************************************/
        float getMaxAcceleration();

        /**************************************************************************/
        /*!
          @brief  Register a callback function that will update telemetry information
          about StrokeEngine. The provided function will be called whenever a motion 
          is executed by a manual command or by a pattern. The returned values are the
          target position of this move, its top speed and wether clipping occurred. 
          @param callbackTelemetry Function must be of type: 
          void callbackTelemetry(float position, float speed, bool clipping)
        */
        /**************************************************************************/
        void registerTelemetryCallback(void(*callbackTelemetry)(float, float, bool));

    protected:
        ServoState _state = UNDEFINED;
        motorProperties *_motor;
        machineGeometry *_physics;
        float _travel;
        int _minStep;
        int _maxStep;
        int _maxStepPerSecond;
        int _maxStepAcceleration;
        int _patternIndex = 0;
        bool _isHomed = false;
        int _index = 0;
        int _depth;
        int _previousDepth;
        int _stroke;
        int _previousStroke;
        float _timeOfStroke;
        float _linearSpeedMmPerSecond = 0.0f;
        int _linearSpeedStepsPerSecond = 1;
        float _sensation;
        bool _applyUpdate = false;
        bool _abortHoming = false;
        MotionDriver *_driver = NULL;
        bool _ownsDriver = false;
        int _defaultStopAcceleration = 0;
        float _motionAccelerationFactor = 1.0f;
        float _reversalAccelerationFactor = 0.35f;
        int _endpointDwellMillis = 10;
        int _lastMoveDirection = 0;
        volatile bool _stopRequested = false;
        volatile bool _stopRequestEmergency = false;
        volatile int _requestedStopAcceleration = 0;
        static void _homingProcedureImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_homingProcedure(); }
        void _homingProcedure();
        void _sensorHomingProcedure();
        void _sensorlessHomingProcedure();
        static void _strokingImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_stroking(); }
        void _stroking();
        static void _streamingImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_streaming(); }
        void _streaming();
        TaskHandle_t _taskStrokingHandle = NULL;
        TaskHandle_t _taskHomingHandle = NULL;
        TaskHandle_t _taskStreamingHandle = NULL;
        SemaphoreHandle_t _patternMutex = xSemaphoreCreateMutex();
        void _applyMotionProfile(motionParameter* motion);
        void _injectCurrentPatternSettings();
        void _updatePatternTimeFromLinearSpeed();
        int _resolveStopAcceleration(float decelerationMmPerSecondSquared);
        int _limitStopAccelerationForTravel(int requestedAcceleration);
        void _beginStop(int stopAcceleration, bool emergency);
        void _completeStop();
        void(*_callBackHomeing)(bool) = NULL;
        void(*_callbackTelemetry)(float, float, bool) = NULL;
        bool _sensorlessHomeing;
        int _homeingSpeed;
        int _homeingPin;
        int _sensorlessHomeingCurrentPin;
        float _sensorlessHomeingCurrentLimit;
        int _homeingToBack;
        bool _homeingActiveLow;      /*> Polarity of the homing signal*/
        bool _fancyAdjustment;
        void _setupDepths();
        float _getAnalogAveragePercent(int pinNumber, int samples);
};
