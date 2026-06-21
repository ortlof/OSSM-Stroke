/**
 *   Patterns of the StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine 
 *
 * Copyright (C) 2021 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#pragma once

#include <Arduino.h>
#include <StrokeEngine.h>
#include <math.h>
#include <string.h>
#include "PatternMath.h"

#define DEBUG_PATTERN                 // Print some debug informations over Serial

#ifndef STRING_LEN
  #define STRING_LEN           64     // Bytes used to initialize char array. No path, topic, name, etc. should exceed this value
#endif

/**************************************************************************/
/*!
  @brief  struct to return all parameters FastAccelStepper needs to calculate
  the trapezoidal profile.
*/
/**************************************************************************/
typedef struct {
    int stroke;         //!< Absolute and properly constrainted target position of a move in steps 
    int speed;          //!< Speed of a move in Steps/second 
    int acceleration;   //!< Acceleration to get to speed or halt 
    bool skip;          //!< no valid stroke, skip this set an query for the next --> allows pauses between strokes
} motionParameter;


/**************************************************************************/
/*!
  @class Pattern 
  @brief  Base class to derive your pattern from. Offers a unified set of
          functions to store all relevant paramteres. These function can be
          overridenid necessary. Pattern should be self-containted and not 
          rely on any stepper/servo related properties. Internal book keeping
          is done in steps. The translation from real word units to steps is
          provided by the StrokeEngine. Also the sanity check whether motion
          parameters are physically possible are done by the StrokeEngine. 
          Imposible motion commands are clipped, cropped or adjusted while 
          still having a smooth appearance.  
*/
/**************************************************************************/
class Pattern {

    public:
        //! Constructor
        /*!
          @param str String containing the name of a pattern 
        */
        Pattern(const char *str) { strcpy(_name, str); }

        //! Set the time a normal stroke should take to complete
        /*! 
          @param speed time of a full stroke in [sec] 
        */
        virtual void setTimeOfStroke(float speed) { _timeOfStroke = speed; }

        //! Set the maximum stroke a pattern may have
        /*! 
          @param stroke stroke distance in Steps 
        */
        virtual void setStroke(int stroke) { _stroke = stroke; }

        //! Set the maximum depth a pattern may have
        /*! 
          @param stroke stroke distance in Steps 
        */
        virtual void setDepth(int depth) { _depth = depth; }

        //! Sensation is an additional parameter a pattern can take to alter its behaviour
        /*! 
          @param sensation Arbitrary value from -100 to 100, with 0 beeing neutral 
        */
        virtual void setSensation(float sensation) { _sensation = sensation; } 

        //! Add a short pause after a move has completed and before the next move is commanded.
        /*!
          @param dwellMillis Pause time in milliseconds. Clamped to 0..500 ms.
        */
        virtual void setEndpointDwellMillis(int dwellMillis) {
            _endpointDwellMillis = constrain(dwellMillis, 0, 500);
            _endpointDwellIndex = 0xffffffffUL;
        }

        //! Retrives the name of a pattern
        /*! 
          @return c_string containing the name of a pattern 
        */
        char *getName() { return _name; }

        //! Calculate the position of the next stroke based on the various parameters
        /*! 
          @param index index of a stroke. Increments with every new stroke. 
          @return Set of motion parameteres like speed, acceleration & position
        */
        virtual motionParameter nextTarget(unsigned int index) {
            _index = index;
            return _nextMove;
        } 

        //! Communicates the maximum possible speed and acceleration limits of the machine to a pattern.
        /*! 
          @param maxSpeed maximum speed which is possible. Higher speeds get truncated inside StrokeEngine anyway.
          @param maxAcceleration maximum possible acceleration. Get also truncated, if impossible.
          @param stepsPerMM 
        */
        virtual void setSpeedLimit(unsigned int maxSpeed, unsigned int maxAcceleration, unsigned int stepsPerMM) { _maxSpeed = maxSpeed; _maxAcceleration = maxAcceleration; _stepsPerMM = stepsPerMM; } 

    protected:
        int _stroke;
        int _depth;
        float _timeOfStroke;
        float _sensation = 0.0;
        int _index = -1;
        char _name[STRING_LEN]; 
        motionParameter _nextMove = {0, 0, 0, false};
        int _startDelayMillis = 0;
        int _delayInMillis = 0;
        unsigned int _maxSpeed = 0;
        unsigned int _maxAcceleration = 0;
        unsigned int _stepsPerMM = 0;
        int _endpointDwellMillis = 0;
        unsigned long _endpointDwellStartMillis = 0;
        unsigned long _endpointDwellIndex = 0xffffffffUL;

        /*!
          @brief Start a delay timer which can be polled by calling _isStillDelayed(). 
          Uses internally the millis()-function.
        */
        void _startDelay() {
            _startDelayMillis = millis();
        } 

        /*! 
          @brief Update a delay timer which can be polled by calling _isStillDelayed(). 
          Uses internally the millis()-function.
          @param delayInMillis delay in milliseconds 
        */
        void _updateDelay(int delayInMillis) {
            _delayInMillis = delayInMillis;
        } 

        /*! 
          @brief Poll the state of a internal timer to create pauses between strokes. 
          Uses internally the millis()-function.
          @return True, if the timer is running, false if it is expired.
        */
        bool _isStillDelayed() {
            return (millis() > (_startDelayMillis + _delayInMillis)) ? false : true; 
        }

        /*!
          @brief Dwell before issuing the next pattern move. StrokeEngine calls
                 nextTarget() only after the previous move has completed, so this
                 pauses exactly at the reversal point. The first move is not
                 delayed.
          @return True while the pattern should skip and wait.
        */
        bool _applyEndpointDwell(unsigned int index) {
            if (_endpointDwellMillis <= 0 || index == 0) {
                _nextMove.skip = false;
                return false;
            }

            if (_endpointDwellIndex != index) {
                _endpointDwellIndex = index;
                _endpointDwellStartMillis = millis();
                _nextMove.skip = true;
                return true;
            }

            bool stillDwelling = ((unsigned long)(millis() - _endpointDwellStartMillis) < (unsigned long)_endpointDwellMillis);
            _nextMove.skip = stillDwelling;
            return stillDwelling;
        }

};

/**************************************************************************/
/*!
  @brief  Simple Stroke Pattern. It creates a trapezoidal stroke profile
  with 1/3 acceleration, 1/3 coasting, 1/3 deceleration. Sensation has 
  no effect.
*/
/**************************************************************************/
class SimpleStroke : public Pattern {
    public:
        SimpleStroke(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }
            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = _depth;
            }

            _index = index;
            return _nextMove;
        }
};

/**************************************************************************/
/*!
  @brief  Simple pattern where the sensation value can change the speed 
  ratio between in and out. Sensation > 0 make the in move faster (up to 5x)
  giving a hard pounding sensation. Values < 0 make the out move going 
  faster. This gives a more pleasing sensation. The time for the overall 
  stroke remains the same. 
*/
/**************************************************************************/
class TeasingPounding : public Pattern {
    public:
        TeasingPounding(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateStrokeTiming();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateStrokeTiming();
        }
        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }
            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfOutStroke);

                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfOutStroke);    
                _nextMove.stroke = _depth - _stroke;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfInStroke); 
     
                // acceleration to meet the profile            
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfInStroke);    
                _nextMove.stroke = _depth;
            }
            _index = index;
            return _nextMove;
        }
    protected:
        float _timeOfFastStroke = 1.0;
        float _timeOfInStroke = 1.0;
        float _timeOfOutStroke = 1.0;
        void _updateStrokeTiming() {
            // calculate the time it takes to complete the faster stroke
            // Division by 2 because reference is a half stroke
            _timeOfFastStroke = (0.5 * _timeOfStroke) / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0);
            // positive sensation, in is faster
            if (_sensation > 0.0) {
                _timeOfInStroke = _timeOfFastStroke;
                _timeOfOutStroke = _timeOfStroke - _timeOfFastStroke;
            // negative sensation, out is faster
            } else {
                _timeOfOutStroke = _timeOfFastStroke;
                _timeOfInStroke = _timeOfStroke - _timeOfFastStroke;
            }
#ifdef DEBUG_PATTERN
            Serial.println("TimeOfInStroke: " + String(_timeOfInStroke));
            Serial.println("TimeOfOutStroke: " + String(_timeOfOutStroke));
#endif
        }
};


/**************************************************************************/
/*!
  @brief  Robot Stroke Pattern. Sensation controls the acceleration of the
  stroke. Positive value increase acceleration until it is a constant speed
  motion (feels robotic). Neutral is equal to simple stroke (1/3, 1/3, 1/3).
  Negative reduces acceleration into a triangle profile.
*/
/**************************************************************************/ 
class RoboStroke : public Pattern {
    public:
        RoboStroke(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }

        void setSensation(float sensation = 0) { 
            _sensation = sensation;
            // scale sensation into the range [0.05, 0.5] where 0 = 1/3
            if (sensation >= 0 ) {
              _x = fscale(0.0, 100.0, 1.0/3.0, 0.5, sensation, 0.0);
            } else {
              _x = fscale(0.0, 100.0, 1.0/3.0, 0.05, -sensation, 0.0);
            }
#ifdef DEBUG_PATTERN
            Serial.println("Sensation:" + String(sensation,0) + " --> " + String(_x,6));
#endif
        }

        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }
            // maximum speed of the trapezoidal motion
            float speed = float(_stroke) / ((1 - _x) * _timeOfStroke);
            _nextMove.speed = int(speed); 

            // acceleration to meet the profile
            _nextMove.acceleration = int(speed / (_x * _timeOfStroke));

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = _depth;
            }

            _index = index;
            return _nextMove;
        }
    protected:
        float _x = 1.0/3.0;
};

/**************************************************************************/
/*!
  @brief  Like Teasing or Pounding, but every second stroke is only half the
  depth. The sensation value can change the speed ratio between in and out. 
  Sensation > 0 make the in move faster (up to 5x) giving a hard pounding 
  sensation. Values < 0 make the out move going faster. This gives a more 
  pleasing sensation. The time for the overall stroke remains the same for
  all strokes, even half ones. 
*/
/**************************************************************************/
class HalfnHalf : public Pattern {
    public:
        HalfnHalf(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateStrokeTiming();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateStrokeTiming();
        }
        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }
            // check if this is the very first 
            if (index == 0) {
              //pattern started for the very fist time, so we start gentle with a half move
              _half = true;
            }

            // set-up the stroke length
            int stroke = _stroke;
            if (_half == true) {
                // half the stroke length
                stroke = _stroke / 2;
            } 

            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * stroke/_timeOfOutStroke);  

                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfOutStroke);    
                _nextMove.stroke = _depth - _stroke;
                // every second move is half
                _half = !_half;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * stroke/_timeOfInStroke);  
     
                // acceleration to meet the profile            
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfInStroke);    
                _nextMove.stroke = (_depth - _stroke) + stroke;  
            }
            _index = index;
            return _nextMove;
        }
    protected:
        float _timeOfFastStroke = 1.0;
        float _timeOfInStroke = 1.0;
        float _timeOfOutStroke = 1.0;
        bool _half = true;
        void _updateStrokeTiming() {
            // calculate the time it takes to complete the faster stroke
            // Division by 2 because reference is a half stroke
            _timeOfFastStroke = (0.5 * _timeOfStroke) / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0);
            // positive sensation, in is faster
            if (_sensation > 0.0) {
                _timeOfInStroke = _timeOfFastStroke;
                _timeOfOutStroke = _timeOfStroke - _timeOfFastStroke;
            // negative sensation, out is faster
            } else {
                _timeOfOutStroke = _timeOfFastStroke;
                _timeOfInStroke = _timeOfStroke - _timeOfFastStroke;
            }
#ifdef DEBUG_PATTERN
            Serial.println("TimeOfInStroke: " + String(_timeOfInStroke));
            Serial.println("TimeOfOutStroke: " + String(_timeOfOutStroke));
#endif
        }
};

/**************************************************************************/
/*!
  @brief  The insertion depth ramps up gradually with each stroke until it
  reaches its maximum. It then resets and restars. Sensations controls how 
  many strokes there are in a ramp.
*/
/**************************************************************************/
class Deeper : public Pattern {
    public:
        Deeper(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        void setSensation(float sensation) { 
            _sensation = sensation;

            // maps sensation to useful values [2,22] with 12 beeing neutral
            if (sensation < 0) {
                _countStrokesForRamp = map(sensation, -100, 0, 2, 11);
            } else {
                _countStrokesForRamp = map(sensation, 0, 100, 11, 32);
            }
#ifdef DEBUG_PATTERN
            Serial.println("_countStrokesForRamp: " + String(_countStrokesForRamp));
#endif
        }

        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }
            // How many steps is each stroke advancing         
            int slope = _stroke / (_countStrokesForRamp);

            // The pattern recycles so we use modulo to get a cycling index.
            // Factor 2 because index increments with each full stroke twice
            // add 1 because modulo = 0 is index = 1
            int cycleIndex = (index / 2) % _countStrokesForRamp + 1;

            // This might be not smooth, as the insertion depth may jump when 
            // sensation is adjusted.

            // Amplitude is slope * cycleIndex
            int amplitude = slope * cycleIndex;
#ifdef DEBUG_PATTERN
            Serial.println("amplitude: " + String(amplitude)
                         + " cycleIndex: " + String(cycleIndex));
#endif

            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * amplitude/_timeOfStroke); 

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = (_depth - _stroke) + amplitude;
            }

            _index = index;
            return _nextMove;
        }
    
    protected:
        int _countStrokesForRamp = 2;

};

/**************************************************************************/
/*!
  @brief  Pauses between a series of strokes. 
  The number of strokes ramps from 1 stroke to 5 strokes and back. Sensation 
  changes the length of the pauses between stroke series.
*/
/**************************************************************************/
class StopNGo : public Pattern {
    public:
        StopNGo(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        void setSensation(float sensation) { 
            _sensation = sensation;

            // maps sensation to a delay from 100ms to 10 sec
            _updateDelay(map(sensation, -100, 100, 100, 10000));
        }

        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }
            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * _stroke/_timeOfStroke); 

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // adds a delay between each stroke
            if (_isStillDelayed() == false) {

                // odd stroke is moving out    
                if (index % 2) {
                    _nextMove.stroke = _depth - _stroke;

                    if (_strokeIndex >= _strokeSeriesIndex) {
                        // Reset stroke index to 1
                        _strokeIndex = 0;

                        // change count direction once we reached the maximum number of strokes
                        if (_strokeSeriesIndex >= _numberOfStrokes) {
                            _countStrokesUp = false;
                        }

                        // change count direction once we reached one stroke counting down
                        if (_strokeSeriesIndex <= 1) {
                            _countStrokesUp = true;
                        }

                        // increment or decrement strokes counter
                        if (_countStrokesUp == true) {
                            _strokeSeriesIndex++;
                        } else {
                            _strokeSeriesIndex--;
                        }

                        // start delay after having moved out
                        _startDelay();
                    }
                    

                // even stroke is moving in
                } else {
                    _nextMove.stroke = _depth- _stroke/2;
                    // Increment stroke index by one
                    _strokeIndex++;
                }
                _nextMove.skip = false;
            } else {
                _nextMove.skip = true;
            }

            _index = index;
            
            return _nextMove;
        }

    protected:
        int _numberOfStrokes = 5;
        int _strokeSeriesIndex = 1;
        int _strokeIndex = 0;
        bool _countStrokesUp = true;

};

/**************************************************************************/
/*!
  @brief  Sensation reduces the effective stroke length while keeping the
  stroke speed constant to the full stroke. This creates interesting 
  vibrational pattern at higher sensation values. With positive sensation the
  strokes will wander towards the front, with negative values towards the back.
*/
/**************************************************************************/
class Insist : public Pattern {
    public:
        Insist(const char *str) : Pattern(str) {}   

        void setSensation(float sensation) { 
            _sensation = sensation;

            // make invert sensation and make into a fraction of the stroke distance
            _strokeFraction = (100 - abs(sensation))/100.0f;

            _strokeInFront = (sensation > 0) ? true : false;

            _updateStrokeTiming();
        }

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed;
            _updateStrokeTiming();
        }   

        void setStroke(int stroke) {
            _stroke = stroke;
            _updateStrokeTiming();
        }

        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }

            // acceleration & speed to meet the profile
            _nextMove.acceleration = _acceleration;
            _nextMove.speed = _speed;

            if (_strokeInFront) {
                // odd stroke is moving out
                if (index % 2) {
                    _nextMove.stroke = _depth - _realStroke;

                // even stroke is moving in
                } else {
                    _nextMove.stroke = _depth;  
                }

            } else {
                // odd stroke is moving out    
                if (index % 2) {
                    _nextMove.stroke = _depth - _stroke;
                    
                // even stroke is moving in
                } else {
                    _nextMove.stroke = (_depth - _stroke) + _realStroke;                
                }
            }

            _index = index;
            
            return _nextMove;
        }

    protected:
        int _speed = 0;
        int _acceleration = 0;
        int _realStroke = 0;
        float _strokeFraction = 1.0;
        bool _strokeInFront = false;
        void _updateStrokeTiming() {
            // maximum speed of the longest trapezoidal motion (full stroke)
            _speed = int(1.5 * _stroke/_timeOfStroke);

            // Acceleration to hold 1/3 profile with fractional strokes
            _acceleration = int(3.0 * _speed/(_timeOfStroke * _strokeFraction));

            // Calculate fractional stroke length
            _realStroke = int((float)_stroke * _strokeFraction);
        }

};

/**************************************************************************/
/*!
  @brief  Vibrational pattern that works like a jack hammer. Vibrates on the 
  way in and pulls out smoothly in one go. Sensation sets the vibration 
  amplitude.
*/
/**************************************************************************/
class JackHammer : public Pattern {
    public:
        JackHammer(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateVibrationParameters();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = 0.5 * speed;
            _updateVibrationParameters();
        }
        void setStroke(int stroke) { 
            _stroke = stroke;
            _updateVibrationParameters();
        }
        void setSpeedLimit(unsigned int maxSpeed, unsigned int maxAcceleration, unsigned int stepsPerMM) { 
            _maxSpeed = maxSpeed; 
            _maxAcceleration = maxAcceleration; 
            _stepsPerMM = stepsPerMM;
            _updateVibrationParameters(); 
        }
        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }

            // revert position for the first move or if depth is exceeded
            if (index == 0 || _nextMove.stroke >= _depth) {
                // Return strokes goes at regular speed without vibration back to 0

                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);  
                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfStroke);  
                // all they way out to start
                _nextMove.stroke = _depth - _stroke;
                // we are done here
                return _nextMove;
            }

            // only calculate new position, if index has incremented: no mid-stroke update, as vibration is sufficiently fast
            // except for return stroke if depth is exceeded.
            if (index != _index) {
             
                // Vibration happens at maximum speed and acceleration of the machine
                _nextMove.speed = _maxSpeed;
                _nextMove.acceleration = _maxAcceleration;

                // odd stroke is shaking out
                if (index % 2) {  
                    _nextMove.stroke = _nextMove.stroke - _outVibrationDistance;
                // even stroke is shaking in
                } else {
                    // limit range to _depth
                    _nextMove.stroke = min((_nextMove.stroke + _inVibrationDistance), _depth);
                }
            }
            _index = index;
            return _nextMove;
        }
    protected:
        int _inVibrationDistance = 0;
        int _outVibrationDistance = 0;
        int _strokeInSpeed = 0;
        void _updateVibrationParameters() {
            // Hammering in takes considerable longer then backing off
            _strokeInSpeed = int(0.5 * _stroke/_timeOfStroke);

            // Scale vibration amplitude from 1mm to 15mm with sensation
            _inVibrationDistance = (int)fscale(-100.0, 100.0, (float)(3.0*_stepsPerMM), (float)(25.0*_stepsPerMM), _sensation, 0.0);

            /* Calculate _outVibrationDistance to match with stroking speed
               d_out = d_in * (v_vib - v_stroke) / (v_vib + v_stroke)
               Formula neglects acceleration. Real timing will be slower due to finite acceleration & deceleration
            */
           _outVibrationDistance = _inVibrationDistance * (_maxSpeed - _strokeInSpeed) / (_maxSpeed + _strokeInSpeed);

#ifdef DEBUG_PATTERN
            Serial.println("_maxSpeed: " + String(_maxSpeed) + " _strokeInSpeed: " + String(_strokeInSpeed)  + " _strokeOutSpeed: " + String(int(1.5 * _stroke/_timeOfStroke)));
            Serial.println("inDist: " + String(_inVibrationDistance) + " outDist: " + String(_outVibrationDistance));
#endif
            
        }
};

/**************************************************************************/
/*!
  @brief  Simple vibrational overlay pattern. Vibrates on the way in and out. 
  Sensation sets the vibration amplitude.
*/
/**************************************************************************/
class StrokeNibbler : public Pattern {
    public:
        StrokeNibbler(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateVibrationParameters();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateVibrationParameters();
        }
        void setStroke(int stroke) { 
            _stroke = stroke;
            _updateVibrationParameters();
        }
        void setSpeedLimit(unsigned int maxSpeed, unsigned int maxAcceleration, unsigned int stepsPerMM) { 
            _maxSpeed = maxSpeed; 
            _maxAcceleration = maxAcceleration; 
            _stepsPerMM = stepsPerMM;
            _updateVibrationParameters(); 
        }
        motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }

            // revert position to start for the first stroke
            if (index == 0) {
                // Set motion parameter
                _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);
                _nextMove.acceleration = _maxAcceleration;

                // go to back position
                _nextMove.stroke = _depth - _stroke;

                // store index and return
                _index = index;
                return _nextMove;
            }

            // Vibration happens at maximum speed and acceleration of the machine
            _nextMove.speed = _maxSpeed;
            _nextMove.acceleration = _maxAcceleration;

            // check if we have reached one of the ends and reverse direction
            if (_nextMove.stroke >= _depth) {
                _returnStroke = true;
            } 
            if (_nextMove.stroke <= (_depth - _stroke)) {
                _returnStroke = false;
            }

            // only calculate new position, if index has incremented: no mid-stroke update, as vibration is sufficiently fast
            if (index != _index) {
                if (_returnStroke == true) {
                    // long vibration distance on way out
                    // odd stroke is shaking out
                    if (index % 2) {  
                        // limit stroke to _depth - _stroke
                        _nextMove.stroke = max(_nextMove.stroke - _inVibrationDistance, _depth - _stroke);

                    // even stroke is shaking in
                    } else {
                        _nextMove.stroke = _nextMove.stroke + _outVibrationDistance;
                    }

                } else {
                    // long vibration distance on way in
                    // odd stroke is shaking out
                    if (index % 2) {  
                        _nextMove.stroke = _nextMove.stroke - _outVibrationDistance;

                    // even stroke is shaking in
                    } else {
                        // limit stroke to _depth
                        _nextMove.stroke = min(_nextMove.stroke + _inVibrationDistance, _depth);
                    }
                }
            }

            _index = index;
            return _nextMove;
        }
    protected:
        bool _returnStroke = false;
        int _inVibrationDistance = 0;
        int _outVibrationDistance = 0;
        int _strokeSpeed = 0;
        void _updateVibrationParameters() {
            // Empirical factor to compensate time losses due to finite acceleration.
            _strokeSpeed = int(5.0 * _stroke/_timeOfStroke);

            // Scale vibration amplitude from 3mm to 25mm with sensation
            _inVibrationDistance = (int)fscale(-100.0, 100.0, (float)(3.0*_stepsPerMM), (float)(25.0*_stepsPerMM), _sensation, 0.0);

            /* Calculate _outVibrationDistance to match with stroking speed
               d_out = d_in * (v_vib - v_stroke) / (v_vib + v_stroke)
               Formula neglects acceleration. Real timing will be slower due to finite acceleration & deceleration
            */
           _outVibrationDistance = _inVibrationDistance * (_maxSpeed - _strokeSpeed) / (_maxSpeed + _strokeSpeed);

#ifdef DEBUG_PATTERN
            Serial.println("_maxSpeed: " + String(_maxSpeed) + " _strokeSpeed: " + String(_strokeSpeed));
            Serial.println("inDist: " + String(_inVibrationDistance) + " outDist: " + String(_outVibrationDistance));
#endif
            
        }
};

/**************************************************************************/
/*!
  @brief Knot pattern with sensation-controlled slow-start position.

  Sensation mapping:
    -100 => slow part starts at 5% of the active stroke
       0 => slow part starts at 50% of the active stroke
     100 => slow part starts at 95% of the active stroke

  The 5%/95% clamp keeps both the fast and slow portions visible. Without it,
  the pattern can legitimately command a zero-length fast or slow move, which
  looks like the machine has stopped even when the code is technically doing
  exactly what it was told. Machines are annoying like that.

  StrokeEngine passes seconds per full stroke to setTimeOfStroke(). This class
  stores the half-stroke time internally, matching the other patterns.
*/
/**************************************************************************/

#ifndef KNOT_MIN_SPEED_INPUT
#define KNOT_MIN_SPEED_INPUT 1.0f
#endif

#ifndef KNOT_MAX_SPEED_INPUT
#define KNOT_MAX_SPEED_INPUT 6000.0f
#endif

#ifndef KNOT_MIN_TIME_OF_STROKE
#define KNOT_MIN_TIME_OF_STROKE 0.005f
#endif

#ifndef KNOT_FAST_SPEED_FACTOR
#define KNOT_FAST_SPEED_FACTOR 1.1f
#endif

#ifndef KNOT_SLOW_SPEED_FACTOR
#define KNOT_SLOW_SPEED_FACTOR 0.03f
#endif

#ifndef KNOT_FAST_ACCEL_FACTOR
#define KNOT_FAST_ACCEL_FACTOR 1.50f
#endif

#ifndef KNOT_SLOW_ACCEL_FACTOR
#define KNOT_SLOW_ACCEL_FACTOR 2.30f
#endif

#ifndef KNOT_RETURN_ACCEL_FACTOR
#define KNOT_RETURN_ACCEL_FACTOR 2.00f
#endif

#ifndef KNOT_STOP_SHORT_RATIO
#define KNOT_STOP_SHORT_RATIO 0.92f
#endif

#ifndef KNOT_MIN_SLOW_RATIO_GAP
#define KNOT_MIN_SLOW_RATIO_GAP 0.05f
#endif

class Knot : public Pattern {
public:
    Knot(const char *str) : Pattern(str) {}

    void setTimeOfStroke(float timeOfStroke = 1.0f) {
        // StrokeEngine gives this value as seconds per full stroke, not FPM.
        float safeFullStrokeTime = constrain(timeOfStroke, 0.01f, 120.0f);

        // Other patterns store the time for one half-stroke internally.
        // IMPORTANT: this must stay in seconds. A previous value of 1.0f here
        // forced the Knot pattern to crawl even when the remote speed was high.
        _timeOfStroke = fmaxf(KNOT_MIN_TIME_OF_STROKE, 0.5f * safeFullStrokeTime);

        // Keep FPM only for debug and future tuning.
        _speed = constrain(60.0f / safeFullStrokeTime, KNOT_MIN_SPEED_INPUT, KNOT_MAX_SPEED_INPUT);
    }

    void setSensation(float sensation) {
        _sensation = constrain(sensation, -100.0f, 100.0f);

        // -100 => early slow point, +100 => late slow point.
        // Keep a guaranteed slow section before the stop-short target, otherwise
        // high sensation values can collapse the slow move into nothing. Because
        // apparently even motion profiles need guardrails.
        float mappedRatio = (_sensation + 100.0f) / 200.0f;
        float maxSlowStartRatio = fmaxf(0.05f, _stopShortRatio - KNOT_MIN_SLOW_RATIO_GAP);
        _slowStartRatio = constrain(mappedRatio, 0.05f, maxSlowStartRatio);
    }

    motionParameter nextTarget(unsigned int index) {
            if (_applyEndpointDwell(index)) {
                _index = index;
                return _nextMove;
            }
        _index = index;

        switch (index % 3) {
            case 0:
                // Pull out to the start of the active stroke.
                setMove(calculateStrokeStart(), KNOT_FAST_SPEED_FACTOR, KNOT_RETURN_ACCEL_FACTOR);
                break;

            case 1:
                // Fast travel to the sensation-controlled slow point.
                setMove(calculateSlowStartStroke(), KNOT_FAST_SPEED_FACTOR, KNOT_FAST_ACCEL_FACTOR);
                break;

            case 2:
                // Slow move, but stop short of full depth. The next step pulls out.
                setMove(calculateStopShortStroke(), KNOT_SLOW_SPEED_FACTOR, KNOT_SLOW_ACCEL_FACTOR);

                #ifdef KNOT_DEBUG
                    Serial.println("Knot speed FPM: " + String(_speed));
                    Serial.println("Knot slow start ratio: " + String(_slowStartRatio, 3));
                    Serial.println("Knot stop short ratio: " + String(_stopShortRatio, 3));
                    Serial.println("Knot stroke start target: " + String(calculateStrokeStart()));
                    Serial.println("Knot slow start target: " + String(calculateSlowStartStroke()));
                    Serial.println("Knot stop short target: " + String(calculateStopShortStroke()));
                    Serial.println("Knot depth target: " + String(_depth));
                #endif
                break;
        }

        return _nextMove;
    }

private:
    float _speed = KNOT_MIN_SPEED_INPUT;
    float _slowStartRatio = 0.70f;
    float _stopShortRatio = constrain(KNOT_STOP_SHORT_RATIO, 0.10f, 0.99f);

    int activeStroke() const {
        // Do not let the pattern request positions before the front/min limit.
        return constrain(_stroke, 0, _depth);
    }

    int calculateStrokeStart() const {
        return _depth - activeStroke();
    }

    int calculateSlowStartStroke() const {
        int safeStroke = activeStroke();
        float strokeStart = float(_depth - safeStroke);
        float target = strokeStart + (float(safeStroke) * _slowStartRatio);
        return constrain(int(roundf(target)), 0, _depth);
    }

    int calculateStopShortStroke() const {
        int safeStroke = activeStroke();
        int strokeStart = _depth - safeStroke;
        int slowStart = calculateSlowStartStroke();

        float targetFloat = float(strokeStart) + (float(safeStroke) * _stopShortRatio);
        int target = int(roundf(targetFloat));

        // Keep the stop-short target after the slow-start target.
        target = max(target, slowStart + 1);

        // Keep it genuinely short of full depth when there is enough room.
        if ((_depth > 0) && (target >= _depth) && (safeStroke > 1)) {
            target = _depth - 1;
        }

        return constrain(target, 0, _depth);
    }

    void setMove(int targetStroke, float speedFactor, float accelFactor) {
        float safeTimeOfStroke = fmaxf(KNOT_MIN_TIME_OF_STROKE, _timeOfStroke);
        int safeStroke = max(1, activeStroke());

        int targetSpeed = int(speedFactor * float(safeStroke) / safeTimeOfStroke);
        targetSpeed = max(1, targetSpeed);
        if (_maxSpeed > 0) {
            targetSpeed = min(targetSpeed, int(_maxSpeed));
        }

        int targetAcceleration = int(accelFactor * float(targetSpeed) / safeTimeOfStroke);
        targetAcceleration = max(1, targetAcceleration);
        if (_maxAcceleration > 0) {
            targetAcceleration = min(targetAcceleration, int(_maxAcceleration));
        }

        _nextMove.stroke = constrain(targetStroke, 0, _depth);
        _nextMove.speed = targetSpeed;
        _nextMove.acceleration = targetAcceleration;
        _nextMove.skip = false;
    }
};


/**************************************************************************/
/*
  Array holding all different patterns. Please include any custom pattern here.
*/
/**************************************************************************/
static Pattern *patternTable[] = { 
  new SimpleStroke("Simple Stroke"),
  new TeasingPounding("Teasing or Pounding"),
  new RoboStroke("Robo Stroke"),
  new HalfnHalf("Half'n'Half"),
  new Deeper("Deeper"),
  new StopNGo("Stop'n'Go"),
  new Insist("Insist"),
  new Knot("Knot"),
  //new Wave("Wave"),
  //new JackHammer("Jack Hammer"),
  // new StrokeNibbler("Stroke Nibbler")
  // <-- insert your new pattern class here!
 };

static const unsigned int patternTableSize = sizeof(patternTable) / sizeof(patternTable[0]);
