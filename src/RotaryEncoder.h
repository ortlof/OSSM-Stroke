#include <stdint.h> 
#pragma once

class Encoder;

class RotaryEncoder
{
  

  private:
    Encoder *encoder;
  
  public:
    RotaryEncoder();
    int check_button();
    bool wasTurnedLeft();
    bool wasTurnedRight();
    bool isPressed();
    bool isReleased();
    bool wasPressed();
    bool wasLongPressed();
};