#include <RotaryEncoder.h>
#include <OSSM_PinDEF.h>
// #define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

#define BTN_NONE   0
#define BTN_SHORT  1
#define BTN_LONG   2
#define BTN_V_LONG 3

RotaryEncoder::RotaryEncoder() {
    encoder = new Encoder(ENC_CLK, ENC_DT);
    pinMode(ENC_SW,   INPUT);
}

int RotaryEncoder::check_button() {
  static bool lastBtn = LOW;
  static unsigned long keyDownTime = 0;
  int btnState = BTN_NONE;
  bool thisBtn = digitalRead(ENC_SW);

  //Detect single presses, no repeating, on keyup
  if(thisBtn == HIGH && lastBtn == LOW){
    keyDownTime = millis();
  }
  
  if (thisBtn == LOW && lastBtn == HIGH) { //there was a keyup
    if((millis()-keyDownTime) >= V_LONG_PRESS_MS){
      btnState = BTN_V_LONG;
    }
    else if((millis()-keyDownTime) >= LONG_PRESS_MS){
      btnState = BTN_LONG;
      }
    else{
      btnState = BTN_SHORT;
      }
    }

  lastBtn = thisBtn;
  return btnState;
}

bool RotaryEncoder::wasTurnedLeft() {
    if (encoder->read() < 0 - ENC_TOL) {
        encoder->write(0);
        return true;
    }
    return false;
}

bool RotaryEncoder::wasTurnedRight() {
    if (encoder->read() > 0 + ENC_TOL) {
        encoder->write(0);
        return true;
    }
    return false;
}