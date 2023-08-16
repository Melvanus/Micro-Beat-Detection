#ifndef __Button_h__
#define __Button_h__

#include <Arduino.h>

#define BUTTON_DEBOUNCE_TIME 40// x ms after button has changed to avoid double clicks


class Button {
  private:
    byte pin;
    bool lastReadState;
    bool readStateThisFrame;
    bool currentState;
    bool thisFrameState;
    unsigned long timeButtonChange;
    unsigned long debounceTime = 30;

  public:
    Button();
    void begin(byte pin); 
    void update(); // Must be called before following functions
    bool isPressedThisFrame();
    bool isPressed();
    bool peek(); // Returns the actual sensor state even before a press is registered
};

#endif