#include "Button.h"

Button::Button() :
  lastReadState(LOW),
  currentState(LOW),
  pin(NOT_A_PIN),
  timeButtonChange(0) {
}

void Button::begin(byte pin) {  
  pinMode(pin, INPUT);
  bool st = digitalRead(pin);
  this->lastReadState = st;
  this->currentState = st;
  this->pin = pin;
}

void Button::update() { 
  this->thisFrameState = false; 
  bool newState = digitalRead(this->pin);
  unsigned long currentTime = millis();
  if (newState != this->lastReadState) { // has changed - because of noise or because the button is being pressed
    this->timeButtonChange = currentTime;
    this->lastReadState = newState;
  }
  else if ((currentTime - this->timeButtonChange) > debounceTime) { // state has been stable for x ms
    if (this->currentState == LOW && newState == HIGH){
      this->thisFrameState = true;
    }
    this->currentState = newState;
  }
}

bool Button::isPressedThisFrame() {
  return this->thisFrameState;
}

bool Button::isPressed() {
  return this->currentState;
}

bool Button::peek() {
  return this->lastReadState;
}