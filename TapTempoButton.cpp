#include "TapTempoButton.h"


TapTempoButton::TapTempoButton() :
    tempo(124),
    lastTapTime(0),
    tapTimes{0} {
}

void TapTempoButton::begin(byte pin) {  
  //pinMode(pin, INPUT_PULLUP);
  this->button.begin(pin);
}

void TapTempoButton::update(){
  this->button.update();
}

bool TapTempoButton::hasTempoChanged() { // to call once per loop()
  if (!this->isPressedThisFrame())
    return false;

  unsigned long now = millis();
  if (now - this->tapTimes[this->lastTapTime] > TAPTEMPO_TIMEBETWEEN) { // start over
    this->tapTimes[0] = now;
    this->lastTapTime = 0;
    return false;
  }
 
  // has been tapped at least once
  if (this->lastTapTime < TAPTEMPO_AMOUNT_OF_TAPS-1) {
    this->lastTapTime++;
    this->tapTimes[this->lastTapTime] = now;
  }
  else { // we have TAPTEMPO_AMOUNT_OF_TAPS taps, we need to shift all the values
    for (byte i = 1 ; i <= this->lastTapTime ; i++)
      this->tapTimes[i-1] = this->tapTimes[i];
    this->tapTimes[this->lastTapTime] = now;
  }

  long timeDifferences = 0;
  for (byte j = 1 ; j <= this->lastTapTime ; j++)
    timeDifferences += this->tapTimes[j] - this->tapTimes[j-1];

  float newTempo = (60000*this->lastTapTime) / (float)timeDifferences; // timeDiff / lastTapTime = average of the time diff between taps
  newTempo = constrain(newTempo, TAPTEMPO_MINTEMPO, TAPTEMPO_MAXTEMPO);
  float diff = abs(this->tempo - newTempo);
  if (diff > TAPTEMPO_EPSILON) {
    this->setTempo(newTempo);
    return true;
  }
  
  return false;
}

bool TapTempoButton::isPressedThisFrame() {
  return this->button.isPressedThisFrame();
}

bool TapTempoButton::isPressed() {
  return this->button.isPressed();
}

bool TapTempoButton::resetTapMeasurement() {
  this->tapTimes[0] = 0;
  this->lastTapTime = 0;
}

int TapTempoButton::getBeatDurationMillis() {
  return (int) (60000.0 / this->tempo);
}

float TapTempoButton::getTempo() { 
  return this->tempo;
}

void TapTempoButton::setTempo(float tempo) {
  tempo = constrain(tempo, TAPTEMPO_MINTEMPO, TAPTEMPO_MAXTEMPO);
  this->tempo = tempo;
}
