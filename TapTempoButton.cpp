#include "TapTempoButton.h"


TapTempoButton::TapTempoButton() :
    tempo(0),
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

float TapTempoButton::getTempo() {
  return this->tempo;
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

  this->tempo = (60000*this->lastTapTime) / (float)timeDifferences; // timeDiff / lastTapTime = average of the time diff between taps
  if (this->tempo > TAPTEMPO_MAXTEMPO)
    this->tempo = TAPTEMPO_MAXTEMPO;
  return true;
}

bool TapTempoButton::isPressedThisFrame() {
  return this->button.isPressedThisFrame();
}

bool TapTempoButton::isPressed() {
  return this->button.isPressed();
}