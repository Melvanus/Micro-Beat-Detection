#ifndef __TapTempoButton_h__
#define __TapTempoButton_h__

#include "Button.h"

#define   TAPTEMPO_AMOUNT_OF_TAPS       6     // tempo will be averaged over x taps
#define   TAPTEMPO_TIMEBETWEEN          2000  // in ms  (2s => 0.5bps = 30bpm is the min tempo)
#define   TAPTEMPO_MINTEMPO             30   // bpm
#define   TAPTEMPO_MAXTEMPO             400   // bpm
#define   TAPTEMPO_EPSILON              0.01

class TapTempoButton {
  private:
    Button button;
    float tempo;
    unsigned long lastTapTime;
    unsigned long tapTimes[TAPTEMPO_AMOUNT_OF_TAPS];

  public:
    TapTempoButton();

    void begin(byte pin); 
    void update(); // Must be called before following functions
    bool hasTempoChanged(); // has to be call regularly to check the button status
    bool isPressedThisFrame();
    bool isPressed();
    bool resetTapMeasurement();
    float getTempo();
    void setTempo(float tempo);
    int getBeatDurationMillis();
    
};

#endif