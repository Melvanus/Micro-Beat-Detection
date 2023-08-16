#ifndef __Mode_h__
#define __Mode_h__

enum class Mode {
  BeatDetection,
  Stroboscope,
  SoundIntensity,
  Pulse,
  On,
  Off,
  END_OF_LIST
};

static inline char *modeToString(enum Mode m)
{
    static const char *strings[] = { 
      "BeatDetection",
      "Stroboscope",
      "SoundIntensity",
      "Pulse",
      "On", 
      "Off",
      "END_OF_LIST"
      };

    return strings[(int)m];
}



// Special behavior for ++Mode
Mode& operator++( Mode &m ) {
  m = (Mode) (((int)m) + 1);
  if (m == Mode::END_OF_LIST) {
    m = Mode::BeatDetection;
  }
  return m;
}

// Special behavior for Mode++
Mode operator++( Mode &m, int ) {
  Mode result = m;
  ++m;
  return result;
}

// Special behavior for ++Mode
Mode& operator--( Mode &m ) {
  m = (Mode) (((int)m) - 1);
  if ((int)m <= -1) {
    m = (Mode) (((int) Mode::END_OF_LIST) - 1);
  }
  return m;
}

// Special behavior for Mode++
Mode operator--( Mode &m, int ) {
  Mode result = m;
  --m;
  return result;
}

#endif