// FHT, see http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // amount of bins to use
#include <FHT.h> // include the library

#include "TapTempoButton.h"

#define FreqLog // use log scale for FHT frequencies
#define FreqGainFactorBits 0
#define FreqSerialBinary
#define VolumeGainFactorBits 0
#define DisableLOG

// Macros for faster sampling, see
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Set to true if you want to use the FHT 128 channel analyser to visualize
// the detected frequencies. Will disable beat detection.
const bool LOG_FREQUENCY_DATA = false;
const bool LOG_SAMPLING = false;

// Set to true if the light should be based on detected beats instead
// of detected amplitudes.
const bool PERFORM_BEAT_DETECTION = true;

const int SOUND_REFERENCE_PIN = 8; // D8
const int HAT_LIGHTS_PIN = 10; // D9
const int HAT_LIGHTS_LOW_PIN = 13; // D11
const int HAT_LIGHTS_HIGH_PIN = 12; // D12
const int HAT_LIGHTS_PULSE_PIN = 9; // D13

const int LIGHT_PULSE_DELAY = 10000;
const int LIGHT_PULSE_DURATION = 2000;

const int LIGHT_FADE_OUT_DURATION = 360; // good value range is [100:1000]
const float MINIMUM_LIGHT_INTENSITY = 0.018; // in range [0:1]
const float MAXIMUM_LIGHT_INTENSITY = 1.0; // in range [0:1]
const float LIGHT_INTENSITY_RANGE = MAXIMUM_LIGHT_INTENSITY - MINIMUM_LIGHT_INTENSITY;

const float MINIMUM_LIGHT_INTENSITY_PULSE = 0.70;
const float MAXIMUM_LIGHT_INTENSITY_PULSE = 0.8; // in range [0:1]
const float LIGHT_INTENSITY_RANGE_PULSE = MAXIMUM_LIGHT_INTENSITY_PULSE - MINIMUM_LIGHT_INTENSITY_PULSE;

const int MAXIMUM_SIGNAL_VALUE = 1024;

const int OVERALL_FREQUENCY_RANGE_START = 1; // should be 0, but first 2 bands produce too much noise
const int OVERALL_FREQUENCY_RANGE_END = FHT_N / 2;
const int OVERALL_FREQUENCY_RANGE = OVERALL_FREQUENCY_RANGE_END - OVERALL_FREQUENCY_RANGE_START;

const int FIRST_FREQUENCY_RANGE_START = 1;
const int FIRST_FREQUENCY_RANGE_END = 3;
const int FIRST_FREQUENCY_RANGE = FIRST_FREQUENCY_RANGE_END - FIRST_FREQUENCY_RANGE_START;

const int SECOND_FREQUENCY_RANGE_START = 2;
const int SECOND_FREQUENCY_RANGE_END = 6;
const int SECOND_FREQUENCY_RANGE = SECOND_FREQUENCY_RANGE_END - SECOND_FREQUENCY_RANGE_START;

const int SINGLE_BEAT_DURATION = 100; // good value range is [50:150]

const int FREQUENCY_MAGNITUDE_SAMPLES = 100; // good value range is [5:15]

int frequencyMagnitudeSampleIndex = 0;

int currentOverallFrequencyMagnitude = 0;
int totalOverallFrequencyMagnitude = 0;
int averageOverallFrequencyMagnitude = 0;
int overallFrequencyMagnitudeVariance = 0;
byte overallFrequencyMagnitudes[FREQUENCY_MAGNITUDE_SAMPLES];

int currentFirstFrequencyMagnitude = 0;
int totalFirstFrequencyMagnitude = 0;
int averageFirstFrequencyMagnitude = 0;
int firstFrequencyMagnitudeVariance = 0;
byte firstFrequencyMagnitudes[FREQUENCY_MAGNITUDE_SAMPLES];

int currentSecondFrequencyMagnitude = 0;
int totalSecondFrequencyMagnitude = 0;
int averageSecondFrequencyMagnitude = 0;
int secondFrequencyMagnitudeVariance = 0;
byte secondFrequencyMagnitudes[FREQUENCY_MAGNITUDE_SAMPLES];

int currentSignal = 0;
int totalSignal = 0;
int averageSignal = 0;
int signalVariance = 0;
byte signals[FREQUENCY_MAGNITUDE_SAMPLES];

unsigned long lastBeatTimestamp = 0;
unsigned long durationSinceLastBeat = 0;
float beatProbability = 0;
float beatProbabilityThreshold = 0.5;

unsigned long lightIntensityBumpTimestamp = 0;
float lightIntensityBumpValue = 0;
float lightIntensityValue = 0;

unsigned long lastPulseTimestamp = 0;

int potiValue = 0;
const int UPDATE_POTI_EVERY_NTH_FRAME = 64;
int potiFrameCounter = UPDATE_POTI_EVERY_NTH_FRAME;
float intensityMultiplier = 1.0;

unsigned long lastFlashTimestamp = 0;

float max_bpm = 140;
int min_delay_between_beats = (int) (60000.0 / max_bpm);

const int FLASH_BUTTON = 4;
int flash_cooldown_millis = 1000;

TapTempoButton tapTempo;
Button flashButton;

void setup() {
  setupADC();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(HAT_LIGHTS_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_LOW_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_HIGH_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_PULSE_PIN, OUTPUT);
  pinMode(SOUND_REFERENCE_PIN, OUTPUT);
  pinMode(FLASH_BUTTON, INPUT);

  digitalWrite(HAT_LIGHTS_PIN, HIGH);
  digitalWrite(SOUND_REFERENCE_PIN, HIGH);
  
  analogWrite(HAT_LIGHTS_LOW_PIN, 255 * MINIMUM_LIGHT_INTENSITY);
  analogWrite(HAT_LIGHTS_HIGH_PIN, 255 * MAXIMUM_LIGHT_INTENSITY);

  for (int i = 0; i < FREQUENCY_MAGNITUDE_SAMPLES; i++) {
    overallFrequencyMagnitudes[i] = 0;
    firstFrequencyMagnitudes[i] = 0;
    secondFrequencyMagnitudes[i] = 0;
    signals[i] = 0;
  }
  
  flashButton.begin(FLASH_BUTTON);
  tapTempo.begin(FLASH_BUTTON);

  Serial.begin(115200);
  //Serial.println("Starting Festival Hat Controller");
}

/**
 * Analog to Digital Conversion needs to be configured to free running mode
 * in order to read the sound sensor values at a high frequency.
 * See: http://maxembedded.com/2011/06/the-adc-of-the-avr/
 */
void setupADC() {
  ADCSRA = 0xe0+7; // "ADC Enable", "ADC Start Conversion", "ADC Auto Trigger Enable" and divider.
  ADMUX = 0x0; // use adc0. Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
  ADMUX |= 0x40; // Use Vcc for analog reference.
  DIDR0 = 0x01; // turn off the digital input for adc0
}

unsigned long start = 0;
unsigned long end = 0;

void printExecutionTime()
{
  end = millis();
  unsigned long time = end - start;
  Serial.print(String(time));
}

void loop() {
  while(1)
  {
    if (LOG_FREQUENCY_DATA) 
    {
      readAudioSamples();
      getFrequencyData();
      logFrequencyData();
    } 
    else 
    {
      //Serial.print(String(millis()));
      readAudioSamples();

      if (PERFORM_BEAT_DETECTION) 
      {
        getFrequencyData();
        processFrequencyData();
        updateBeatProbability();

        if (lastFlashTimestamp + flash_cooldown_millis < millis())
        {
          updateLightIntensityBasedOnBeats();
        }
      } 
      else 
      {
        if (lastFlashTimestamp + flash_cooldown_millis < millis())
        {
          updateLightIntensityBasedOnAmplitudes();
        }
      }

      if (potiFrameCounter++ >= UPDATE_POTI_EVERY_NTH_FRAME)
      {
        readPoti();
        potiFrameCounter = 0;
      }

      updateLights();

      unsigned long end = millis();

    }
  }

}

/**
 * Will read the sound sensor values from pin A0.
 */
void readAudioSamples() {
  long currentAverage = 0;
  long currentMaximum = 0;
  long currentMinimum = MAXIMUM_SIGNAL_VALUE;
  
  float sampleSum = 0;

  long timeStart = millis();
  for (int i = 0; i < FHT_N; i++) 
  { // save 256 samples
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int) j << 8) | m; // form into an int

    float sample = (k - 256) / 256.0;
    sampleSum += sample;
  
    currentMinimum = min(currentMinimum, k);
    currentMaximum = max(currentMaximum, k);
    currentAverage += k;
    
    /*
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    k <<= FreqGainFactorBits;
    */

    fht_input[i] = k - 256; // put real data into bins
  }
  long timeEnd = millis();
  long duration = timeEnd - timeStart;
  Serial.println(duration);

  float sampleAverage = sampleSum / FHT_N;
  if (LOG_SAMPLING)
  {
    Serial.print("Min:");
    Serial.print(-1);
    Serial.print(",");
    Serial.print("Max:");
    Serial.print(1);
    Serial.print(",");
    Serial.print("Signal:");
    Serial.println(sampleAverage);
  }

  currentAverage /= FHT_N;
  
  int signalDelta = currentMaximum - currentAverage;
  currentSignal = currentAverage + (2 * signalDelta);
    
  processHistoryValues(
    signals, 
    frequencyMagnitudeSampleIndex, 
    currentSignal, 
    totalSignal, 
    averageSignal, 
    signalVariance
  );
  
  //logValue("A", (float) currentAverage / MAXIMUM_SIGNAL_VALUE, 10);
  //logValue("M", (float) currentMaximum / MAXIMUM_SIGNAL_VALUE, 10);
  //logValue("S", (float) currentSignal / MAXIMUM_SIGNAL_VALUE, 20);
}

void readPoti()
{
  cli();  // UDRE interrupt slows this way down on arduino1.0
  while(!(ADCSRA & 0x10)); // wait for adc to be ready
  ADCSRA = 0xf5; // restart adc
  ADMUX = 0x1; // use adc1. Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
  ADMUX |= 0x40; // Use Vcc for analog reference.
  delayMicroseconds(100);
  byte m = ADCL; // fetch adc data
  byte j = ADCH;
  int output = (j << 8) | m; // form into an int
  if (abs(potiValue - output) > 1 || true){
    potiValue = output;
    intensityMultiplier = potiValue/1023.0;
    //intensityMultiplier = pow(intensityMultiplier, 0.1);
    //intensityMultiplier = pow(sin(intensityMultiplier * HALF_PI), 4);
    intensityMultiplier = pow(intensityMultiplier, 1.5);
    //logValue("M", intensityMultiplier, 100);
  }
  
  sei();
  ADMUX = 0x0; // use adc1. Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
  ADMUX |= 0x40; // Use Vcc for analog reference.
}

/**
 * Will run the Fast Hartley Transform to convert the time domain signals
 * to the frequency domain.
 *
 * See: http://wiki.openmusiclabs.com/wiki/ArduinoFHT
 */
void getFrequencyData() 
{
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the FHT
  fht_run(); // process the data in the FHT
  fht_mag_log(); // get the magnitude of each bin in the FHT
}

void logFrequencyData() 
{
#ifdef FreqSerialBinary
  // print as binary
  Serial.write(255); // send a start byte
  Serial.write(fht_log_out, FHT_N / 2); // send out the data
#else
  // print as text
  for (int i = 0; i < FHT_N / 2; i++) {
      int logValue = (9 * fht_log_out[i]) / 255;
      Serial.print(logValue);
      Serial.print(',');
  }
  //Serial.print('\n');
#endif
}

/**
 * Will extract insightful features from the frequency data in order
 * to perform the beat detection.
 */
void processFrequencyData() 
{
  processOverallFrequencyMagnitude();
  processFirstFrequencyMagnitude();
  processSecondFrequencyMagnitude();
  
  // prepare the magnitude sample index for the next update
  frequencyMagnitudeSampleIndex += 1;
  if (frequencyMagnitudeSampleIndex >= FREQUENCY_MAGNITUDE_SAMPLES) {
    frequencyMagnitudeSampleIndex = 0; // wrap the index
  }
}

void processOverallFrequencyMagnitude() 
{
  currentOverallFrequencyMagnitude = getFrequencyMagnitude(
    fht_log_out, 
    OVERALL_FREQUENCY_RANGE_START, 
    OVERALL_FREQUENCY_RANGE_END
  );
  
  processHistoryValues(
    overallFrequencyMagnitudes, 
    frequencyMagnitudeSampleIndex, 
    currentOverallFrequencyMagnitude, 
    totalOverallFrequencyMagnitude, 
    averageOverallFrequencyMagnitude, 
    overallFrequencyMagnitudeVariance
  );
}

void processFirstFrequencyMagnitude() 
{
  currentFirstFrequencyMagnitude = getFrequencyMagnitude(
    fht_log_out, 
    FIRST_FREQUENCY_RANGE_START, 
    FIRST_FREQUENCY_RANGE_END
  );
  
  processHistoryValues(
    firstFrequencyMagnitudes, 
    frequencyMagnitudeSampleIndex, 
    currentFirstFrequencyMagnitude, 
    totalFirstFrequencyMagnitude, 
    averageFirstFrequencyMagnitude, 
    firstFrequencyMagnitudeVariance
  );
}

void processSecondFrequencyMagnitude() 
{
  currentSecondFrequencyMagnitude = getFrequencyMagnitude(
    fht_log_out, 
    SECOND_FREQUENCY_RANGE_START, 
    SECOND_FREQUENCY_RANGE_END
  );
  
  processHistoryValues(
    secondFrequencyMagnitudes, 
    frequencyMagnitudeSampleIndex, 
    currentSecondFrequencyMagnitude, 
    totalSecondFrequencyMagnitude, 
    averageSecondFrequencyMagnitude, 
    secondFrequencyMagnitudeVariance
  );
}

byte getFrequencyMagnitude(byte frequencies[], const int startIndex, const int endIndex)
{
  int total = 0;
  int average = 0;
  int maximum = 0;
  int minimum = MAXIMUM_SIGNAL_VALUE;
  int current;
  
  for (int i = startIndex; i < endIndex; i++) {
    current = frequencies[i];
    total += current;
    maximum = max(maximum, current);
    minimum = min(minimum, current);
  }
  
  average = total / (endIndex - startIndex);
  
  int value = average;
  //int value = maximum - average;
  //logValue("F", (float) value / 128, 10);
  
  return value;
}

void processHistoryValues(byte history[], int &historyIndex, int &current, int &total, int &average, int &variance)
{
  total -= history[historyIndex]; // subtract the oldest history value from the total
  total += (byte) current; // add the current value to the total
  history[historyIndex] = current; // add the current value to the history
  
  average = total / FREQUENCY_MAGNITUDE_SAMPLES;
  
  // update the variance of frequency magnitudes
  long squaredDifferenceSum = 0;
  for (int i = 0; i < FREQUENCY_MAGNITUDE_SAMPLES; i++) {
    squaredDifferenceSum += pow(history[i] - average, 2);
  }
  variance = (double) squaredDifferenceSum / FREQUENCY_MAGNITUDE_SAMPLES;
}

/**
 * Will update the beat probability, a value between 0 and 1
 * indicating how likely it is that there's a beat right now.
 */
void updateBeatProbability()
{
  beatProbability = 1;
  beatProbability *= calculateSignalChangeFactor();
  beatProbability *= calculateMagnitudeChangeFactor();
  beatProbability *= calculateVarianceFactor();
  beatProbability *= calculateRecencyFactor();
  
  if (beatProbability >= beatProbabilityThreshold) {
    lastBeatTimestamp = millis();
    durationSinceLastBeat = 0;
  }
  
  logValue("B", beatProbability, 5);
}

/**
 * Will calculate a value in range [0:2] based on the magnitude changes of
 * different frequency bands.
 * Low values are indicating a low beat probability.
 */
float calculateSignalChangeFactor()
{
  float aboveAverageSignalFactor;
  if (averageSignal < 75 || currentSignal < 150) {
    aboveAverageSignalFactor = 0;
  } else {
    aboveAverageSignalFactor = ((float) currentSignal / averageSignal);
    aboveAverageSignalFactor = constrain(aboveAverageSignalFactor, 0, 2);
  }
  
  //logValue("SC", (float) currentSignal / 512, 10);
  //logValue("SA", (float) averageSignal / 512, 10);
  logValue("SF", aboveAverageSignalFactor / 2, 2);
  return aboveAverageSignalFactor;
}

/**
 * Will calculate a value in range [0:1] based on the magnitude changes of
 * different frequency bands.
 * Low values are indicating a low beat probability.
 */
float calculateMagnitudeChangeFactor()
{
  float changeThresholdFactor = 1.1;
  if (durationSinceLastBeat < 750) {
    // attempt to not miss consecutive beats
    changeThresholdFactor *= 0.95;
  } else if (durationSinceLastBeat > 1000) {
    // reduce false-positives
    changeThresholdFactor *= 1.05;
  }
  
  // current overall magnitude is higher than the average, probably 
  // because the signal is mainly noise
  float aboveAverageOverallMagnitudeFactor = ((float) currentOverallFrequencyMagnitude / averageOverallFrequencyMagnitude);
  aboveAverageOverallMagnitudeFactor -= 1.05;
  aboveAverageOverallMagnitudeFactor *= 10;
  aboveAverageOverallMagnitudeFactor = constrain(aboveAverageOverallMagnitudeFactor, 0, 1);
  
  // current magnitude is higher than the average, probably 
  // because the there's a beat right now
  float aboveAverageFirstMagnitudeFactor = ((float) currentFirstFrequencyMagnitude / averageFirstFrequencyMagnitude);
  aboveAverageOverallMagnitudeFactor -= 0.1;
  aboveAverageFirstMagnitudeFactor *= 1.5;
  aboveAverageFirstMagnitudeFactor = pow(aboveAverageFirstMagnitudeFactor, 3);
  aboveAverageFirstMagnitudeFactor /= 3;
  aboveAverageFirstMagnitudeFactor -= 1.25;
  
  aboveAverageFirstMagnitudeFactor = constrain(aboveAverageFirstMagnitudeFactor, 0, 1);
  
  float aboveAverageSecondMagnitudeFactor = ((float) currentSecondFrequencyMagnitude / averageSecondFrequencyMagnitude);
  aboveAverageSecondMagnitudeFactor -= 1.01;
  aboveAverageSecondMagnitudeFactor *= 10;
  aboveAverageSecondMagnitudeFactor = constrain(aboveAverageSecondMagnitudeFactor, 0, 1);
  
  float magnitudeChangeFactor = aboveAverageFirstMagnitudeFactor;
  if (magnitudeChangeFactor > 0.15) {
    magnitudeChangeFactor = max(aboveAverageFirstMagnitudeFactor, aboveAverageSecondMagnitudeFactor);
  }
  
  if (magnitudeChangeFactor < 0.5 && aboveAverageOverallMagnitudeFactor > 0.5) {
    // there's no bass related beat, but the overall magnitude changed significantly
    //magnitudeChangeFactor = max(magnitudeChangeFactor, aboveAverageOverallMagnitudeFactor);
  } else {
    // this is here to avoid treating signal noise as beats
    //magnitudeChangeFactor *= 1 - aboveAverageOverallMagnitudeFactor;
  }
  
  //float maximumMagnitude = 128; //128;
  
  //logValue("CO", (currentOverallFrequencyMagnitude - averageOverallFrequencyMagnitude) / maximumMagnitude, 5);
  //logValue("C1", (currentFirstFrequencyMagnitude - averageFirstFrequencyMagnitude) / maximumMagnitude, 5);
  //logValue("C2", (currentSecondFrequencyMagnitude - averageSecondFrequencyMagnitude) / maximumMagnitude, 5);

  //logValue("CO", (currentOverallFrequencyMagnitude) / maximumMagnitude, 10);
  //logValue("C1", (currentFirstFrequencyMagnitude) / maximumMagnitude, 10);
  //logValue("C2", (currentSecondFrequencyMagnitude) / maximumMagnitude, 10);

  logValue("AO", aboveAverageOverallMagnitudeFactor, 2);
  logValue("A1", aboveAverageFirstMagnitudeFactor, 10);
  logValue("A2", aboveAverageSecondMagnitudeFactor, 10);
  //logValue("A1|2", max(aboveAverageFirstMagnitudeFactor, aboveAverageSecondMagnitudeFactor), 1);
  
  logValue("M", magnitudeChangeFactor, 1);
  
  return magnitudeChangeFactor;
}

/**
 * Will calculate a value in range [0:1] based on variance in the first and second
 * frequency band over time. The variance will be high if the magnitude of bass
 * frequencies changed in the last few milliseconds.
 * Low values are indicating a low beat probability.
 */
float calculateVarianceFactor()
{
  // a beat also requires a high variance in recent frequency magnitudes
  float firstVarianceFactor = ((float) (firstFrequencyMagnitudeVariance - 50) / 20) - 1;
  firstVarianceFactor = constrain(firstVarianceFactor, 0, 1);
  
  float secondVarianceFactor = ((float) (secondFrequencyMagnitudeVariance - 50) / 20) - 1;
  secondVarianceFactor = constrain(secondVarianceFactor, 0, 1);
  
  float varianceFactor = max(firstVarianceFactor, secondVarianceFactor);
  
  //logValue("V", varianceFactor, 1);
  
  return varianceFactor;
}

/**
 * Will calculate a value in range [0:1] based on the recency of the last detected beat.
 * Low values are indicating a low beat probability.
 */
float calculateRecencyFactor()
{
  float recencyFactor = 1;
  durationSinceLastBeat = millis() - lastBeatTimestamp;
  
  int referenceDuration = min_delay_between_beats - SINGLE_BEAT_DURATION;
  recencyFactor = 1 - ((float) referenceDuration / durationSinceLastBeat);
  recencyFactor = constrain(recencyFactor, 0, 1);
  
  //logValue("R", recencyFactor, 5);
  
  return recencyFactor;
}

/**
 * Will update the light intensity bump based on the recency of detected beats.
 */
void updateLightIntensityBasedOnBeats()
{
  float intensity = 1 - ((float) durationSinceLastBeat / LIGHT_FADE_OUT_DURATION);
  intensity = constrain(intensity, 0, 1);
  
  if (intensity > lightIntensityValue) {
    lightIntensityBumpValue = intensity;
    lightIntensityBumpTimestamp = millis();
  }
}

/**
 * Will update the light intensity bump based on measured amplitudes.
 */
void updateLightIntensityBasedOnAmplitudes()
{
  float intensity;
  if (averageSignal < 1 || currentSignal < 1)
  {
    intensity = 0;
  } else {
    intensity = (float) (currentSignal - averageSignal) / MAXIMUM_SIGNAL_VALUE;
    intensity *= pow(intensity, 3);
    
    if (intensity < 0.1) {
      intensity = 0;
    } else {
      intensity -= 0.1;
      intensity = pow(1 + intensity, 3) - 1;
      intensity = constrain(intensity, 0, 1);
    }
  }
  
  //logValue("I", intensity, 10);
  
  if (intensity > lightIntensityValue) {
    lightIntensityBumpValue = intensity;
    lightIntensityBumpTimestamp = millis();
  }
}

void setMaxBPM(float bpm)
{
    max_bpm = (int) (bpm * 1.05);
    min_delay_between_beats = 60000L / max_bpm;
    flash_cooldown_millis = (60000L / max_bpm) * 2;
}

/**
 * Will update the hat lights based on the last light intensity bumps.
 */
void updateLights()
{
  tapTempo.update();

  if (tapTempo.isPressedThisFrame()){
    lightIntensityBumpValue = 1.0f;
    lightIntensityBumpTimestamp = millis();
    lastFlashTimestamp = millis();
    durationSinceLastBeat = 0;
  }

  if (tapTempo.hasTempoChanged()){
    int bpm = (int) tapTempo.getTempo();
    setMaxBPM(bpm);
  }

  unsigned long durationSinceLastBump = millis() - lightIntensityBumpTimestamp;
  float fadeFactor = 1 - ((float) durationSinceLastBump / LIGHT_FADE_OUT_DURATION);
  fadeFactor = constrain(fadeFactor, 0, 1);
  
  lightIntensityValue = lightIntensityBumpValue * fadeFactor * intensityMultiplier;
  lightIntensityValue = constrain(lightIntensityValue, 0, 1);
  
  //logValue("L", lightIntensityValue, 20);
  
  // scale the intensity to be in range of maximum and minimum
  float scaledLightIntensity = MINIMUM_LIGHT_INTENSITY + (lightIntensityValue * LIGHT_INTENSITY_RANGE);
  
  int pinValue = 255 * scaledLightIntensity;
  analogWrite(HAT_LIGHTS_PIN, pinValue);
  
  // also use the builtin LED, for debugging when no lights are connected
  if (scaledLightIntensity > MAXIMUM_LIGHT_INTENSITY - (LIGHT_INTENSITY_RANGE / 4)) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // update the pulse signal
  unsigned long durationSincePulse = millis() - lastPulseTimestamp;
  fadeFactor = ((float) durationSincePulse / (LIGHT_PULSE_DURATION * 2));
  if (durationSincePulse >= LIGHT_PULSE_DURATION) {
    fadeFactor = 1 - fadeFactor;
  }
  fadeFactor *= 2;
  fadeFactor = constrain(fadeFactor, 0, 1);
  
  // scale the intensity to be in range of maximum and minimum
  float lightIntensityPulse = MINIMUM_LIGHT_INTENSITY_PULSE + (fadeFactor * LIGHT_INTENSITY_RANGE_PULSE);
  
  //logValue("P", lightIntensityPulse, 10);
  
  pinValue = 255 * lightIntensityPulse * intensityMultiplier;
  analogWrite(HAT_LIGHTS_PULSE_PIN, pinValue);
  
  if (durationSincePulse >= LIGHT_PULSE_DELAY) {
    lastPulseTimestamp = millis();
  }
}

/**
 * Converts the specified value into an ASCII-art progressbar
 * with the specified length.
 */
String toProgressBar(float value, const int length)
{
  int amount = max(0, min(length, value * length));
  String progressBar = "[";
  for (int i = 0; i < amount; i++) {
    progressBar += "=";
  }
  for (int i = 0; i < length - amount; i++) {
    progressBar += " ";
  }
  progressBar += "]\n";
  return progressBar;
}

void logValue(String name, boolean value)
{
  logValue(name, value ? 1.0 : 0.0, 1);
}

void logValue(String name, float value)
{
  logValue(name, value, 10);
}

void logValue(String name, float value, int length)
{
#ifndef DisableLOG
  Serial.print(" | " + name + ": " + toProgressBar(value, length));
#endif
}