#include <Arduino.h>
#include <arduinoFFT.h>      //https://github.com/kosme/arduinoFFT
#include <FastLED.h>

//FFT
arduinoFFT FFT = arduinoFFT();

const uint16_t SAMPLES = 512;              //Must be a power of 2
const uint16_t SAMPLING_FREQUENCY = 20000; //Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT.
const float BIN_FREQUENCY  = (SAMPLING_FREQUENCY/SAMPLES);

unsigned int sampling_period_us =  round(1000000 * (1.0 / SAMPLING_FREQUENCY));
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime, oldTime;

uint16_t lowVoiceFreqBin;
uint16_t highVoiceFreqBin;
double lowVoiceFreq = 100; //In Hz
double highVoiceFreq = 1000; //In Hz

//Define Analog Pin
#define ADC_PIN 34
double highestMagnitude = 0;

//FastLED
#define LED_PIN 23
#define NUM_LEDS 10
CRGB leds[NUM_LEDS];

void setLedStrip (CRGB (&strip)[NUM_LEDS], CRGB color) {
  for (int led = 0; led < NUM_LEDS; led++) {
    strip[led] = color;
  }
}

struct FrequencyBin {
  uint16_t bin;
  double magnitude;

  float frequency;

  FrequencyBin(uint16_t index, double _magnitude) {
    bin = index;
    magnitude = _magnitude;
    frequency = index * BIN_FREQUENCY;
  }
};

FrequencyBin maxFFTfromTo(double fft[], uint16_t startBin, uint16_t endBin) {
  uint16_t maxMagBin = 1;

  for (uint16_t bin = startBin; bin < endBin; bin++) {
    if (fft[bin] > maxMagBin) maxMagBin = bin;
    Serial.println();
  }

  FrequencyBin bin = FrequencyBin(maxMagBin, fft[maxMagBin]);

  return bin;
}

FrequencyBin maxFFT(double fft[], uint16_t length) {
  return maxFFTfromTo(fft, 0, length);
}

void setup() {
  Serial.begin(115200);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);

  for (int led = 0; led < NUM_LEDS; led++) {
    leds[led] = CRGB::Black;
  }

  for (uint16_t bin = 0; bin < SAMPLES/2; bin++) {
    if (bin * BIN_FREQUENCY > lowVoiceFreq) lowVoiceFreqBin = bin;
  }
  for (uint16_t bin = 0; bin < SAMPLES/2; bin++) {
    if (bin * BIN_FREQUENCY > highVoiceFreq) highVoiceFreqBin = bin;
  }

}

void loop() {

  //Take samples from microphone pin for FFT calculation
  for (uint16_t i = 0; i < SAMPLES; i++) {
    newTime = micros() - oldTime;
    oldTime = newTime;
    vReal[i] = analogRead(ADC_PIN);
    vImag[i] = 0;
    while (micros() < (newTime + sampling_period_us))
    {
      delay(0);
    }
  }
  //Calculate FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  FrequencyBin maxBin = maxFFTfromTo(vReal, lowVoiceFreqBin, highVoiceFreqBin);
  
  if (maxBin.magnitude > highestMagnitude) {
    highestMagnitude = maxBin.magnitude;
  }

  if (maxBin.frequency > 440) {
    setLedStrip(leds, CRGB::Blue);
  }
  else {
    setLedStrip(leds, CRGB::Red);
  }

  Serial.print(maxBin.magnitude);
  Serial.print("  ");
  Serial.print(maxBin.bin);
  Serial.print("  ");
  Serial.println(highestMagnitude);
  FastLED.setBrightness(map(maxBin.magnitude, 0, highestMagnitude, 0, 125));
  FastLED.show();
}