#define ARM_MATH_CM4
#include <arm_math.h>
#include <Adafruit_NeoPixel.h>


int SAMPLE_RATE_HZ = 9000;   // Sample rate of the audio in hertz.
const int TONE_LOWS[] = {              // Lower bound (in hz) of each tone in the input sequence.
  500, 500, 500, 500, 500
};
const int TONE_HIGHS[] = {             // Upper bound (in hz) of each tone in the input sequence.
  1000, 1200, 1000, 1000, 1000
};

const int AUDIO_INPUT_PIN = 14;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).

float mainVolume = 0;
int TONE_WINDOW_MS = 4000;             // Maximum amount of milliseconds allowed to enter the full sequence.
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256

void setup() {
  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);

  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  mainVolume = analogRead(AUDIO_INPUT_PIN);    // read the input pin

  Serial.println(mainVolume);             // debug value
}



void samplingBegin() {
 
}

boolean samplingIsDone() {
 
}


