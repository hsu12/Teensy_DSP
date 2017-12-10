#define ARM_MATH_CM4
#include <arm_math.h>
#include <stdio.h>
#include <stdlib.h>

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

const int NEO_PIXEL_COUNT = 10;

float mainVolume = 0;
int TONE_WINDOW_MS = 4000;             // Maximum amount of milliseconds allowed to enter the full sequence.
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256
//char mag[50];




IntervalTimer samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;
//char commandBuffer[MAX_CHARS];
float frequencyWindow[10+1];

char *spectum;//spectum diplay on serial monitor


void setup() {
    // Set up serial port.
  Serial.begin(38400);

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);

  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);

  // Begin sampling audio
  samplingBegin();


   // Initialize spectrum display
  //spectrumSetup();

  // Begin sampling audio
  samplingBegin();
  spectum=(char*) malloc(50*sizeof(char));
}

void loop() {
  // put your main code here, to run repeatedly:
  mainVolume = analogRead(AUDIO_INPUT_PIN);    // read the input pin
               // debug value
    if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);
    for (int i = 1; i < FFT_SIZE; ++i) {
       if(i%10==0)
       {
        Serial.print(i);
        Serial.print(":");
        //Serial.println(magnitudes[i]);


        spectum=(char*) realloc(spectum, (int)(magnitudes[i]/50));
        memset(spectum,'*',(int)(magnitudes[i]/50));
        Serial.println(spectum);
       }
    }
   // Detect tone sequence.
    //toneLoop();//uncomment to enter pitch detection

    // Restart audio sampling.
    samplingBegin();
  }



}


// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < FFT_SIZE/2; ++i) {
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}






void toneLoop() {
  // Calculate the low and high frequency bins for the currently expected tone.

  // Get the average intensity of frequencies inside and outside the tone window.


}





void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  //Serial.println(analogRead(AUDIO_INPUT_PIN));
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
   // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}
