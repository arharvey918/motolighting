/*
LED VU meter for Arduino and Adafruit NeoPixel LEDs.

Hardware requirements:
 - Most Arduino or Arduino-compatible boards (ATmega 328P or better).
 - Adafruit Electret Microphone Amplifier (ID: 1063)
 - Adafruit Flora RGB Smart Pixels (ID: 1260)
   OR
 - Adafruit NeoPixel Digital LED strip (ID: 1138)
 - Optional: battery for portable use (else power through USB or adapter)
Software requirements:
 - Adafruit NeoPixel library

Connections:
 - 3.3V to mic amp +
 - GND to mic amp -
 - Analog pin to microphone output (configurable below)
 - Digital pin to LED data input (configurable below)
 See notes in setup() regarding 5V vs. 3.3V boards - there may be an
 extra connection to make and one line of code to enable or disable.

Written by Adafruit Industries.  Distributed under the BSD license.
This paragraph must be included in any redistribution.
*/

#include <Adafruit_NeoPixel.h>

#define N_PIXELS  144  // Number of pixels in strand
#define MIC_PIN   A9  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     20  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS + 2) // Allow dot to go slightly off scale
#define PEAK_FALL 4  // Rate of peak falling dot
#define MEDIAN_READINGS 5
#define MAX_SENSORS 1

byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 1024;
Adafruit_NeoPixel
  strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

int level[MEDIAN_READINGS] = {0};

float alpha = 0.03;
float lastema;
short initialized[MAX_SENSORS] = { 0 };


void setup() {

  Serial.begin(9600);

  // This is only needed on 5V Arduinos (Uno, Leonardo, etc.).
  // Connect 3.3V to mic AND TO AREF ON ARDUINO and enable this
  // line.  Audio samples are 'cleaner' at 3.3V.
  // COMMENT OUT THIS LINE FOR 3.3V ARDUINOS (FLORA, ETC.):
//  analogReference(EXTERNAL);

  memset(vol, 0, sizeof(vol));
  strip.begin();
}

void loop() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
 

  n   = analogRead(MIC_PIN);                        // Raw reading from mic 
  Serial.println(n);

  if (n == 0) {
    n = 1024;
  }


  
  
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum

  Serial.print("actual reading: ");
  Serial.println(n);  
  
  lvl = moving_median(1, n, level);
  Serial.print("mm reading: ");
  Serial.println(lvl);
  
  
  lvl = ema(1, lvl);

  Serial.print("ema reading: ");
  Serial.println(lvl);
  
  //lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (long)(lvl) / (long)(450);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {
    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
    
  }



  // Draw peak dot  
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));



  
   strip.show(); // Update strip

// Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }



  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {  
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


/**
 * Calculates an exponential moving average given a sensor and the current value.
 */
int ema(int sensor, int current) {
  // Define static variables
  float ema;

      Serial.print("last ema value: ");
    Serial.println(lastema);

  // If initialized is set to 0, then we must be on the first observation.
  // Set the returned ema value to the current value and set the initialized flag to true.
  if (initialized[sensor] == 0) {
    ema = current;
    initialized[sensor] = 1;
  }
  
  // If the initialized flag has already been set, then there was a prior value.
  // We can now use the recursive formula to calculate the EMA. 
  else {
    ema = (alpha * current + (1 - alpha) * lastema);

  }

  // Set the previous value to the current EMA calculation to be used on the next round.
  lastema = ema;

  // Return the EMA as an integer (will truncate decimal)
  return (int)ema;
}


/**
  * Calculates the moving median of a set of numbers given the sensor, current value, and list of past values.
  */
int moving_median(int sensor, int current, int *values) {

  // Set up the static variables.  The 'count' array tells us how many past values have been stored.  'dupArray' is used to store the data that gets passed to the selection algorithm.  The selection algorithm has the potential to reorder the list, and we need the list to maintain the same order so we know which values are old and can be discarded from the moving filter.
  static short readingsAvailable = 0;
  static int dupArray[MEDIAN_READINGS] = { 0 };


  // If there are less readings than the number we need to take a median, return the current value and store it in the array.
  if (readingsAvailable < MEDIAN_READINGS) {
    values[readingsAvailable] = current;
    readingsAvailable++;
    return current;
  }

  // If the array containing the past readings is full, shift everything left (pushing the first observation out) and put the current reading in the end position of the array.  Take the median of this and return it.
  else {
      // Shift everything one position left
      for (int i = 0; i < (MEDIAN_READINGS - 1); i++) {
        values[i] = values[i + 1];
    }
    values[MEDIAN_READINGS - 1] = current;

    // Duplicate array to pass so 'values' doesn't get sorted
    for (int i = 0; i < MEDIAN_READINGS; i++) {
      dupArray[i] = values[i];
    }

    // Return the median of the values
    return selection_algorithm(0, MEDIAN_READINGS - 1, MEDIAN_READINGS/2, dupArray);
  }

  // We should never get to this point.
  return -1;
}

/**
  * Used to partition a list using the concept of quick-sort.
  * This method was modified from the following source:
  * http://discuss.codechef.com/questions/1489/find-median-in-an-unsorted-array-without-sorting-it 
  */
int partitions(int low, int high, int *values) {
  int p = low, r = high, x = values[r], i = p - 1;
  for (int j = p; j <= r - 1; j++) {
    if (values[j] <= x) {
      i = i + 1;
      int tmp = values[i];
      values[i] = values[j];
      values[j] = tmp;
    }
  }
  int tmp2 = values[i + 1];
  values[i + 1] = values[r];
  values[r] = tmp2;
  return i + 1;
}

/**
  * Finds the kth smallest number in an unsorted list using the selection algorithm.
  * This method was modified form the following source:
  * http://discuss.codechef.com/questions/1489/find-median-in-an-unsorted-array-without-sorting-it
  */
int selection_algorithm(int left, int right, int kth, int *values) {
  for (;;) {
    // Pivot between left and right
    int pivotIndex = partitions(left, right, values);
    int len = pivotIndex - left + 1;

    if (kth == len) {
      return values[pivotIndex];
    } else if (kth < len) {
      right = pivotIndex - 1;
    } else {
      kth = kth - len;
      left = pivotIndex + 1;
    }
  }
}
