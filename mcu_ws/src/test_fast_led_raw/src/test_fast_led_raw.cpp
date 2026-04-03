#include <FastLED.h>

#define NUM_LEDS 5      // Number of LEDs in your strip
#define DATA_PIN D2      // Arduino pin connected to the LED data line
CRGB leds[NUM_LEDS];     // Define the array of LEDs

void setup() { 
  // Tell FastLED there is a WS2812 strip on pin 6
  FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS); 
}

void loop() { 
  leds[0] = CRGB::Red;   // Set first LED to red
  FastLED.show();        // Update the strip
  delay(500); 

  leds[0] = CRGB::Blue;  // Set first LED to blue
  FastLED.show(); 
  delay(500); 

  leds[1] = CRGB::Blue;  // Set first LED to blue
  FastLED.show(); 
  delay(500); 

  leds[2] = CRGB::Blue;  // Set first LED to blue
  FastLED.show(); 
  delay(500); 

  leds[3] = CRGB::Blue;  // Set first LED to blue
  FastLED.show(); 
  delay(500); 

  leds[4] = CRGB::Blue;  // Set first LED to blue
  FastLED.show(); 
  delay(500); 
}