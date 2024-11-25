#pragma once
#include "Arduino.h"


#define BATTERY_PIN                 -1 //34
#define BATTERY_READ_PIN            -1 //35 //Pin to read battery voltage from
#define BATTERY_READ_ENABLE_PIN     -1 //14

// RGB LED PIN //
#define RGB_LED_PIN                 48 //LED_BUILTIN

// I2S PINS - SOUND OUTPUT //we use MAX98357 I2S Amp

#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_I2S_DAC

#define MOZZI_AUDIO_BITS            16
#define MOZZI_I2S_PIN_BCK           37
#define MOZZI_I2S_PIN_WS            (MOZZI_I2S_PIN_BCK+1) // CANNOT BE CHANGED, HAS TO BE NEXT TO pBCLK, NOw 38
#define MOZZI_I2S_PIN_DATA          36



void setupPins ();

