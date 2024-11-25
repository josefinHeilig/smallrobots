#include "hw_config.h"
 
void setupPins (){


    // RBG LED PIN //
    pinMode(RGB_LED_PIN, OUTPUT);


    
    // BATTERY READ VOLTAGE PIN //
    // pinMode(BATTERY_PIN, INPUT);
    // adcAttachPin(BATTERY_PIN);

    // //LOW POWER BATTERY READ VOLTAGE PIN //
    // pinMode(BATTERY_READ_PIN, INPUT);
    // adcAttachPin(BATTERY_READ_PIN);
    // pinMode(BATTERY_READ_ENABLE_PIN, OUTPUT);

    }