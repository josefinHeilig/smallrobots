#pragma once
#include "Arduino.h"
#include "../esp32/hw_config.h"


#define V_OVERCHARGED 2.688     // -> 8.6 V input
#define V_FULLYCHARGED 2.625    // -> 8.4 V input
#define V_LOWWARNING 1.876      // -> 6.0 V input
#define V_LOWOFF 1.87           // 1.75; //-> 5.6 V input   

namespace SmallRobots {

    class BatteryChecker {



    private:
        
        float voltage = 0;
        bool batteryReadEnabled = false;

    

    public:
        BatteryChecker() {};

        float getVoltage()
        {
            return voltage;
        }

        void  enableReadVoltage()
        {
            // ENABLE BATTERY READING //
            // read power is normally disabled to safe power
            digitalWrite(BATTERY_READ_ENABLE_PIN, HIGH);
            batteryReadEnabled= true;
        }

        void  disableReadVoltage()
        {
            // DISABLE BATTERY READING //
            // disable read power to safe power
            digitalWrite(BATTERY_READ_ENABLE_PIN, HIGH);
            batteryReadEnabled= false;
        }

        bool readVoltage()
        {
            bool error = false;
            double reading = analogRead(BATTERY_READ_PIN); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
            if (reading > 0 || reading <= 4095)
            {
                //float v = reading / 4095.0f;
                //v *= 3.3;
                double y =  -0.000000000009824 * pow(reading, 3) + 0.000000016557283 * pow(reading, 2) + 0.000854596860691 * reading + 0.065440348345433;
                voltage = (float) y;
            } else error = true;
            return error;
        }

        bool isOverCharged(){
            return voltage>V_OVERCHARGED;
        }

        bool isLowWarning(){
            return voltage<V_LOWWARNING && voltage>V_LOWOFF;
        }

        bool isLowOff(){
            return voltage<V_LOWOFF;
        }

    };
};
extern float battery_voltage;
