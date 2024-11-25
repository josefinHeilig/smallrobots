

#pragma once

#include "../esp32/hw_config.h"
//#include <FastLED.h>
//#include "../esp32/wifi/WifiStateMachine.h"

#define NB_OF_RGB_PIXELS 1 



namespace SmallRobots {
    
            
    class RGBLedService {

        private:

            //CRGB lightstrip[NB_OF_RGB_PIXELS];   // Define the array of leds

            uint8_t redValue = 0, greenValue = 0, blueValue = 0;

            //CRGB finalCol = CRGB(0,0,0);

            bool pixelOn_0, pixelOn_1, pixelOn_2 = false;

            void togglePixel(bool *toggle);

        public:

            RGBLedService(){};
           // ~RGBLedService(){};

            void setup();
        
            void setFinalCol ( uint8_t r, uint8_t g,uint8_t b);
            //void setFinalCol (CRGB col);

            void allShow();
            void allBlink();
            void allOff();
            //void pixelBlink(int indexPixel);
 
            
            //void showBatteryStatus();
            //void showWifiStatus();
            //void showValuesFromWifi();

            //void pixelShow(int indexPixel);
            //void pixelShow(int indexPixel, CRGB col);
            //void pixelShowBrightness(int indexPixel, float val=1.0);
            //void pixelOff(int indexPixel);

           // CRGB convertVoltageToColor( float value, float min, float max);
            

        
    };

    extern RGBLedService rgbService;

};
