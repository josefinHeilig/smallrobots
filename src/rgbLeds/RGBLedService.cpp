#include "RGBLedService.h"
#include "../behaviour/Behaviours.h"
#include "../power/BatteryCheckService.h"






namespace SmallRobots {
     
    RGBLedService rgbService;
    

    void RGBLedService::setup(){
        //FastLED.addLeds<WS2812, RGB_LED_PIN, RGB>(lightstrip, NB_OF_RGB_PIXELS);  // GRB ordering is typical

        

        allShow();
    }

  

    void RGBLedService::setFinalCol (uint8_t r, uint8_t g,uint8_t b)
    {
        redValue = r;
        greenValue = g;
        blueValue = b;
        //finalCol = col;
        //finalCol = CRGB(col.g, col.r,col.b);

        
    }

    void RGBLedService::allShow()
    {
        analogSetAttenuation(ADC_0db);
         neopixelWrite(BUILTIN_LED, redValue, greenValue, blueValue); 
        // for (uint16_t indexPixel = 0; indexPixel < NB_OF_RGB_PIXELS; indexPixel++)
        // {
        // lightstrip[indexPixel] = finalCol; //CRGB ( finalCol.Green,finalCol.Blue, finalCol.Red);
        
        // }
        // FastLED.show();
    }

    void RGBLedService::allOff()
    {
        analogSetAttenuation(ADC_0db);
         neopixelWrite(BUILTIN_LED, 0, 0, 0); 
        // for (uint16_t indexPixel = 0; indexPixel < NB_OF_RGB_PIXELS; indexPixel++)
        // {
        // lightstrip[indexPixel] = CRGB::Black;
        
        // }
        // FastLED.show();
    }

    // void RGBLedService::showBatteryStatus(){
    //     // CRGB temp = convertVoltageToColor(battery_voltage, V_LOWOFF, V_FULLYCHARGED);
    //     // setFinalCol(temp);
       
    // }


    // void RGBLedService::pixelBlink(int indexPixel)
    // {
    //     // if(indexPixel == 0) togglePixel(&pixelOn_0, indexPixel);
    //     // else  if(indexPixel == 1) togglePixel(&pixelOn_1, indexPixel);
    //     // else  if(indexPixel == 2) togglePixel(&pixelOn_2, indexPixel);    
    // }

   void RGBLedService::allBlink()
    {

        togglePixel(&pixelOn_0);
        // else  if(indexPixel == 1) togglePixel(&pixelOn_1, indexPixel);
        // else  if(indexPixel == 2) togglePixel(&pixelOn_2, indexPixel);
        // for (uint16_t indexPixel = 0; indexPixel < NB_OF_RGB_PIXELS; indexPixel++)
        // {
        // pixelBlink( indexPixel);
        
        // }
 
    }

    void RGBLedService::togglePixel(bool *toggle)
    {
        analogSetAttenuation(ADC_0db);
        if (*toggle) neopixelWrite(BUILTIN_LED, redValue, greenValue, blueValue); 
        else neopixelWrite(BUILTIN_LED, 0, 0, 0); 
        *toggle =! *toggle;
       //                                                                                                                                                                                                                                                                                                                                                Serial.println(*toggle);
    }

    // void RGBLedService::pixelShow(int indexPixel, CRGB col)
    // {
    //     // lightstrip[indexPixel]= col;
    //     // FastLED.show();
    // }

    // void RGBLedService::pixelShow(int indexPixel)
    // {
    //     // lightstrip[indexPixel]= finalCol;
    //     // FastLED.show();

    // }

    // void RGBLedService::pixelShowBrightness(int indexPixel, float val)
    // {
    //     // CRGB col = CRGB(finalCol.Red*val, finalCol.Green*val,finalCol.Blue*val);
    //     // lightstrip[indexPixel]= col;
    //     // FastLED.show();
    // }

    // void RGBLedService::pixelOff(int indexPixel)
    // {
    //     // lightstrip[indexPixel]= CRGB::Black;
    //     // FastLED.show();
    // }

    // CRGB RGBLedService::convertVoltageToColor( float value, float min, float max)
    // {

    //         float val = (value - min) / (max - min) ; 

    //         int r = int (2.0f * (1 - val) * 255);
    //         int g = int( 2.0f * val * 255);
    //         int b = 0;
    //         int w = 0;

    //         if (r > 255) r = 255;
    //         else if (r < 0) r = 0;
    //         if (g > 255) g = 255;
    //         else if (g < 0) g = 0;

    //     CRGB col = CRGB(r,g,b);
    //     //Serial.println ( String (r) + ", "+ String(g) + ", " + String(b) + ", " + String(w));
    //     return col;
    // };

};