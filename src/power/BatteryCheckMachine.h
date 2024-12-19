#pragma once

#include "Arduino.h"
#include "./StateMachine.h"
#include "../config/SmallRobotConfig.h"
#include "../control/SmallRobotControl.h"
#include "control/SmallRobotEventBus.h"

#include "./../../src/WhisperDualDriver.h"


namespace SmallRobots {
    
    #define BATTERY_CHECK_INTERVAL 30000

    #define V_OVERCHARGED 8.6     // -> 8.6 V input
    #define V_FULLYCHARGED 8.4    // -> 8.4 V input
    #define V_LOWWARNING 7.4      // -> 6.0 V input
    #define V_LOWOFF 5.6           // 1.75; //-> 5.6 V input   


    class BatteryCheckStateMachine {
    public:
        StateMachine machine;

        bool battery_read_error = false;

        STATE(reading); //read, disable, wait disabled longer
        STATE(failed);

        TRANSITION(reading2reading, reading_timeout, reading, reading);
        TRANSITION(reading2failed, battery_reading_failed, reading, failed);

        BatteryCheckStateMachine(WhisperDualDriver & _drive): drive(_drive)
        {
            machine.all_states = {&reading,&failed};
            machine.all_transitions = {&reading2reading, &reading2failed};
            machine.initial_state = &reading;

            //could also be written like this: enable.enter = [this](){ BatteryCheckStateMachine::on_enter_enable(); };
            reading.enter = std::bind(&BatteryCheckStateMachine::on_enter_reading, this);
            failed.enter = std::bind(&BatteryCheckStateMachine::on_enter_failed, this);
        
            
            
            reading.timeout = BATTERY_CHECK_INTERVAL; 

            // reading2enable.guard = [this](){
            //     return !battery_read_error;
            // };
            // reading2failed.guard = [this](){
            //     return battery_read_error;
            // };
            
        };
        ~BatteryCheckStateMachine() {};
        

        void on_enter_reading() {
            //Serial.println("BATTERY CHECK");
            battery_voltage = drive.getBatteryVoltage();
            Serial.println (String(battery_voltage) + " V");
            

            // if(isOverCharged()) event_bus.emit("robot_shutdown");
            if(isLowWarning()) event_bus.emit("battery_low");
            // if(isLowOff()) event_bus.emit("robot_shutdown");

        };

        void on_enter_failed() {
            Serial.println("BatteryReadStateMachine: on_enter_failed");
        };
    
    
        void start() {
            machine.start();
        };
        
        void tick() {
            machine.tick();
        };

        float getMaxVoltage(){
            return V_FULLYCHARGED;
        }

        float getMinVoltage(){
            return V_LOWOFF;
        }

        float getVoltage(){
            return battery_voltage;
        }

        protected:
            WhisperDualDriver& drive;
            float battery_voltage=0;

            bool isOverCharged(){
            return battery_voltage>V_OVERCHARGED;
            }

            bool isLowWarning(){
                return battery_voltage<V_LOWWARNING && battery_voltage>V_LOWOFF;
            }

            bool isLowOff(){
                return battery_voltage<V_LOWOFF;
            }


    };

};