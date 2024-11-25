#pragma once

#include "Arduino.h"
#include "./StateMachine.h"
#include "../config/SmallRobotConfig.h"
#include "../control/SmallRobotControl.h"
#include "../power/BatteryCheckService.h"
#include "control/SmallRobotEventBus.h"

namespace SmallRobots {
    
    #define BATTERY_CHECK_INTERVAL 2000
    #define WAIT_TIME_AFTER_ENABLE 10

    class BatteryCheckStateMachine {
    public:
        StateMachine machine;
        BatteryChecker batteryChecker;

        bool battery_read_error = false;

        
        STATE(enable); //enable, wait enabled very short
        STATE(reading); //read, disable, wait disabled longer
        STATE(failed);

        
        TRANSITION(enable2reading, enable_timeout, enable, reading);
        TRANSITION(reading2enable, reading_timeout, reading, enable);
        TRANSITION(reading2failed, battery_reading_failed, reading, failed);

        BatteryCheckStateMachine() {
            machine.all_states = {&enable, &reading,&failed};
            machine.all_transitions = {&enable2reading, &reading2enable, &reading2failed};
            machine.initial_state = &enable;

            enable.enter = std::bind(&BatteryCheckStateMachine::on_enter_enable, this);  //could also be written like this: enable.enter = [this](){ BatteryCheckStateMachine::on_enter_enable(); };
            reading.enter = std::bind(&BatteryCheckStateMachine::on_enter_reading, this);
            failed.enter = std::bind(&BatteryCheckStateMachine::on_enter_failed, this);
        
            
            enable.timeout = WAIT_TIME_AFTER_ENABLE;
            reading.timeout = BATTERY_CHECK_INTERVAL;

            reading2enable.guard = [this](){
                return !battery_read_error;
            };
            reading2failed.guard = [this](){
                return battery_read_error;
            };
            
        };
        ~BatteryCheckStateMachine() {};
        
   
        void on_enter_enable() {
            batteryChecker.enableReadVoltage();
        };

        void on_enter_reading() {
            battery_read_error = batteryChecker.readVoltage();
            float v = batteryChecker.getVoltage();
            //Serial.println (String(v) + " V");
            batteryChecker.disableReadVoltage();
            battery_voltage = v;

            if(batteryChecker.isOverCharged()) event_bus.emit("robot_shutdown");
            if(batteryChecker.isLowWarning()) event_bus.emit("robot_warn_battery_low");
            if(batteryChecker.isLowOff()) event_bus.emit("robot_shutdown");

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


    };

};