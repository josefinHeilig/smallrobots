#pragma once

#include "./Behaviours.h"
#include <OSCMessage.h>
#include "control/SmallRobotControl.h"
#include "esp32/wifi/WiFiStateMachine.h"



#define FEEDBACK_UPDATE_RATE 10000 //how often is the led updated in the behaviour engine


namespace SmallRobots {


   class FeedbackBehaviour : public Behaviour {
        public:
            FeedbackBehaviour() : Behaviour(FEEDBACK_UPDATE_RATE) {

                

            };

            virtual Behaviour* run() {

                if (udp.connected()){ 
                    OSCMessage msg ("/feedback");
                    // //msg.add(name);
                   
                   
                    //msg.sendTo(udp,UDP_PORT_SEND); 
        
                    udp.broadcastTo("feedback", UDP_PORT_SEND);  //TODO SEND OSC MESSAGE?

                    msg.empty();
                    Serial.println ("feedback");
                }
                return this;
              
            };

            virtual const char* getName() override { return name.c_str(); };
           

        protected:
            String name = "FeedbackBehaviour";
            
    };


}
