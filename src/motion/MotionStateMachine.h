#pragma once
#include "Arduino.h"
#include "../config/SmallRobotConfig.h"
#include "../control/SmallRobotControl.h"
#include "control/SmallRobotEventBus.h"

#include "StateMachine.h"
#include "DifferentialKinematics.h"
#include "Odometry.h"

#define ODOMETRY_UPDATE_RATE_TIMEOUT 1000 //ms

namespace SmallRobots {


 class MotionStateMachine {
    public:
        StateMachine machine;  

        int subPathIndex = 0;   //to identify which of the 3 path parts of the dubin path are currently dealt with

        STATE(idle);
        STATE(new_target_pose);
        STATE(moving);
        STATE(arrived_at_tangentA);
        STATE(arrived_at_tangentB);
        STATE(arrived_at_target_pose);
        STATE(waiting); //for not requesting odometry infos too fast

        TRANSITION(idle2new_target_pose,set_new_pose, idle, new_target_pose);
        TRANSITION(new_target_pose2moving, start_seg1,new_target_pose, moving);
        TRANSITION(moving2arrived_at_tangentA, finished_seg1, moving, arrived_at_tangentA);
        TRANSITION(arrived_at_tangentA2moving, start_seg2, arrived_at_tangentA, moving);
        TRANSITION(moving2arrived_at_tangentB, finished_seg2, moving, arrived_at_tangentB);
        TRANSITION(arrived_at_tangentB2moving, start_seg3, arrived_at_tangentB, moving);
        TRANSITION(moving2arrived_at_target_pose, finished_seg3, moving, arrived_at_target_pose);
        TRANSITION(arrived_at_target_pose2idle, next, arrived_at_target_pose, idle);
        // TRANSITION(moving2waiting,wait, moving, waiting);
        // TRANSITION(waiting2moving,waiting_timeout, waiting, moving); //timeout needs:  trigger(current_state->name+"_timeout");
        TRANSITION(moving2moving,moving_timeout, moving, moving);
        TRANSITION(arrived_at_target_pose2new_target_pose, follow_path,arrived_at_target_pose,new_target_pose);

        TRANSITION(moving2idle, pause, moving, idle);

       
        MotionStateMachine(MotionController& _ctrl) : ctrl(_ctrl){
            machine.all_states = {&idle, &new_target_pose, &moving, &arrived_at_tangentA, &arrived_at_tangentB, &arrived_at_target_pose};
            machine.all_transitions = { &idle2new_target_pose, 
                                        &new_target_pose2moving,
                                        &moving2arrived_at_tangentA, 
                                        &arrived_at_tangentA2moving,
                                        &moving2arrived_at_tangentB,
                                        &arrived_at_tangentB2moving,
                                        &moving2arrived_at_target_pose,
                                        &arrived_at_target_pose2idle,
                                        &moving2moving,
                                        //&moving2waiting,
                                        //&waiting2moving,
                                        &arrived_at_target_pose2new_target_pose,

                                        &moving2idle
                                        };
            machine.initial_state = &idle;

            new_target_pose.enter = std::bind(&MotionStateMachine::on_enter_new_target_pose, this);
            arrived_at_tangentA.enter =  std::bind(&MotionStateMachine::on_enter_arrived_at_tangentA, this);
            arrived_at_tangentB.enter = std::bind(&MotionStateMachine::on_enter_arrived_at_tangentB, this);
            arrived_at_target_pose.enter = std::bind(&MotionStateMachine::on_enter_arrived_at_target_pose, this);
            moving.enter = std::bind(&MotionStateMachine::on_enter_moving, this);
            idle.enter =  std::bind(&MotionStateMachine::on_enter_idle, this);

            //waiting.timeout = ODOMETRY_UPDATE_RATE_TIMEOUT;
            moving.timeout = ODOMETRY_UPDATE_RATE_TIMEOUT;

            moving2idle.on = std::bind(&MotionStateMachine::on_pause, this);
            idle2new_target_pose.on =  std::bind(&MotionStateMachine::on_go, this);
        };
        ~MotionStateMachine() {};

        
        void on_pause()
        {
            ctrl.stopMoving();
        };

             
        void on_go()
        {
           ctrl.enableMotors();
        };
        
        
        void on_enter_idle()
        {
            if (first_on) {
                Serial.println("Motion idle -  disable Motors");
                ctrl.stopMoving();
                first_on = false;
            }

        };

        void on_enter_new_target_pose() { 
            //reset subPathIndex
            subPathIndex = 0;
            Serial.println("ctrl.setTarget()");
            ctrl.setTarget();
            Serial.println("ctrl.setWheelVelocitiesSeg1()");
            ctrl.setWheelVelocitiesSeg1();
            Serial.println("odometry.resetLastTime()");
            odometry.resetLastTime();
            Serial.println("machine.trigger(start_seg1)");
            machine.trigger("start_seg1");
        };
        void on_enter_arrived_at_tangentA() { 
            ctrl.setWheelVelocitiesSeg2();
            odometry.resetLastTime();
            machine.trigger("start_seg2");
        };

        void on_enter_arrived_at_tangentB() { 
            ctrl.setWheelVelocitiesSeg3();
            odometry.resetLastTime();
            machine.trigger("start_seg3");
        };

        void on_enter_arrived_at_target_pose() { 
            //stop moving, give feedback that next pose in path can be executed
            //if next pose in path -> go to new pose
            ctrl.loopPath();
            machine.trigger("set_new_pose");
            //otherwise stop moving TODO
            //ctrl.stopMoving();

        };
        
        void on_enter_moving() {
            //update odometry
            //check if arrived
            //no -> repeat this state
            //yes -> move to next state 
           
            odometry.updatePose();
            Pose curPose = odometry.getCurPose();
            ctrl.setCurPose(curPose);

            Serial.println ("curPose : " + (String) curPose.x+ (String) curPose.y+ (String) curPose.angle) ;
            
            if (ctrl.checkIfArrived()){
                if (subPathIndex == 0) machine.trigger("finished_seg1");
                else if (subPathIndex == 1) machine.trigger("finished_seg2");
                else if (subPathIndex == 2) machine.trigger("finished_seg3");

                subPathIndex++;
            } 
           // else machine.trigger("wait");
        };

        void trigger(String arg)
        {
            machine.trigger(arg);
        };

        void triggerSetNewPose()
        {
            machine.trigger("set_new_pose");
        };

        void start(){
            machine.start();
        };

        void tick() {
            machine.tick();
        };

     
        protected:
            MotionController& ctrl;
            Odometry odometry = Odometry(ctrl.kinematics);
            bool first_on = true;
        


    }; // class MotionStateMachine

};  // namespace SmallRobots



