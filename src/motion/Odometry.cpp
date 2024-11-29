#include "Odometry.h"

namespace SmallRobots {

    Odometry::Odometry(DifferentialKinematics& _kinematics): kinematics(_kinematics)
    {
        lastTime = micros();
    };
    Odometry::~Odometry()
    {};

    void Odometry::updatePose()
    {
        Serial.println("Odometry::updatePose()");
    
        MotorsVelocity vel = kinematics.getMotorsVelocity();

        Serial.println ("vel.left: " + (String) vel.left + ", vel.right: "+ (String) vel.right);
        updateDeltaT();

        float vel_left_rad = vel.left;
        float vel_right_rad = vel.right;
        float vel_left_dist = kinematics.wheel_rad_to_dist(vel_left_rad);
        Serial.println("vel_left_dist: " + (String) vel_left_dist);
        float vel_right_dist = kinematics.wheel_rad_to_dist(vel_right_rad);
        Serial.println("vel_right_rad: " + (String) vel_right_rad);
        
        curPose = kinematics.wheelVelToNextPose(vel_left_dist,vel_right_dist,deltaT, curPose);

    };

    void Odometry::resetLastTime(){
        lastTime = millis();
        Serial.println("lastTime: " + lastTime);
    };
    int Odometry::updateDeltaT(){
        deltaT = millis() - lastTime; //later converted to seconds
        lastTime = millis();
        Serial.println("lastTime: " + lastTime);
        Serial.println("deltaT: " + deltaT);
        return deltaT;
    };
    
    Pose Odometry::getCurPose()
    {
        return curPose;
    };


}; // namespace SmallRobots