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
    
        MotorsVelocity vel = kinematics.getMotorsVelocity();
        updateDeltaT();

        float vel_left_rad = vel.left;
        float vel_right_rad = vel.right;
        float vel_left_dist = kinematics.wheel_rad_to_dist(vel_left_rad);
        float vel_right_dist = kinematics.wheel_rad_to_dist(vel_right_rad);
        
        curPose = kinematics.wheelVelToNextPose(vel_left_dist,vel_right_dist,deltaT, curPose);

    };

    void Odometry::resetLastTime(){
        lastTime = micros();
    };
    int Odometry::updateDeltaT(){
        deltaT = micros() - lastTime; //later converted to seconds
        lastTime = micros();
        return deltaT;
    };
    
    Pose Odometry::getCurPose()
    {
        return curPose;
    };


}; // namespace SmallRobots