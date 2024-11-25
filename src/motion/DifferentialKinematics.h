
#pragma once

#include <stdint.h>
#include <limits>
#include "Arduino.h"
#include "../config/globalStructs.h"


#define RADIUS_STREIGHT (std::numeric_limits<float>::infinity())


namespace SmallRobots {

    // typedef struct 
    // {
    //     float x = 0;
    //     float y = 0;
    //     float angle = 0;
    // } Pose;

    class DifferentialKinematics {
        public:
            DifferentialKinematics(float _wheel_base, float wheel_diameter);
            ~DifferentialKinematics();

            virtual void setSpeed(float left, float right) = 0;

            virtual void move(float speed, float radius = RADIUS_STREIGHT);
            virtual void rotate(float speed);
            virtual void stop();
            virtual void enable() = 0;
            virtual void disable() = 0;
            virtual MotorsPosition getMotorsPosition() = 0;
            virtual MotorsVelocity getMotorsVelocity() = 0;

            float wheel_dist_to_rad (float dist);
            float wheel_rad_to_dist (float rad);

            Pose wheelVelToNextPose (float vL, float vR, int deltaT, Pose lastPose);
            float poseToLeftWheelDist (Pose pose);
            float poseToRightWheelDist (Pose pose);

            float wheel_base;
            float half_wheel_base;
            float wheel_radius;
            float wheel_circumference;
            float default_speed;
    };


    class DifferentialPathPlanner {

        private:

            Pose startPose, endPose; //start and end pose (x,y,angle)
            Vector S, E; //start and end vector of Pose for calculating
            Vector Sdir, Edir; //direction vector rotated around angle of pose

            Vector unitX = Vector(1,0,0);
            Vector unitZ = Vector(0,0,1); 

            Vector R1, R2, R3, L1, L2, L3;

            Vector A, B, C, D;

            Vector T1, T2, T3, T4, T5, T6; //tangent points, R first
            Vector T7, T8, T9, T10, T11, T12; //tangent points, L first
            Vector T5dir, T11dir;

            String name = "Dubin";
            String allNames [6] = {"RSL", "RSR", "RLR", "LSR", "LSL", "LRL"};
            float allLength [6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //order: RSLlen, RSRlen, RLRlen, LSRlen, LSLlen, LRLlen
            int shortestPathIndex = -1; //0=RSL, 1=RSR, 2=RLR, 3=LSR, 4=LSL, 5=LRL


        public:
            DifferentialPathPlanner();
            ~DifferentialPathPlanner();

            // TODO think  about where these methods belong
            // possible model to seperate the concepts
            //  - Kinematics - models the robot dimensions, type of drive etc, 
            //                 thus implementing raw motion e.g. like DifferentialKinematics
            //  - PathPlanner - composes paths in terms of the robot's capabilities, e.g. Point 
            //                  and Shoot, decomposing curves into circular arcs, etc.
            //  - "MotionController"? - executes paths in terms of raw movements over time/distance.
            //                          "DeadReconningMotionController", or "OdometricMotionController"
            //                          could be examples. Keeps state in terms of motions in progress,
            //                          and provides facilities to queue and/or interrupt motions.
           // virtual void move(float distance, float speed, float radius = RADIUS_STREIGHT) = 0;
            //virtual void rotate(float angle, float speed) = 0;
            //virtual void move(float distance, float speed, float radius);
            //virtual void moveTo(float dx, float dy, float da, float speed);

            void calculate(Pose start, Pose end) ;

            int getShortestPathIndex ();
            String getShortestPathName();

            float minRadius = 5.0; //radius of dubin path, variable

            Vector arcCenter1;
            float arcRadius1 = minRadius;
            float arcAngle1 = 0;
            String arcDirName1 = "DUBIN1";

            Vector arcCenter2;
            float arcRadius2 = minRadius;
            float arcAngle2 = 0;
            String arcDirName2 = "DUBIN3";

            Vector arcCenter12;
            float arcRadius12 = minRadius;
            float arcAngle12 = 0;
            String arcDirName12 = "DUBIN2";

            Vector lineStart;
            Vector lineEnd;
            float lineLength = 0;

    };

      class MotionController {

        private:

            std::vector<Pose> path;
            int curPathIndex = 0;


            Pose curPose;
            Pose targetPose;

            float default_speed =0.01;
            float default_speed_Ang = radians(0.05);


            Vector ICC; //Instantaneous Center of Curvature
            float R = 0; //dist between robot's pose and its ICC

            float vRobot =0;
            float vRobotAng = 0;

            float vR =0.00;
            float vL =0.00;

            float targetAngle=0;

            float arriveDistance = 0.01; //precission of arriving behaviour
            float arriveAngleDistance = 0;

            String curDirName = "N";

        public:

                    
                    
            MotionController(DifferentialKinematics& kinematics); //, DifferentialPathPlanner& pathPlanner);
            ~MotionController();


            void addPoseToPath(Pose p);
            void setPoseToReplacePath(Pose p);
            void setTarget(); //get next pose in path, calculate dubin path from current pose and target pose
            void setWheelVelocitiesSeg1(); //ARC left or right with minRad
            void setWheelVelocitiesSeg2(); //STRAIGHT or left or right ARC
            void setWheelVelocitiesSeg3(); //ARC left or right
            void stopMoving();
            void enableMotors();

            void setCurPose(Pose pose);

            void loopPath();
            bool checkIfArrived();

            DifferentialKinematics& kinematics;
            DifferentialPathPlanner pathPlanner = DifferentialPathPlanner();
    };

}; // namespace SmallRobots
