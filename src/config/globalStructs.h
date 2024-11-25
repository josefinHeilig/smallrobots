
#pragma once
#include <inttypes.h>
#include "simplefoc/I2CCommanderMaster.h"

struct MotorsStatus {
    simplefoc::FOCMotorStatus left;
    simplefoc::FOCMotorStatus right;
};

struct MotorsPosition {
    int32_t left_turns;
    float left;
    int32_t right_turns;
    float right;
};

struct MotorsVelocity {
    float left;
    float right;
};


namespace SmallRobots {

    typedef struct 
    {
        float x = 0;
        float y = 0;
        float angle = 0;
    } Pose;


    // typedef struct 
    //     {
    //         float x = 0;
    //         float y = 0;
    //         float z = 0;
    //     } Vector;


    class Vector {

    public:

        float x = 0;
        float y = 0;
        float z = 0;

        // Vector& operator*(float v) {
           
        //    this->x *=v;
        //    this->y *=v;
        //    this->z *=v;
        //    return *this;
        // };

        // Vector& operator+(Vector A) {
           
        //    this->x +=A.x;
        //    this->y +=A.y;
        //    this->z +=A.z;
        //    return *this;
        // };

        Vector operator*(float v) {
           Vector B;
           B.x = this->x *v;
           B.y = this->y *v;
           B.z = this->z *v;
           return B;
        };

        Vector operator+(Vector A) {
           Vector B;
           B.x = this->x +A.x;
           B.y = this->y +A.y;
           B.z = this->z +A.z;
           return B;
        };

        Vector operator-(Vector A) {
           Vector B;
           B.x = this->x -A.x;
           B.y = this->y -A.y;
           B.z = this->z -A.z;
           return B;
        };

    

     private:
    
    };

    Vector rotation (Vector dir, float angle);

    float circularArcLengthCW (Vector dirA, Vector A, Vector B, float radius); 
    float circularArcAngleCW( Vector dirA, Vector A, Vector B);
    float circularArcLengthCCW (Vector dirA, Vector A, Vector B, float radius); 
    float circularArcAngleCCW( Vector dirA, Vector A, Vector B);

    Vector crossProduct (Vector A, Vector B);
    float scalarProduct(Vector A, Vector B);

    float magnitude (Vector A);
    Vector unit (Vector A);
    float distance (Vector A, Vector B);

  
}; //end: namespace SmallRobots