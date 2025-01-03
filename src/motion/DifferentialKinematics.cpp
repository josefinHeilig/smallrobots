

#include "./DifferentialKinematics.h"



namespace SmallRobots {

    DifferentialKinematics::DifferentialKinematics(float _wheel_base, float wheel_diameter) {
        default_speed = 0.5;
        half_wheel_base = _wheel_base/2.0f;
        wheel_base = _wheel_base;
        wheel_radius = wheel_diameter/2.0f;
        wheel_circumference = 2*wheel_radius * PI;
    };

    DifferentialKinematics::~DifferentialKinematics() {
    };




    void DifferentialKinematics::move(float speed, float radius) { //negative radius + negative speed when leftturn = counterclockwise
        
        //speed in mm/s for straight
        //speed in mm/s for movement on arc
        setCurRobotSpeed(speed);
        setCurRobotRadius(radius);
     
        if (radius == RADIUS_STREIGHT) {
            float wheelSpeed = speed /wheel_circumference* 2*PI;
            setSpeed(wheelSpeed, -wheelSpeed);
        }
        else if (radius == 0.0) {
            float wheelSpeed = speed / half_wheel_base;
            setSpeed(-wheelSpeed, -wheelSpeed); 
        }
        else {

            float angularSpeed = speed / abs(radius);
            if (abs(radius) <= wheel_base) angularSpeed = speed / wheel_base;
            

            //R is ICC to the center of the robot
            // // LEFT ARC (+radius) FORWARD (+speed) or BACKWARD (-speed)
            float left =   (radius - half_wheel_base) * angularSpeed /wheel_radius;
            float right = -(radius + half_wheel_base) * angularSpeed /wheel_radius;

            //RIGHT ARC (-radius) FORWARD (-speed) or BACKWARD (+speed)
            if(radius <0 ) {
                radius = abs(radius);
                left =   (radius + half_wheel_base) * angularSpeed /wheel_radius;
                right = -(radius - half_wheel_base) * angularSpeed /wheel_radius;
            }

            //WORKS TOO
            //R is ICC to the outside wheel of the robot
            // // LEFT ARC (+radius) FORWARD (+speed) or BACKWARD (-speed)
            // float left =   (radius) * angularSpeed /wheel_radius;
            // float right = -(radius + 2* half_wheel_base) * angularSpeed /wheel_radius;
            
            // //RIGHT ARC (-radius) FORWARD (-speed) or BACKWARD (+speed)
            // if(radius <0 ) {
            //     radius = abs(radius);
            //     left =   (radius + 2* half_wheel_base) * angularSpeed /wheel_radius;
            //     right = -(radius ) * angularSpeed /wheel_radius;
            // }
          
            // Serial.println("left: " + (String) left + " , right: " + (String) right);
            setSpeed(left, right);

    
        }
    };
    void DifferentialKinematics::turnLeftForward(float speed, float radius){//cannot be used to go into another direction than specified,to guarantee the direction, values are turned absolute
        move(abs(speed),abs(radius));
    };
    void DifferentialKinematics::turnRightForward(float speed, float radius){
        move(abs(speed),-abs(radius));
    };

    void DifferentialKinematics::turnLeftBackward(float speed, float radius){
        move(-abs(speed),abs(radius));
    };
    void DifferentialKinematics::turnRightBackward(float speed, float radius){
        move(-abs(speed),-abs(radius));
    };

    void DifferentialKinematics::rotate(float speed) {
        move(speed, 0.0f);
    };


    Pose DifferentialKinematics::wheelVelToNextPose (float vL, float vR, int deltaT, Pose lastPose, String curDirName){
        
        //Serial.println("vL: " + (String) vL + " , vR: " + (String) vR);

        vR = -vR; //the way motors are mounted

        vL = vL * wheel_radius; //at shaft --> at wheel, wheel tangential velocities
        vR = vR * wheel_radius;

        // float aV = (vR-vL)/wheel_base;//angular velocity of robot
        // float R = half_wheel_base *(vR + vL)/(vR - vL); //R = distance from ICR to the center of the robot

        //Serial.println ("R: " + (String) R + "mm");
        //Serial.println ("angular velocity: " + (String) aV + "unit?");

        // float V = (vR + vL)/2.0 ; //instantaneous velocity V of the point midway between the robot's wheels

        //Serial.println ("instantaneous velocity midway between wheels: " + (String) V + "unit?");


        deltaTseconds = deltaT / 1000.0f;
        //Serial.println("deltaTseconds: " +  (String) deltaTseconds);

        // float theta = lastPose.angle;
        // float L = half_wheel_base;

        float deltaSR = vR*deltaTseconds;
        float deltaSL = vL*deltaTseconds;

        // float deltaTheta = (deltaSR-deltaSL)/(2*L);
        // float deltaS = (deltaSR+deltaSL)/2.0;

        // float deltaX = deltaS * cos(theta + deltaTheta/2.0);
        // float deltaY = deltaS * sin(theta + deltaTheta/2.0);
    

        //R is distance to outside wheel
        //WORKS
        // pose.x = lastPose.x +  (deltaSR+deltaSL)/2.0 * cos(lastPose.angle + (deltaSR-deltaSL)/(4*half_wheel_base)) ;
        // pose.y = lastPose.y +  (deltaSR+deltaSL)/2.0 * sin(lastPose.angle + (deltaSR-deltaSL)/(4*half_wheel_base)) ;
        // pose.angle = lastPose.angle + (deltaSR-deltaSL)/(2*half_wheel_base);
         
        //Serial.println ("pose : " + (String) pose.x+ ", " +(String) pose.y+ ", " + (String) degrees(pose.angle)) ;
       

        //R is distance to robot center
        pose.x = lastPose.x + (deltaSR+deltaSL)/2.0  * cos( lastPose.angle ) ;
        pose.y = lastPose.y + (deltaSR+deltaSL)/2.0  * sin( lastPose.angle  ) ;
        pose.angle = lastPose.angle + (deltaSR-deltaSL)/(2*half_wheel_base);

        Serial.println ("pose : " + (String) pose.x+ ", " +(String) pose.y+ ", " + (String) degrees(pose.angle)) ;
  
        return pose;
    };

    void DifferentialKinematics::setCurRobotSpeed(float _curRobotSpeed){
        curRobotSpeed = _curRobotSpeed;
    };
    float DifferentialKinematics::getCurRobotSpeed(){
        return curRobotSpeed;
    };

    void DifferentialKinematics::setCurRobotRadius(float _curRobotRadius){
        curRobotRadius =_curRobotRadius;
    };
    float DifferentialKinematics::getCurRobotRadius(){
        return curRobotRadius;
    };

    //------------------------------------------------------------------------------------------------------------------
    //  DifferentialPathPlanner
    //------------------------------------------------------------------------------------------------------------------

    // DifferentialPathPlanner::DifferentialPathPlanner(DifferentialKinematics& kinematics) : kinematics(kinematics) {
     DifferentialPathPlanner::DifferentialPathPlanner() {
    };

    DifferentialPathPlanner::~DifferentialPathPlanner() {
    };

    void DifferentialPathPlanner::calculate(Pose start, Pose end) {
        Serial.println ("turnRadius: " + (String)turnRadius);
        this->startPose = start;
        this->endPose = end;

        this->S = Vector(startPose.x, startPose.y);
        this->E = Vector(endPose.x, endPose.y);

        this->Sdir = rotation(unitX, startPose.angle);
        this->Edir = rotation(unitX, endPose.angle);    
        // Serial.println ("S: " + (String) S.x + ", " + (String) S.y + ", " + (String) S.z +", angle:" + startPose.angle);
        // Serial.println ("E: " + (String) E.x + ", " + (String) E.y + ", " + (String) E.z +", angle:" + endPose.angle);
        // Serial.println ("Sdir: " + (String) Sdir.x + ", " + (String) Sdir.y + ", " + (String) Sdir.z );
        // Serial.println ("Edir: " + (String) Edir.x + ", " + (String) Edir.y + ", " + (String) Edir.z );

        //RSL = Right, Straight, Left--------------------------------------------------------
        unitV = unit(Sdir);
        cross = crossProduct(unitV, unitZ);
        R1 = S + (cross * turnRadius); //center of first circle to turn Right
        // Serial.println ("unitV: " + (String) unitV.x + ", " + (String) unitV.y + ", " + (String) unitV.z );
        // Serial.println ("cross: " + (String) cross.x + ", " + (String) cross.y + ", " + (String) cross.z );
        // Serial.println ("R1: " + (String) R1.x + ", " + (String) R1.y + ", " + (String) R1.z );

        unitV = unit(Edir);
        cross = crossProduct( unitV, unitZ);
        L2 = E - (cross* turnRadius);   //center of second circle to turn Left
        // Serial.println ("unitV: " + (String) unitV.x + ", " + (String) unitV.y + ", " + (String) unitV.z );
        // Serial.println ("cross: " + (String) cross.x + ", " + (String) cross.y + ", " + (String) cross.z );
        // Serial.println ("L2: " + (String) L2.x + ", " + (String) L2.y + ", " + (String) L2.z );

        //middle point
        a = L2 -  R1;
        A = R1 + (a * 0.5); //only because both circles are the same
        // Serial.println("Middle Point");
        // Serial.println ("a: " + (String) a.x + ", " + (String) a.y + ", " + (String) a.z );
        // Serial.println ("A: " + (String) A.x + ", " + (String) A.y + ", " + (String) A.z );

        //tangent point T1
        float temp = (2*turnRadius)/ magnitude(a);
        // Serial.println("tangent point T1");
        // Serial.println ("temp: " + (String) temp);
        if (temp >=0 && temp <=1) //value region of acos
        {
            alpha = acos(temp);
            // Serial.println ("alpha: " + (String) degrees(alpha));

            u = unit (a) * (turnRadius * cos(alpha)) ;
            unitV = unit(a);
            cross = crossProduct(unitZ, unitV);
            v = cross * (turnRadius*sin(alpha));

            T1= R1 + u + v;
        
            //tangent point T2

            RSL = (A - T1) * 2;

            //Serial.println ("RSL: " + (String) RSL.x + ", " + (String) RSL.y + ", " + (String) RSL.z );

            T2 = T1 + RSL;

            allLength[0] = circularArcLengthCW (Sdir, S, T1, turnRadius) + distance( T1,T2) +  circularArcLengthCCW (Edir, E, T2, turnRadius);
            if (isnan(allLength[0])) allLength[0] = -1;  // should not happen
        } else
        {
            allLength[0] = -1;  
        }                               
        Serial.println (allNames[0]+ ": "+ allLength[0]);

        //RSR = Right, Straight, Right------------------------------------------------------------------
        unitV = unit(Edir);
        cross = crossProduct(unitV, unitZ);
        R2 = E + ( cross*turnRadius);

        //parallel line
        b = R2- R1;

        //tangent point T3
        unitV = unit(b);
        cross = crossProduct(unitZ, unitV);
        T3 = R1 + (cross *turnRadius);

        //tangent point T4
        T4 = T3 + b;

        allLength[1] = circularArcLengthCW (Sdir, S, T3, turnRadius) + distance(T3,T4) +  circularArcLengthCCW (Edir, E, T4, turnRadius);
       if (isnan(allLength[1]))  allLength[1] = -1; //should not happen as RSR and LSL is always possible
        Serial.println (allNames[1]+ ": "+ allLength[1]);

        //RLR = Right, Left, Right ----------------------------------------------------------------------
        float v1 = 4.0*turnRadius*turnRadius;
        float v2 = magnitude(b)*magnitude(b)/4.0;
        if (v1 >=v2 ) //otherwise the circles do not touch and sqrt of negative value
        {
            B = R1 + (b*0.5);
            unitV = unit(b);
            w = (crossProduct(unitV, unitZ)) * ( sqrt(v1 -v2) );
            L3 = B+w;

            //tangent point T5
            T5 = R1 + ((L3 -R1) * 0.5 );
            T6 = R2 + ((L3 -R2) *0.5  );
            sub = T5-R1;
            unitV = unit(sub);
            cross = crossProduct( unitV, unitZ);
            T5dir = unit ( cross );
            allLength[2] = circularArcLengthCW (Sdir, S, T5, turnRadius) +  circularArcLengthCW (T5dir, T5, T6, turnRadius)  +  circularArcLengthCCW (Edir, E, T6, turnRadius);

            if (isnan(allLength[2])) allLength[2] = -1; //should not happen with the check
        }
        else{
             allLength[2] = -1;
        }
        Serial.println (allNames[2]+ ": "+ allLength[2]);

        //--------------------------------------------------------------------------------------------------------------------------------------
        // MIRROWED
        //--------------------------------------------------------------------------------------------------------------------------------------

        //LSR = Left, Straight, Right --------------------------------------------------------

        unitV = unit(Sdir);
        cross = crossProduct(unitV, unitZ);
        L1 = S - ( cross * turnRadius );


        //middle point
        c = R2 - L1;
        C= L1 + (c*0.5); //only because both circles are the same


        //tangent point T7
        temp = (2*turnRadius)/ magnitude(c);
        if (temp >=0 && temp <=1) //value region of acos
        {
            alpha = acos(temp);

            u = unit (c) * ( turnRadius * cos(alpha)) ;
            unitV = unit(c);
            cross = crossProduct(unitV, unitZ);
            v = cross * (turnRadius*sin(alpha));

            T7= L1 + u + v;

            //tangent point T8
            LSR = (C - T7) * 2;
            T8 = T7 + LSR;

            allLength[3] = circularArcLengthCW (Sdir, S, T7, turnRadius) + distance (T7,T8) +  circularArcLengthCCW (Edir, E, T8, turnRadius);
            if (isnan(allLength[3])) allLength[3] = -1; //should not happen anymore
        } else
        {
            allLength[3] = -1;
        }
        Serial.println (allNames[3]+ ": "+ allLength[3]);

        //LSL = Left, Straight, Left ------------------------------------------------------------------

        ////parallel line
        d = L2-L1;

        ////tangent point T9
        unitV = unit(d);
        cross = crossProduct(unitZ, unitV);
        T9 = L1 - (cross * turnRadius);

        ////tangent point T10
        T10 = T9 +d;

        allLength[4] = circularArcLengthCW (Sdir, S, T9, turnRadius) + distance (T9,T10) +  circularArcLengthCCW (Edir, E, T10, turnRadius);
        if (isnan(allLength[4])) allLength[4] = -1; //should not happen as RSR and LSL is always possible
        Serial.println (allNames[4]+ ": "+ allLength[4]);

        //LRL = Left, Right, Left ----------------------------------------------------------------------
        v1 = 4.0*turnRadius*turnRadius;
        v2 =  magnitude(d)*magnitude(d)/4.0;
        if (v1 >=v2 ) //otherwise the circles do not touch and sqrt of negative value
        {
            D = L1 +(d * 0.5);

            unitV = unit(d);
            cross = crossProduct(unitV, unitZ);
            w = cross * sqrt( v1 - v2 ) ;
            R3 = D + w;

            //tangent point T11,12
            T11 = L1+ (R3-L1)*0.5;
            T12 = L2+ (R3-L2)*0.5;

            sub = T11-L1;
            unitV = unit(sub);
            cross = crossProduct( unitV, unitZ);
            T11dir = unit (cross );

            allLength[5] = circularArcLengthCW (Sdir, S, T11, turnRadius) + circularArcLengthCCW (T11dir, T11, T12, turnRadius) +  circularArcLengthCCW (Edir, E, T12, turnRadius);
            if (isnan(allLength[5]) ) allLength[5] = -1;
        } else {
            allLength[5] = -1;
        }
        Serial.println (allNames[5]+ ": "+ allLength[5]);

        getShortestPathIndex ();
        setPathParametersOfIndex();
    };

    int DifferentialPathPlanner::getShortestPathIndex ()
    {
        float compL = 1000000000;
        for (int i=0; i< 6; i++)
        {
            //if (!isnan(allLength[i])
            if (allLength[i] != -1) //path does not exist
            {
                float l = allLength[i];
                if (l< compL)
                {
                    compL = l;
                    shortestPathIndex= i;
                }
            }
        }
        
        this->name = &allNames[shortestPathIndex];

        Serial.println ("Shortest path is: "+ allNames[shortestPathIndex]+ "  with distance: "+ allLength[shortestPathIndex]+ "   at turnRadius: "+ turnRadius);
        return shortestPathIndex;
    };

    bool DifferentialPathPlanner::setPathParametersOfIndex(int index)//default: shortestPathIndex, returns true if path exits
    {
        if (index ==-1)index = shortestPathIndex;
        bool pathExists = true;
        if(allLength[index]== -1)pathExists = false;

        //RSL = Right, Straight, Left--------------------------------------------------------
        if ( allNames[index].equals("RSL"))
        {
            this->arcCenter1 = R1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T1); this->arcDirName1 ="R";
            this->arcCenter2 = L2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T2); this->arcDirName2 ="L";
            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";
            this->lineStart = T1; this->lineEnd = T2; this->lineLength = distance(lineStart, lineEnd);           
        }
        //RSR = Right, Straight, Right------------------------------------------------------------------
        if ( allNames[index].equals("RSR"))
        {
            this->arcCenter1 = R1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T3); this->arcDirName1 ="R";
            this->arcCenter2 = R2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T4); this->arcDirName2 ="R";
            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";
            this->lineStart = T3; this->lineEnd = T4;this->lineLength = distance(lineStart, lineEnd);
        }
        //RLR = Right, Left, Right ----------------------------------------------------------------------
        if ( allNames[index].equals("RLR"))
        {
            this->arcCenter1 = R1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T5);  this->arcDirName1 ="R";
            this->arcCenter2 = R2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T6); this->arcDirName2 ="R";
            this->arcCenter12 = L3; this->arcRadius12 = turnRadius; this->arcAngle12 =  circularArcAngleCW (T5dir, T5, T6); this->arcDirName12 ="L";
            this->lineStart = Vector(); this->lineEnd = Vector();this->lineLength = 0;
        }
        //--------------------------------------------------------------------------------------------------------------------------------------
        // MIRROWED
        //--------------------------------------------------------------------------------------------------------------------------------------
        //LSR = Left, Straight, Right --------------------------------------------------------
        if ( allNames[index].equals("LSR"))
        {
            this->arcCenter1 = L1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T7); this->arcDirName1 ="L";
            this->arcCenter2 = R2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T8); this->arcDirName2 ="R";
            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";
            this->lineStart = T7; this->lineEnd = T8;this->lineLength = distance(lineStart, lineEnd);
        }
        //LSL = Left, Straight, Left ------------------------------------------------------------------
    if ( allNames[index].equals("LSL"))
        {
            this->arcCenter1 = L1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T9); this->arcDirName1 ="L";
            this->arcCenter2 = L2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T10); this->arcDirName2 ="L";
            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";
            this->lineStart = T9; this->lineEnd = T10;this->lineLength = distance(lineStart, lineEnd);
        }
        //LRL = Left, Right, Left ----------------------------------------------------------------------
        if ( allNames[index].equals("LRL"))
        {
            this->arcCenter1 = L1; this->arcRadius1 = turnRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T11);this->arcDirName1 ="L";
            this->arcCenter2 = L2; this->arcRadius2 = turnRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T12);this->arcDirName2 ="L";
            this->arcCenter12 = R3; this->arcRadius12 = turnRadius; this->arcAngle12 =  circularArcAngleCW (T11dir, T11, T12); this->arcDirName12 ="R";
            this->lineStart = Vector(); this->lineEnd = Vector();this->lineLength = 0;
        }
        return pathExists;
    };

    String DifferentialPathPlanner::getShortestPathName()
    {
        return allNames[shortestPathIndex];
    };

    void DifferentialPathPlanner::setPathRadius(float _radius){
        turnRadius = _radius;//in mm
    }; 


  
    
}; // namespace SmallRobots