

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

     
        if (radius == RADIUS_STREIGHT) {
            float wheelSpeed = speed /wheel_circumference* 2*PI;
            setSpeed(wheelSpeed, -wheelSpeed);
        }
        else if (radius == 0.0) {
            float wheelSpeed = speed / half_wheel_base;
            setSpeed(-wheelSpeed, -wheelSpeed); //TO BE CHECKED
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
          
            Serial.println("left: " + (String) left + " , right: " + (String) right);
            setSpeed(left, right);
        }
    };

    void DifferentialKinematics::turnLeftForward(float speed, float radius){

        float left =   (radius ) * speed /wheel_radius;
        float right = -(radius + 2* half_wheel_base) * speed /wheel_radius;

        Serial.println("TURN LEFT FORWARD");
        Serial.println("left: " + (String) left + " , right: " + (String) right);
        setSpeed(left, right);

    };
    void DifferentialKinematics::turnRightForward(float speed, float radius){

        float left =   (radius + 2* half_wheel_base ) * speed /wheel_radius;
        float right = -(radius ) * speed /wheel_radius;

        Serial.println("TURN RIGHT FORWARD");
        Serial.println("left: " + (String) left + " , right: " + (String) right);
        setSpeed(left, right);

    };

        void DifferentialKinematics::turnLeftBackward(float speed, float radius){

        float left =   (radius ) * speed /wheel_radius;
        float right = -(radius + 2* half_wheel_base) * speed /wheel_radius;

        Serial.println("TURN LEFT BACKWARD");
        Serial.println("left: " + (String) left + " , right: " + (String) right);
        setSpeed(left, right);

    };
    void DifferentialKinematics::turnRightBackward(float speed, float radius){

        float left =   (radius + 2* half_wheel_base ) * speed /wheel_radius;
        float right = -(radius ) * speed /wheel_radius;

        Serial.println("TURN RIGHT BACKWARD");
        Serial.println("left: " + (String) left + " , right: " + (String) right);
        setSpeed(left, right);

    };


    void DifferentialKinematics::rotate(float speed) {
        move(speed, 0.0f);
    };


 
    float DifferentialKinematics::wheel_dist_to_rad (float dist){
        
        return dist / wheel_circumference * 2* PI;
    };

    float DifferentialKinematics::wheel_rad_to_dist (float rad)
    {
        return rad / (2*PI) * wheel_circumference;         
    };

    float DifferentialKinematics::shaft_vel_to_wheel_vel_rad(float rad)
    {
        return rad * wheel_radius;
    };

    Pose DifferentialKinematics::wheelVelToNextPose (float vL, float vR, int deltaT, Pose lastPose, String curDirName){
        
        //Serial.println("vL: " + (String) vL + " , vR: " + (String) vR);

        vR = -vR; //the way motors are mounted

        vL = vL * wheel_radius; //at shaft --> at wheel, wheel tangential velocities
        vR = vR * wheel_radius;

        float aV = (vR-vL)/wheel_base;//angular velocity of robot
        float R = half_wheel_base *(vR + vL)/(vR - vL); //R = distance from ICR to the center of the robot

        //Serial.println ("R: " + (String) R + "mm");
        //Serial.println ("angular velocity: " + (String) aV + "unit?");

        float V = (vR + vL)/2.0 ; //instantaneous velocity V of the point midway between the robot's wheels

        //Serial.println ("instantaneous velocity midway between wheels: " + (String) V + "unit?");


        deltaTseconds = deltaT / 1000.0f;
        //Serial.println("deltaTseconds: " +  (String) deltaTseconds);

        float theta = lastPose.angle;
        float L = half_wheel_base;

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
  
 


        //float left =  (radius + half_wheel_base) * vRobotAng /wheel_radius;

        // left * wheel_radius/  (radius + half_wheel_base) =   vRobotAng ;
        // right *wheel_radius/ (radius - half_wheel_base) = vRobotAng ;

        // vRobotAng =   ( left * wheel_radius/  (radius + half_wheel_base) - right *wheel_radius/ (radius + half_wheel_base) )/2

        //vRobotAng =   ( (vL  - vR) *wheel_radius/ (radius + half_wheel_base) )/2
        

        // float right = -(radius - half_wheel_base) * speed /wheel_radius;

        // Serial.println("left: " + (String) left + " , right: " + (String) right);

        // vL = vL * wheel_radius; //at shaft --> at wheel
        // vR = vR * wheel_radius;
        //Serial.println("vL: " + (String) vL + ", vR: " + (String) vR);

        
        // deltaTseconds = deltaT / 1000.0f;
        // //Serial.println("deltaTseconds: " +  (String) deltaTseconds);

        

    //     //Serial.println("curDirName: " + curDirName);

    //     if(curDirName.equals("S")) //GOING STRAIGHT
    //     {
    //         float vRobot = (vR + vL)/2.0 ; //average the two, account for negative sign of motor right
    //         //vRobot = wheel_rad_to_dist (vRobot); //convert from angular velocity to mm
    //         pose.x += vRobot * cos(lastPose.angle)*deltaTseconds;
    //         pose.y += vRobot * sin(lastPose.angle)*deltaTseconds;
    //         pose.angle = lastPose.angle;
    //     }
    //     else
    //     {
        
    //         //if (vR - vL !=0) R = half_wheel_base *(vL + vR)/(vR - vL);
    //         //else 
    //         // R = minRadius;
    //         // vRobotAng = (vR - vL)/wheel_base;
        
    //         //R=minRadius;
    //         //Serial.println("R: " +  (String) R);

            
    //     // Serial.println("vRobotAng: " +  (String) vRobotAng);
    //         ICC = Vector(lastPose.x - R * sin(lastPose.angle),  lastPose.y + R * cos (lastPose.angle) );
    //         //Serial.println("ICC: " +  (String) ICC.x + " , " + ICC.y);

    //         pose.x = cos(vRobotAng*deltaTseconds)*(lastPose.x -ICC.x) -sin(vRobotAng*deltaTseconds)*(lastPose.y -ICC.y) + 0*lastPose.angle +ICC.x;
    //         pose.y = sin(vRobotAng*deltaTseconds)*(lastPose.x -ICC.x) +cos(vRobotAng*deltaTseconds)*(lastPose.y -ICC.y) + 0*lastPose.angle +ICC.y;
    //         pose.angle = 0*(lastPose.x -ICC.x) + 0*(lastPose.y -ICC.y) + 1* lastPose.angle + vRobotAng*deltaTseconds;
    //         //pose.angle += globalCoordinateSystemOffsetAngle; //global coordinate systyem
    //     }
    //    // Serial.println ("pose : " + (String) pose.x+ ", " +(String) pose.y+ ", " + (String) degrees(pose.angle)) ;
       
        return pose;
    };

    float DifferentialKinematics::poseToLeftWheelDist (Pose pose){
        float left = 0;
        return left;
    };
    float DifferentialKinematics::poseToRightWheelDist (Pose pose){
        float right = 0;
        return right;
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
        Serial.println ("minRadius: " + (String)minRadius);
        this->startPose = start;
        this->endPose = end;

        this->S = Vector(startPose.x, startPose.y);
        this->E = Vector(endPose.x, endPose.y);

        this->Sdir = rotation(unitX, startPose.angle);
        this->Edir = rotation(unitX, endPose.angle);    

        //RSL = Right, Straight, Left--------------------------------------------------------
        unitV = unit(Sdir);
        cross = crossProduct(unitV, unitZ);
        R1 = S + (cross * minRadius); //center of first circle to turn Right

        unitV = unit(Edir);
        cross = crossProduct( unitV, unitZ);
        L2 = E - (cross* minRadius);   //center of second circle to turn Left

        //middle point
        a = L2 -  R1;
        A = R1 + (a * 0.5); //only because both circles are the same

        //tangent point T1
        float temp = (2*minRadius)/ magnitude(a);
        if (temp >=0 && temp <=1) //value region of acos
        {
            alpha = acos(temp);

            u = unit (a) * (minRadius * cos(alpha)) ;
            unitV = unit(a);
            cross = crossProduct(unitZ, unitV);
            v = cross * (minRadius*sin(alpha));

            T1= R1 + u + v;
        
            //tangent point T2

            RSL = A - (T1 * 2);
            T2 = T1 + RSL;

            allLength[0] = circularArcLengthCW (Sdir, S, T1, minRadius) + distance( T1,T2) +  circularArcLengthCCW (Edir, E, T2, minRadius);
            if (isnan(allLength[0])) allLength[0] = -1;  // should not happen
        } else
        {
            allLength[0] = -1;  
        }                               
        Serial.println (allNames[0]+ ": "+ allLength[0]);

        //RSR = Right, Straight, Right------------------------------------------------------------------
        unitV = unit(Edir);
        cross = crossProduct(unitV, unitZ);
        R2 = E + ( cross*minRadius);

        //parallel line
        b = R2- R1;

        //tangent point T3
        unitV = unit(b);
        cross = crossProduct(unitZ, unitV);
        T3 = R1 + (cross *minRadius);

        //tangent point T4
        T4 = T3 + b;

        allLength[1] = circularArcLengthCW (Sdir, S, T3, minRadius) + distance(T3,T4) +  circularArcLengthCCW (Edir, E, T4, minRadius);
       if (isnan(allLength[1]))  allLength[1] = -1; //should not happen as RSR and LSL is always possible
        Serial.println (allNames[1]+ ": "+ allLength[1]);

        //RLR = Right, Left, Right ----------------------------------------------------------------------
        float v1 = 4.0*minRadius*minRadius;
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
            allLength[2] = circularArcLengthCW (Sdir, S, T5, minRadius) +  circularArcLengthCW (T5dir, T5, T6, minRadius)  +  circularArcLengthCCW (Edir, E, T6, minRadius);

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
        L1 = S - ( cross * minRadius );


        //middle point
        c = R2 - L1;
        C= L1 + (c*0.5); //only because both circles are the same


        //tangent point T7
        temp = (2*minRadius)/ magnitude(c);
        if (temp >=0 && temp <=1) //value region of acos
        {
            alpha = acos(temp);

            u = unit (c) * ( minRadius * cos(alpha)) ;
            unitV = unit(c);
            cross = crossProduct(unitV, unitZ);
            v = cross * (minRadius*sin(alpha));

            T7= L1 + u + v;

            //tangent point T8
            LSR = (C - T7) * 2;
            T8 = T7 + LSR;

            allLength[3] = circularArcLengthCW (Sdir, S, T7, minRadius) + distance (T7,T8) +  circularArcLengthCCW (Edir, E, T8, minRadius);
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
        T9 = L1 - (cross * minRadius);

        ////tangent point T10
        T10 = T9 +d;

        allLength[4] = circularArcLengthCW (Sdir, S, T9, minRadius) + distance (T9,T10) +  circularArcLengthCCW (Edir, E, T10, minRadius);
        if (isnan(allLength[4])) allLength[4] = -1; //should not happen as RSR and LSL is always possible
        Serial.println (allNames[4]+ ": "+ allLength[4]);

        //LRL = Left, Right, Left ----------------------------------------------------------------------
        v1 = 4.0*minRadius*minRadius;
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

            allLength[5] = circularArcLengthCW (Sdir, S, T11, minRadius) + circularArcLengthCCW (T11dir, T11, T12, minRadius) +  circularArcLengthCCW (Edir, E, T12, minRadius);
            if (isnan(allLength[5]) ) allLength[5] = -1;
        } else {
            allLength[5] = -1;
        }
        Serial.println (allNames[5]+ ": "+ allLength[5]);

        getShortestPathIndex ();
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

        Serial.println ("Shortest path is: "+ allNames[shortestPathIndex]+ "  with distance: "+ allLength[shortestPathIndex]+ "   at minRadius: "+ minRadius);



        //RSL = Right, Straight, Left--------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("RSL"))
        {
            //set values for shortest path
            // Serial.println("set values for RSL");
            this->arcCenter1 = R1; 
            // Serial.println("arcCenter1: " + (String) arcCenter1.x + " , "+ (String) arcCenter1.y);
            this->arcRadius1 = minRadius; 
            // Serial.println("arcRadius1: " + (String) arcRadius1);
            this->arcAngle1 = circularArcAngleCW (Sdir, S, T1); this->arcDirName1 ="R";
            // Serial.println("arcAngle1: " + (String) arcAngle1);

            this->arcCenter2 = L2; this->arcRadius2 = minRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T2); this->arcDirName2 ="L";

            // Serial.println("arcCenter2: " + (String) arcCenter2.x + " , "+  (String) arcCenter2.y);
            // Serial.println("arcRadius2: " + (String) arcRadius2);
            // Serial.println("arcAngle2: " + (String) arcAngle2);

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";
            // Serial.println("arcCenter12: " + (String) arcCenter12.x + " , "+  (String) arcCenter12.y);
            // Serial.println("arcRadius12: " + (String) arcRadius12);
            // Serial.println("arcAngle12: " + (String) arcAngle12);

            this->lineStart = T1; this->lineEnd = T2; this->lineLength = distance(lineStart, lineEnd);
            // Serial.println("lineStart: " + (String) lineStart.x +  " , "+ (String) lineStart.y);
            // Serial.println("lineEnd: " + (String) lineEnd.x +  " , "+ (String) lineEnd.y);
            // Serial.println("lineLength: " + (String) lineLength);
            
        }
        
        //RSR = Right, Straight, Right------------------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("RSR"))
        {
            //set values for shortest path
            this->arcCenter1 = R1; this->arcRadius1 = minRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T3); this->arcDirName1 ="R";

            this->arcCenter2 = R2; this->arcRadius2 = minRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T4); this->arcDirName2 ="R";

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";

            this->lineStart = T3; this->lineEnd = T4;this->lineLength = distance(lineStart, lineEnd);
        }
        //RLR = Right, Left, Right ----------------------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("RLR"))
        {
            //set values for shortest path
            this->arcCenter1 = R1; this->arcRadius1 = minRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T5);  this->arcDirName1 ="R";

            this->arcCenter2 = R2; this->arcRadius2 = minRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T6); this->arcDirName2 ="R";

            this->arcCenter12 = L3; this->arcRadius12 = minRadius; this->arcAngle12 =  circularArcAngleCW (T5dir, T5, T6); this->arcDirName12 ="L";

            this->lineStart = Vector(); this->lineEnd = Vector();this->lineLength = 0;
        }
        //--------------------------------------------------------------------------------------------------------------------------------------
        // MIRROWED
        //--------------------------------------------------------------------------------------------------------------------------------------

        //LSR = Left, Straight, Right --------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("LSR"))
        {
            //set values for shortest path
            this->arcCenter1 = L1; this->arcRadius1 = minRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T7); this->arcDirName1 ="L";

            this->arcCenter2 = R2; this->arcRadius2 = minRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T8); this->arcDirName2 ="R";

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";

            this->lineStart = T7; this->lineEnd = T8;this->lineLength = distance(lineStart, lineEnd);
        }
        //LSL = Left, Straight, Left ------------------------------------------------------------------
    if ( allNames[shortestPathIndex].equals("LSL"))
        {
            //set values for shortest path
            this->arcCenter1 = L1; this->arcRadius1 = minRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T9); this->arcDirName1 ="L";

            this->arcCenter2 = L2; this->arcRadius2 = minRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T10); this->arcDirName2 ="L";

            this->arcCenter12 = Vector(); this->arcRadius12 = 0; this->arcAngle12 = 0; this->arcDirName12 ="S";

            this->lineStart = T9; this->lineEnd = T10;this->lineLength = distance(lineStart, lineEnd);
        }
        //LRL = Left, Right, Left ----------------------------------------------------------------------
        if ( allNames[shortestPathIndex].equals("LRL"))
        {
            //set values for shortest path
            this->arcCenter1 = L1; this->arcRadius1 = minRadius; this->arcAngle1 = circularArcAngleCW (Sdir, S, T11);this->arcDirName1 ="L";

            this->arcCenter2 = L2; this->arcRadius2 = minRadius; this->arcAngle2 = circularArcAngleCCW (Edir, E, T12);this->arcDirName2 ="L";

            this->arcCenter12 = R3; this->arcRadius12 = minRadius; this->arcAngle12 =  circularArcAngleCW (T11dir, T11, T12); this->arcDirName12 ="R";

            this->lineStart = Vector(); this->lineEnd = Vector();this->lineLength = 0;
        }

        return shortestPathIndex;
    };

    String DifferentialPathPlanner::getShortestPathName()
    {
        return allNames[shortestPathIndex];
    };


  
    
}; // namespace SmallRobots