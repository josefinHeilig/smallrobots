

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

   


    void DifferentialKinematics::move(float speed, float radius) { //negative radius when leftturn = counterclockwise
        if (radius == RADIUS_STREIGHT) {
            setSpeed(speed, speed);
        }
        else if (radius == 0.0) {
            setSpeed(speed, -speed);
        }
        else {
            float left = speed * (radius + half_wheel_base);
            float right = speed * (radius - half_wheel_base);
            setSpeed(left, right);
        }
    };



    void DifferentialKinematics::rotate(float speed) {
        move(speed, 0.0f);
    };

    void DifferentialKinematics::stop() {
        Serial.println("STOP");
        this->setSpeed(0,0);
        this->disable();
    }

 
    float DifferentialKinematics::wheel_dist_to_rad (float dist){
        
        return dist / wheel_circumference * 2* PI;
    };
    float DifferentialKinematics::wheel_rad_to_dist (float rad)
    {
        return rad / (2*PI) * wheel_circumference;
         
    };

    Pose DifferentialKinematics::wheelVelToNextPose (float vL, float vR, int deltaT, Pose lastPose){
        Serial.println("DifferentialKinematics::wheelVelToNextPose");
        R = half_wheel_base *(vL + vR)/(vR - vL);
        Serial.println("R: " +  (String) R);
        vRobotAng = (vR - vL)/wheel_base;
        Serial.println("vRobotAng: " +  (String) vRobotAng);
        ICC = Vector(lastPose.x - R * sin(lastPose.angle),  lastPose.y + R * cos (lastPose.angle) );
        Serial.println("ICC: " +  (String) ICC.x + " , " + ICC.y);

        deltaTseconds = deltaT / 1000.0f;
        Serial.println("deltaTseconds: " +  (String) deltaTseconds);

        pose.x = cos(vRobotAng*deltaTseconds)*(lastPose.x -ICC.x) -sin(vRobotAng*deltaTseconds)*(lastPose.y -ICC.y) + 0*lastPose.angle +ICC.x;
        pose.y = sin(vRobotAng*deltaTseconds)*(lastPose.x -ICC.x) +cos(vRobotAng*deltaTseconds)*(lastPose.y -ICC.y) + 0*lastPose.angle +ICC.y;
        pose.angle = 0*(lastPose.x -ICC.x) + 0*(lastPose.y -ICC.y) + 1* lastPose.angle + vRobotAng*deltaTseconds;

        Serial.println("pose.x : " +  (String) pose.x );
        Serial.println("pose.y : " +  (String) pose.y );
        Serial.println("pose.angle : " +  (String) pose.angle );
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


  //------------------------------------------------------------------------------------------------------------------
    //  MotionController
    //------------------------------------------------------------------------------------------------------------------

    MotionController::MotionController(DifferentialKinematics& drive) : kinematics(drive) {
    };

    MotionController::~MotionController() {
    };

    void MotionController::addPoseToPath(Pose p)
    {
        path.push_back(p);

    };
    void MotionController::setPoseToReplacePath(Pose p)
    {
        path.clear();
        path.push_back(p);

    };


    void MotionController::setTarget() //get next pose in path, calculate dubin path from current pose and target pose
    {
        Serial.println ("MotionController::setTarget()");
        Serial.println("curPathIndex: " + (String) curPathIndex);
        Serial.println("path queue length: " + (String) path.size());
        targetPose = path[curPathIndex];
        Serial.println("Target Pose: " + (String)targetPose.x + ", " + (String) targetPose.y + ", " + (String) targetPose.angle);
        pathPlanner.calculate(curPose, targetPose);
        Serial.println ("Update path, shortest path: " + pathPlanner.getShortestPathName());

        Serial.println ("1.) -"+ pathPlanner.arcDirName1+ "- with angle: "+ degrees (pathPlanner.arcAngle1)+ "°, around center: "+ pathPlanner.arcCenter1.x +"," + pathPlanner.arcCenter1.y);
        if (pathPlanner.arcDirName12.equals ("S")) Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with distance: "+ pathPlanner.lineLength+ ", from start: "+ pathPlanner.lineStart.x + ", " +pathPlanner.lineStart.y+ " to end: "+ pathPlanner.lineEnd.x + ", " + pathPlanner.lineEnd.y);
        else Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with angle: "+ degrees (pathPlanner.arcAngle12)+ "°, around center: "+ pathPlanner.arcCenter12.x +"," + pathPlanner.arcCenter12.y);
        Serial.println ("3.) -"+ pathPlanner.arcDirName2+ "- with angle: "+ degrees (pathPlanner.arcAngle2)+ "°, around center: "+ pathPlanner.arcCenter2.x +"," + pathPlanner.arcCenter2.y);

        ICC = pathPlanner.arcCenter1;
        Serial.println ("ICC: " + (String) ICC.x +  " , " + (String) ICC.y);
        curV = Vector (curPose.x, curPose.y);
        Serial.println("curV: " +  (String) curV.x + " , " + (String) curV.y);

        R = distance(curV, ICC);
        Serial.println("R: " + (String) R);
    };

    void MotionController::setWheelVelocitiesSeg1() //ARC left or right with minRad
    {
        //Serial.println ("Set wheel velocities segment 1(3) of path.");
        vRobotAng = default_speed_Ang;

        curDirName = pathPlanner.arcDirName1;

        if (pathPlanner.arcDirName1.equals ("L")) {
          //vR = (R+kinematics.half_wheel_base)* vRobotAng; //SEND TO WHEELS as angular velocities
          //vL = (R-kinematics.half_wheel_base)* vRobotAng;
          kinematics.move(vRobotAng, -R); //negative when left turn = counterclockwise
          targetAngle = pathPlanner.arcAngle1 + curPose.angle ;
        } else if (pathPlanner.arcDirName1.equals ("R")){
          //vR = (R-kinematics.half_wheel_base)* vRobotAng; //SEND TO WHEELS as angular velocities
          //vL = (R+kinematics.half_wheel_base)* vRobotAng;
          kinematics.move(vRobotAng, R); //positive when right turn = clockwise
          targetAngle = -pathPlanner.arcAngle1 + curPose.angle ;
        }

        //Serial.println ("targetAngle: "+ String (degrees(targetAngle))+ " °");

    };
    void MotionController::setWheelVelocitiesSeg2() //STRAIGHT or left or right ARC
    {

        //Serial.println ("Set wheel velocities segment 2(3) of path.");

        curDirName = pathPlanner.arcDirName12;

        if (pathPlanner.arcDirName12.equals ("S")) {

            //Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with distance: "+ pathPlanner.lineLength+ "+ from start: "+ pathPlanner.lineStart.x + ", " +  pathPlanner.lineStart.y + " to end: "+ pathPlanner.lineEnd.x + ", " + pathPlanner.lineEnd.y);
            // GO Straight
            vRobotAng = 0;
            vRobot = default_speed;
            kinematics.move(vRobot);
          } else
          { //R or L

            //Serial.println ("2.) -"+ pathPlanner.arcDirName12+ "- with angle: "+ degrees (pathPlanner.arcAngle12)+ "°, around center: "+ pathPlanner.arcCenter12.x + ", " + pathPlanner.arcCenter12.y);

            //Serial.println("current robot angle:  "+ String(degrees (curPose.angle))+ " °");

            ICC = pathPlanner.arcCenter12; //OR ? in theory the same: Vector(curPose.x - R * sin(curPose.angle), curPose.y + R* cos(curPose.angle) );
            R = pathPlanner.minRadius;//OR ? in theory the same:  distance(Vector (curPose.x, curPose.y), ICC);
            
            vRobotAng = default_speed_Ang;

            if (pathPlanner.arcDirName12.equals ("L")) {
                kinematics.move(vRobotAng, -R); //negative when left turn = counterclockwise
                targetAngle = pathPlanner.arcAngle12 + curPose.angle ;
            } else if (pathPlanner.arcDirName12.equals ("R")) {
                kinematics.move(vRobotAng, R); //positive when right turn = clockwise
                targetAngle = -pathPlanner.arcAngle12 + curPose.angle ;
            }

            R = kinematics.half_wheel_base *(vL+vR)/(vR - vL);
            vRobotAng = (vR - vL)/kinematics.wheel_base;
            ICC = Vector(curPose.x - R * sin(curPose.angle), curPose.y + R* cos(curPose.angle) );

            //Serial.println ("targetAngle: "+ String (degrees(targetAngle))+ " °");
          }

    };
    void MotionController::setWheelVelocitiesSeg3() //ARC left or right
    {

        //Serial.println ("Set wheel velocities segment 3(3) of path.");

        //Serial.println ("3.) -"+ pathPlanner.arcDirName2+ "- with angle: "+ degrees (pathPlanner.arcAngle2)+ "°, around center: "+ pathPlanner.arcCenter2.x + ", " + pathPlanner.arcCenter2.y);
        //Serial.println("current robot angle:  "+ String(degrees (curPose.angle))+ " °");

        ICC = pathPlanner.arcCenter2;//OR ? in theory the same: Vector(curPose.x - R * sin(curPose.angle), curPose.y + R* cos(curPose.angle) );
        R = pathPlanner.minRadius;//OR ? in theory the same:  distance(Vector (curPose.x, curPose.y), ICC);

        vRobotAng = default_speed_Ang;

        curDirName = pathPlanner.arcDirName2;

        if (pathPlanner.arcDirName2.equals ("L")) {
            kinematics.move(vRobotAng, -R); //negative when left turn = counterclockwise
            targetAngle = pathPlanner.arcAngle2 + curPose.angle ;
        } else if (pathPlanner.arcDirName2.equals ("R")){
            kinematics.move(vRobotAng, R); //positive when right turn = clockwise
            targetAngle = -pathPlanner.arcAngle2 + curPose.angle ;
        }

        //Serial.println ("targetAngle: " + String (degrees(targetAngle)) + " °");

    };

    void MotionController::stopMoving(){
        kinematics.stop();
    };

    void MotionController::enableMotors(){
        kinematics.enable();
    };


    void MotionController::setCurPose(Pose pose){
        curPose = pose;
    };


    void MotionController::loopPath()
    {
        curPathIndex++;
        if (curPathIndex >= path.size()) curPathIndex= 0; 

    };

    bool MotionController::checkIfArrived(){
        bool arrived = false;
        float dist = 1000000000;
        if (curDirName.equals("S")){
            Vector curV = Vector(curPose.x, curPose.y);
            Vector tarV = Vector (targetPose.x, targetPose.y);
            dist = distance( curV, tarV );
        }
        if (
          (curDirName.equals ("R") && targetAngle - curPose.angle >= arriveAngleDistance )
          ||
          (curDirName.equals ("L")  && targetAngle -  curPose.angle <= -arriveAngleDistance) 
          ||
          (curDirName.equals ("S")  && dist <= arriveDistance) )
          {
            arrived = true;
          }
        return arrived;
    };
    
}; // namespace SmallRobots