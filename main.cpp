#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <math.h>
#include <string.h>
#define PI 3.141592653
#define DISTANCE_TOLERANCE .35
#define ANGLE_TOLERANCE 1.5
#define ARM_SERVO_MIN 1620
#define ARM_SERVO_MAX 2450
#define CLAW_SERVO_MIN 560
#define CLAW_SERVO_MAX 1950
#define PLATFORM_SERVO_MIN 560
#define PLATFORM_SERVO_MAX 2500
#define INITIAL_PLATFORM_ANGLE 170.0
#define INITIAL_ARM_ANGLE 115.0
#define INITIAL_CLAW_ANGLE 0.0
#define DRIVING_STATE platform_servo.SetDegree(180);Sleep(.5);arm_servo.SetDegree(35);Sleep(.5);claw_servo.SetDegree(0);
#define MOTOR_PERCENT_ON_PULSE 30
#define MOTOR_PERCENT_ON_LEVEL 30
#define MOTOR_PERCENT_ON_INCLINE 45
#define CW false
#define CCW true

using namespace std;

/**
* Declaring and initializing analog input pin for cds cell.
*/

AnalogInputPin CdS_Cell(FEHIO::P0_0);

/**
* Declaring and initializing the two motors in the front of the robot.
*/

FEHMotor left_motor (FEHMotor::Motor0, 9.0);

FEHMotor right_motor (FEHMotor::Motor1, 9.0);

/**
* Servo motors for the rotating platform mount, the arm, and the claw.
*/

FEHServo platform_servo(FEHServo::Servo7);

FEHServo arm_servo(FEHServo::Servo6);

FEHServo claw_servo(FEHServo::Servo5);

/**
* Enumerations of task.
*/

enum Task{TOKEN,DDR,FOOSBALL,LEVER,BUTTON};

/**
* Boolean to determine if it is the start of the run
*/
bool isStartOfRun = true;

/**
* Boolean to determine if light has been detected for DDR and an integer to store the color
*/

bool hasDoneDDR = false;
int color = -1;

/**
* Correction factors for RPS
*/

float xFactor;
float yFactor;

/**
* Correction factor for ambient light differences
*/
float redThreshold = .25;
float blueThreshold = .85;

/**
* @brief distance
*              helper function to calculate distance between two points
* @param x
*              x-coordinate of first point
* @param y
*              y-coordinate of first point
* @param z
*              x-coordinate of second point
* @param w
*              y-coordinate of second point
* @return
*              distance between the points (x,y) and (z,w)
*/
float distance(float x, float y, float z, float w){

    return pow(pow(z - x,2) + pow(w - y,2),.5);
}

/**
* @brief getLightColor
*                  helper function to determine light color
* @return
*          integer value corresponding to a color (Red -> 0, Blue ->1, and No light -> -1)
*/
int getLightColor(){

    if (CdS_Cell.Value() <= redThreshold + .5){

        return 0;

    }else if(CdS_Cell.Value() > redThreshold + .5 &&CdS_Cell.Value() < blueThreshold + .35){

        return 1;
    }

    return -1;
}

/**
* @brief turn
*              General function for the robot's turning.
* @param isLeft
*              boolean parameter to dictate if the robot turn's left or not
* @param motor_percent
*              the motor power percentage
* @param angle
*              the amount the robot needs to turn in degrees
* @param expHeading
*              an array which stores the robot's expected heading
*/
void turn(bool isLeft, float motor_percent, float angle, float expHeading[]){

    //Get current orientation and determine the expected heading

    float initHeading = RPS.Heading();

    expHeading[0] = isLeft ? fmod((initHeading + angle+360), (float)360.0) : fmod((initHeading - angle+360), (float)360.0);

    //Have the wheels move in opposite directions for a calculated amount of time

    left_motor.SetPercent(motor_percent * pow(-1,(int)isLeft));

    right_motor.SetPercent(motor_percent * pow(-1,(int)!isLeft));

    float currTime = TimeNow();

    while(TimeNow() - currTime < abs(((8.58*PI/4)/(112.19*(motor_percent/100.)*3.5*PI/60)) * (angle/90.0))){
        SD.Printf("Time: %f\n\t%f\n",TimeNow(), CdS_Cell.Value());
    }

    //Stop both motors.

    right_motor.Stop();
    left_motor.Stop();
}

/**
* @brief pulseTurn
*              function pulses the robot to the correct heading if it is not within a certain thresholds
* @param expHeading
*              pointer to a float where the expected heading is stored
*/

void pulseTurn(float* expHeading){
    if(RPS.Heading() >= 0){
        //Counters for the number of pulse in a (counter)clockwise direction
        int numPulsesCW = 0;
        int numPulsesCCW = 0;
        float angleOff;
        float currTime = TimeNow();

        //Temporary float array for the calls to the turn method
        float temp[1];

        //Consider the case where the expected heading is close to 0 degrees separately
        if(*expHeading <= ANGLE_TOLERANCE || *expHeading >= 360 - ANGLE_TOLERANCE){
            while ((RPS.Heading() > *expHeading + 1 || RPS.Heading() < *expHeading - 1) && !(numPulsesCCW > 2 && numPulsesCW > 2)){
                //Depending on which side of 0 degrees the robot is facing, turn in a certain direction
                if(RPS.Heading() < 360 - ANGLE_TOLERANCE && RPS.Heading() > 270){
                    angleOff = abs((360*(*expHeading <= ANGLE_TOLERANCE && *expHeading >= 0) + *expHeading) - RPS.Heading());
                    turn(CCW,MOTOR_PERCENT_ON_PULSE, angleOff/2 + ANGLE_TOLERANCE,temp);
                    numPulsesCCW++;
                }else if(RPS.Heading() > ANGLE_TOLERANCE  && RPS.Heading() < 90){
                    angleOff = abs((360*(*expHeading >=0 && *expHeading >= 360 - ANGLE_TOLERANCE) + RPS.Heading()) - *expHeading);
                    turn(CW,MOTOR_PERCENT_ON_PULSE, angleOff/2 + ANGLE_TOLERANCE,temp);
                    numPulsesCW++;
                }
                if(TimeNow() - currTime >= 2){
                    break;
                }
                Sleep(.4);

            }

        }else{

            //Loop until the robot is within a certain threshold or if the robot pulses back and forth more than twice
            while ((RPS.Heading() > *expHeading + ANGLE_TOLERANCE || RPS.Heading() < *expHeading - ANGLE_TOLERANCE) && !(numPulsesCCW > 2 && numPulsesCW > 2))
            {

                //Determine how much the turn was off by and assign the smaller of the two values to a variable
                angleOff = fmod(RPS.Heading() - *expHeading + 360.0,360.0) < abs(RPS.Heading() - *expHeading) ? fmod(RPS.Heading() - *expHeading + 360.0,360.0) : abs(RPS.Heading() - *expHeading);

                /* Determine which direction to turn in and pulse a fraction of the angleOff
               * The amount pulse will decrease for each pulse to reduce pulse count
               * Increment the counters*/

                if(fmod(RPS.Heading() +angleOff+360.0,360.0) < *expHeading + .1 && fmod(RPS.Heading() +angleOff+360.0,360.0) > *expHeading - .1){
                    turn(CCW,MOTOR_PERCENT_ON_PULSE, (angleOff/2.) + 1.9,temp);
                    numPulsesCCW++;
                }else{
                    turn(CW,MOTOR_PERCENT_ON_PULSE,(angleOff/2.) + 1.9,temp);
                    numPulsesCW++;
                }
                if(TimeNow() - currTime >= 2){
                    break;
                }
                Sleep(.4);

            }
        }
    }
}

/**
* @brief driveStraight
*              general function for the robot driving straight
* @param motor_percent
*              the motor percentage
* @param dist
*              distance robot needs to travel
* @param initCoord
*              array which stores the initial coordinates of the robot
*/
void driveStraight(float motor_percent, float dist, float initCoord[]){

    //Get initial x and y coordinates from RPS and store in initCoords
    initCoord[0] = RPS.X() - xFactor;
    initCoord[1] = RPS.Y() - yFactor;

    //Turn both motors on at given percent motor power.
    left_motor.SetPercent(motor_percent);
    right_motor.SetPercent(motor_percent);

    //Test to allow the robot to drive over the initial red light rather than stopping at the light
    if(isStartOfRun){
        SD.Printf("Time: %f\n\t%f\n",TimeNow(), CdS_Cell.Value());
        Sleep(.45);
        isStartOfRun = false;
    }

    //Get the current time and loop for a calculated amount of time
    float currTime = TimeNow();
    SD.Printf("%f: In loop",TimeNow());
    while(TimeNow() - currTime < abs(dist/(112.19*motor_percent*3.5*PI/6000))){
        SD.Printf("Time: %f\n\t%f\n",TimeNow(), CdS_Cell.Value());
        //If the DDR task isn't done, this will stop the robot once it detects a light
        if(!hasDoneDDR){
            color = getLightColor();
            if(color != -1){
                break;
            }
        }
    }
    SD.Printf("%f: Out of driving loop",TimeNow());
    LCD.WriteLine("Out of driving loop");
    //Stop both motors if it hasn't already
    right_motor.Stop();
    left_motor.Stop();
}

/**
* @brief pulseDrive
*              function pulses the robot to the correct distance traveled if it is not within a certain thresholds
* @param initCoord
*              the array that points to the coordinates of the robot before it started driving
* @param dist
*          distance robots was supposed to have traveled
* @param isForward
*          boolean which stores whether the robot was driving in the positive x or y-direction
*/
void pulseDrive(float initCoord[], float dist,bool isForward){
    if(RPS.X() >= 0){
        //Counter that keeps track of how many times the robot pulses forwards or backwards
        int numPulsesForward = 0;
        int numPulsesBack = 0;

        //Temporary array for driveStraight calls
        float temp[2];

        //Stores the how much the robot's distance traveled was off
        float distOff = dist - distance(initCoord[0],initCoord[1], RPS.X()-xFactor,RPS.Y()-yFactor);

        //Loop until the robot is within a certain threshold or if the robot pulses back and forth more than once
        while(abs(distOff) >  DISTANCE_TOLERANCE && !(numPulsesForward >= 2 && numPulsesBack >= 2) && !(numPulsesBack + numPulsesForward > 5)){
            LCD.WriteLine("In pulse method");
            /* Determine which direction to drive in and pulse a fraction of the distOff
           * The amount pulse will decrease for each pulse to reduce pulse count
           * Increment the counters*/

            if(distOff > 0){
                driveStraight(MOTOR_PERCENT_ON_PULSE * pow(-1,(int)!isForward),abs(distOff/2.) + (2*DISTANCE_TOLERANCE - .01),temp);
                numPulsesForward += 1*isForward;
                numPulsesBack += 1*!isForward;

            }else if(distOff < 0){
                driveStraight(-MOTOR_PERCENT_ON_PULSE * pow(-1,(int)!isForward),abs(distOff/2.) + (2*DISTANCE_TOLERANCE - .01), temp);
                numPulsesForward += 1*!isForward;
                numPulsesBack += 1*isForward;
            }
            Sleep(.4);
            //Stores the how much the robot's distance traveled was off
            distOff = dist - distance(initCoord[0],initCoord[1], RPS.X()-xFactor,RPS.Y()-yFactor);
        }
    }
}

void pulseDriveToCoordinate(float expCoord[]){

    int numPulsesForward = 0;
    int numPulsesBack = 0;

    //Temporary array for driveStraight calls
    float temp[2];

    float distOff = distance(expCoord[0],expCoord[1], RPS.X(),RPS.Y());

    float intendedHead[1] = {(atan((expCoord[1] -RPS.Y())/(expCoord[0] - RPS.X())) * 180.0/PI)};

    pulseTurn(intendedHead);

    //Loop until the robot is within a certain threshold or if the robot pulses back and forth more than once
    while(abs(distOff) > .5 && !(numPulsesForward >= 2 && numPulsesBack >= 2)){

        /* Determine which direction to drive in and pulse a fraction of the distOff
       * The amount pulse will decrease for each pulse to reduce pulse count
       * Increment the counters*/

        driveStraight(MOTOR_PERCENT_ON_PULSE * pow(-1,(int)(expCoord[1] < RPS.Y())),abs(distOff/2.) + .5,temp);

        if(expCoord[1] < RPS.Y()){
            numPulsesForward++;
        }else{
            numPulsesBack++;
        }

        Sleep(.4);

        //Stores the how much the robot's distance traveled was off
        distOff = distance(expCoord[0],expCoord[1], RPS.X(),RPS.Y());
    }
}

/*
* Navigate with time purely
*/

void driveStraightWithTime(float motor_percent, float t){
    left_motor.SetPercent(motor_percent);
    right_motor.SetPercent(motor_percent);

    Sleep(t);

    right_motor.Stop();
    left_motor.Stop();
}

void turnWithTime(bool isLeft, float motor_percent, float t){
    left_motor.SetPercent(motor_percent * pow(-1,(int)isLeft));
    right_motor.SetPercent(motor_percent * pow(-1,(int)!isLeft));

    Sleep(t);

    right_motor.Stop();
    left_motor.Stop();
}


/**
* @brief driveTo
*          contains the directions that the robot needs to follow to drive to a certain task location
* @param task
*          an element of the Task enumerations
*/
void driveTo(Task task){
    switch(task){

    case TOKEN:
    {
        //Initialize arrays to store values in drive and turn methods
        float initCoord[2];
        float expHeading[1];

        //Lower the arm
        arm_servo.SetDegree(80);
        Sleep(500);

        //Drive straight to get in front of token task and pulse
        driveStraight(MOTOR_PERCENT_ON_LEVEL*2,2,initCoord);
        pulseDrive(initCoord,7.5,true);

        //Turn towards the token task
        turn(CCW,MOTOR_PERCENT_ON_LEVEL,90 - RPS.Heading(),expHeading);
        pulseTurn(expHeading);

        //Drive to the token task
        driveStraight(MOTOR_PERCENT_ON_LEVEL*2,7.75,initCoord);
        pulseDrive(initCoord,9.75,true);

    }
        break;

    case DDR:
    {
        //Initialize arrays to store values in drive and turn methods
        float initCoord[2];
        float expHeading[1];

        //Pulse so that the robot is straight when it drives
        expHeading[0] = 90.0;
        pulseTurn(expHeading);

        //Drive backwards to get next to the DDR task and pulse
        float y = RPS.Y();
        driveStraight(-MOTOR_PERCENT_ON_LEVEL*2,/*14.25*/ y - yFactor - 1,initCoord);
        pulseDrive(initCoord,/*13.75*/y - yFactor - .6,false);

        //Turn towards the DDR task and pulse
        turn(CW,MOTOR_PERCENT_ON_LEVEL,RPS.Heading() + 15,expHeading);
        expHeading[0] = 0;
        pulseTurn(expHeading);

        //Drive to the first DDR light
        driveStraight(MOTOR_PERCENT_ON_LEVEL*1.25,/*23*/13.25 - RPS.X() + xFactor,initCoord);
    }
        break;

    case FOOSBALL:{
        //Initialize arrays to store values in drive and turn methods
        float initCoord[2];
        float expHeading[1];

        //Check the color detected by the DDR
        if(color == 0|| color == -1){

            //Reverse actions to press red buttton
            driveStraight(-MOTOR_PERCENT_ON_LEVEL*.75,1.75,initCoord);
            driveStraightWithTime(-MOTOR_PERCENT_ON_LEVEL*.75, .35);
            turn(CCW,MOTOR_PERCENT_ON_LEVEL,33.5,expHeading);
            driveStraight(-MOTOR_PERCENT_ON_LEVEL,5,initCoord);
        }else if(color == 1){

            //Reverse actions to press blue button
            driveStraight(-MOTOR_PERCENT_ON_LEVEL*.75,.5,initCoord);
            driveStraightWithTime(-MOTOR_PERCENT_ON_LEVEL*.75, .55);
            turn(CCW,MOTOR_PERCENT_ON_LEVEL,35,expHeading);
            driveStraight(-MOTOR_PERCENT_ON_LEVEL*1.25,8.75,initCoord);
        }


        //Turn towards the ramp
        turn(CCW,MOTOR_PERCENT_ON_LEVEL,45,expHeading);
        expHeading[0] = 45.0;
        pulseTurn(expHeading);

        //Drive forward til the y-coordinate reaches a certain value and pulse
        float y = RPS.Y();
        driveStraight(MOTOR_PERCENT_ON_LEVEL*1.5,3,initCoord);
        pulseDrive(initCoord, 1.5 + ((/*12*/3 - y + yFactor)/sin(RPS.Heading() * PI/180.)),true);

        //Turn to get perpendicular to the ramp and pulse
        turn(CW,MOTOR_PERCENT_ON_LEVEL, 60,expHeading);
        expHeading[0] = 0;
        pulseTurn(expHeading);

        //Drive to get in front of the ramp and turn to face the ramp
        driveStraight(MOTOR_PERCENT_ON_LEVEL*1.5,.75*(/*27.75*/ 18.5-RPS.X()+xFactor),initCoord);
        pulseDrive(initCoord,23.5 - RPS.X() + xFactor,true);


        turn(CCW,MOTOR_PERCENT_ON_LEVEL,90,expHeading);
        expHeading[0] = 85.0;
        pulseTurn(expHeading);


        //Drive up the top of the ramp
        driveStraight(MOTOR_PERCENT_ON_INCLINE*1.5,43.5,initCoord);

        turn(CCW,MOTOR_PERCENT_ON_LEVEL,25,expHeading);

        //Drive straight and turn back to 90 degrees to drive towards foosball
        driveStraight(MOTOR_PERCENT_ON_LEVEL,3.5,initCoord);
        turn(CW,MOTOR_PERCENT_ON_LEVEL,25,expHeading);
    }
        break;

    case LEVER:
    {
        //Initialize arrays to store values in drive and turn methods
        float initCoord[2];
        float expHeading[1];

        //Drive away from foosball and turn towards the right side of the obstactle
        driveStraight(-MOTOR_PERCENT_ON_LEVEL,3.15,initCoord);
        turn(CCW,MOTOR_PERCENT_ON_LEVEL,70,expHeading);

        //Get arm ready for knocking down the obstactle and drive towards the obstacle
        //and turn to get on the right side of the obstacle

        arm_servo.SetDegree(100);
        Sleep(.50);
        driveStraight(MOTOR_PERCENT_ON_LEVEL,7.5,initCoord);
        turn(CCW,MOTOR_PERCENT_ON_LEVEL,30,expHeading);

        //Knock down the obstacle
        arm_servo.SetDegree(80);
        Sleep(.50);
        platform_servo.SetDegree(50);
        Sleep(.5);
        arm_servo.SetDegree(30);
        Sleep(.5);
        driveStraight(MOTOR_PERCENT_ON_LEVEL,1,initCoord);
        platform_servo.SetDegree(130);
        Sleep(.5);

        DRIVING_STATE;

        //Drive to the lever
        driveStraight(MOTOR_PERCENT_ON_LEVEL,4.25,initCoord);

    }
        break;

    case BUTTON:
    {
        //Initialize arrays to store values in drive and turn methods
        float initCoord[2];
        float expHeading[1];
        //driveStraight(-MOTOR_PERCENT_ON_LEVEL,1.5,initCoord);

        //Perform a three point turn to face the final button
        turn(CCW,MOTOR_PERCENT_ON_LEVEL,40,expHeading);

        driveStraight(MOTOR_PERCENT_ON_LEVEL,3.5,initCoord);
        turn(CCW,MOTOR_PERCENT_ON_LEVEL,50,expHeading);

        driveStraight(MOTOR_PERCENT_ON_LEVEL,12,initCoord);

        if(RPS.Heading() > 0){
            turn(CCW,MOTOR_PERCENT_ON_LEVEL,abs(270 - RPS.Heading()),expHeading);
            expHeading[0] = 270.0;
            pulseTurn(expHeading);
        }else{
            turn(CCW,MOTOR_PERCENT_ON_LEVEL,20,expHeading);
        }
        while(RPS.Heading() < 0){
            driveStraight(MOTOR_PERCENT_ON_LEVEL,1,initCoord);
        }
        Sleep(.35);

        //Perform RPS check before going down the ramp
        if(RPS.Heading() > 0){
            expHeading[0] = 270.0;
            pulseTurn(expHeading);
        }

        //Drive to final button and face the button
        driveStraight(MOTOR_PERCENT_ON_LEVEL*1.5,40,initCoord);
        turn(CW,MOTOR_PERCENT_ON_LEVEL,45,expHeading);

    }

        break;

    default:

        LCD.WriteLine("This message should not appear. You goofed.");

    }
}

/**
* @brief complete
*          functions which tells the robot what to do to complete the task
* @param t
*          an element of the Task enumeration
*/
void complete(Task t){

    switch(t){

    case TOKEN:
        //Move the arm above the slot and open claw to drop the coin
        platform_servo.SetDegree(52);
        Sleep(500);
        claw_servo.SetDegree(110);
        Sleep(500);
        claw_servo.SetDegree(0);

        break;

    case DDR:
    {
        //Initialize arrays to store values in drive and turn methods
        float initCoord[2];
        float expHeading[1];

        //Get color of light twice to ensure that the color is right
        color = getLightColor();
        driveStraightWithTime(20,.1);
        color = getLightColor();

        //Do an RPS check to make sure the robot is parallel to side of the course
        expHeading[0] = 0.0;
        pulseTurn(expHeading);

        //Press the button based on color detected
        if(color != 1){
            if(color == 0){
                LCD.Clear(RED);
            }else{
                LCD.Clear(BLACK);
                LCD.WriteLine("CdS cell didn't read anything. It goofed.");
            }

            driveStraightWithTime(-20,.2);

            turn(CW,MOTOR_PERCENT_ON_LEVEL,34,expHeading);
            driveStraightWithTime(MOTOR_PERCENT_ON_LEVEL*.75, .75);
            driveStraight(MOTOR_PERCENT_ON_LEVEL*.75,4.,initCoord);

            Sleep(5.0);

        }else {
            LCD.Clear(BLUE);

            driveStraightWithTime(MOTOR_PERCENT_ON_LEVEL,.3);
            driveStraight(MOTOR_PERCENT_ON_LEVEL,1.85,initCoord);
            turn(CW,MOTOR_PERCENT_ON_LEVEL,35,expHeading);
            driveStraightWithTime(MOTOR_PERCENT_ON_LEVEL*.75, .5);
            driveStraight(MOTOR_PERCENT_ON_LEVEL*.75,3,initCoord);

            Sleep(5.0);
        }
        hasDoneDDR = true;

    }
        break;

    case FOOSBALL:
    {
        float initCoord[2];
        float expHeading[1];
        //Get the foosball scoring disk in the grasp of the robot's claw
        arm_servo.SetDegree(60);
        Sleep(250);
        platform_servo.SetDegree(75);
        Sleep(500);
        arm_servo.SetDegree(0);
        Sleep(500);
        claw_servo.SetDegree(100);
        Sleep(500);
        if(color != 1){
            driveStraight(MOTOR_PERCENT_ON_LEVEL,.75,initCoord);
        }else{
            driveStraight(MOTOR_PERCENT_ON_LEVEL,.75,initCoord);
        }
        Sleep(500);
        platform_servo.SetDegree(30);
        Sleep(500);
        claw_servo.SetDegree(30);
        Sleep(500);

        //Back the robot up a bit so the arm can extend and then have the arm complete the task

        for(int i = 4; i < 9;i++){
            platform_servo.SetDegree(10+10*i);

        }
        Sleep(250);
        driveStraight(-MOTOR_PERCENT_ON_LEVEL,.5,initCoord);

        claw_servo.SetDegree(90);
        driveStraight(-MOTOR_PERCENT_ON_LEVEL,1,initCoord);
        arm_servo.SetDegree(35);
        Sleep(500);
    }
        break;

    case LEVER:
    {
        float initCoord[2];
        arm_servo.SetDegree(60);
        Sleep(.5);
        platform_servo.SetDegree(10);
        Sleep(.5);
        driveStraight(MOTOR_PERCENT_ON_LEVEL,2.5,initCoord);
        arm_servo.SetDegree(15);
        Sleep(.5);
        arm_servo.SetDegree(60);
        for(int i = 3; i < 8;i++){
            platform_servo.SetDegree(5*i);
            Sleep(.5);
            arm_servo.SetDegree(30);
            Sleep(.5);
            arm_servo.SetDegree(50);
            Sleep(.5);
            platform_servo.SetDegree(15);
            Sleep(.5);
            arm_servo.SetDegree(60);
            Sleep(.5);
        }
    }
        break;

    case BUTTON:
    {
        float initCoord[2];
        driveStraight(MOTOR_PERCENT_ON_LEVEL*1.5,5,initCoord);
        LCD.WriteLine("CONGRATS!!! The robot reached the end of the course.");
    }
        break;

    default:

        LCD.WriteLine("This message should not appear. You goofed.");

    }
    Sleep(500);
    DRIVING_STATE
}


/**
* @brief initialRobotState
*                  initialize the robot so that it meets the requirements of the starting position
*/
void initialRobotState(){

    /*
   * Set up servos and have them in starting state
   */

    arm_servo.SetMin(ARM_SERVO_MIN);
    arm_servo.SetMax(ARM_SERVO_MAX);

    platform_servo.SetMin(PLATFORM_SERVO_MIN);
    platform_servo.SetMax(PLATFORM_SERVO_MAX);


    claw_servo.SetMin(CLAW_SERVO_MIN);
    claw_servo.SetMax(CLAW_SERVO_MAX);

    arm_servo.SetDegree(INITIAL_ARM_ANGLE);
    Sleep(.5);
    claw_servo.SetDegree(INITIAL_CLAW_ANGLE);
    Sleep(.5);
    platform_servo.SetDegree(INITIAL_PLATFORM_ANGLE);
}



/**
* @brief finalAction
*             start menu that will act as final action
*/
void finalAction(){

    LCD.Clear(BLACK);

    FEHIcon::Icon start_button;

    start_button.SetProperties("READY", 120, 95, 80, 50, WHITE, WHITE);

    start_button.Draw();


    float xTouch, yTouch;

    LCD.Touch(&xTouch,&yTouch);

    while(!start_button.Pressed(xTouch,yTouch,1)){
        LCD.Touch(&xTouch,&yTouch);

    }

    LCD.Clear(BLACK);
    LCD.WriteAt("Set",120, 95);

    /*
 * Set up correction factors
 */
    xFactor = RPS.X();
    yFactor = RPS.Y();
}

/*
* Testing methods
*/

void testingServo(FEHServo s){

    float deg = 0.00;

    int i = 0;

    s.SetDegree(deg);

    Sleep(1.0);

    while(deg <= 180.00){

        LCD.WriteAt(deg,175,10 + i*10);

        s.SetDegree(deg);

        deg+=10;

        Sleep(1.0);

    }

}

void testMethods(){

    LCD.Clear(BLACK);

    FEHIcon::Icon turn_button;

    turn_button.SetProperties("TURN", 10, 20, 80, 50, WHITE, WHITE);

    turn_button.Draw();


    FEHIcon::Icon arm_button;

    arm_button.SetProperties("ARM", 10, 80, 80, 50, WHITE, WHITE);

    arm_button.Draw();


    FEHIcon::Icon claw_button;

    claw_button.SetProperties("CLAW", 10, 140, 80, 50, WHITE, WHITE);

    claw_button.Draw();


    FEHIcon::Icon platform_button;

    platform_button.SetProperties("PLATFORM", 10, 200, 80, 50, WHITE, WHITE);

    platform_button.Draw();


    FEHIcon::Icon drive_button;

    drive_button.SetProperties("DRIVE", 100, 20, 80, 50, WHITE, WHITE);

    drive_button.Draw();


    while(true){

        float x, y;

        LCD.Touch(&x,&y);

        if(turn_button.Pressed(x,y,0)){
            LCD.Clear(BLACK);
            float eh[1];
            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(CCW,30,90,eh);
            Sleep(.5);
            //            LCD.WriteLine("Turn");
            //            pulseTurn(eh);
            //            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(CCW,30,90,eh);
            Sleep(.5);

            //            LCD.WriteLine("Turn");
            //            pulseTurn(eh);
            //            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(CCW,30,90,eh);
            Sleep(.5);

            //            LCD.WriteLine("Turn");
            //            pulseTurn(eh);
            //            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(CCW,30,90,eh);

            //            LCD.WriteLine("Turn");
            //            pulseTurn(eh);
            //            LCD.WriteLine("Pulsed");

        }else if (arm_button.Pressed(x,y,0)) {

            testingServo(arm_servo);

        }else if (claw_button.Pressed(x,y,0)) {

            testingServo(claw_servo);

        }else if (platform_button.Pressed(x,y,0)) {

            testingServo(platform_servo);

        }else if (drive_button.Pressed(x,y,0)){
            LCD.Clear(BLACK);
            float initCoord[2];
            driveStraight(30,7.5,initCoord);
            LCD.WriteLine("Drove forward");
            pulseDrive(initCoord,7.5,true);
            LCD.WriteLine("Pulsed");
            Sleep(1.0);
            driveStraight(-30,7.5,initCoord);
            LCD.WriteLine("Drove backward");
            pulseDrive(initCoord,7.5,false);
            LCD.WriteLine("Pulsed");
        }

    }

}

/*
* Main function
*/

int main(void){

    //Initialize robot

    initialRobotState();

    //Initialize RPS and open SD log

    RPS.InitializeTouchMenu();
    SD.OpenLog();

    //Perform final action

    finalAction();

    float currTime = TimeNow();
    while(getLightColor() != 0 || TimeNow() - currTime < 30);

    //Starting message
    LCD.Clear(BLACK);
    LCD.WriteAt("Go",120, 95);

    //Drive to and complete token
    driveTo(TOKEN);
    complete(TOKEN);

    //Drive to and complete DDR
    driveTo(DDR);
    complete(DDR);

    //Drive to and complete foosball
    driveTo(FOOSBALL);
    complete(FOOSBALL);

    //Drive to and complete lever
    driveTo(LEVER);
    complete(LEVER);

    //Drive to and complete final button
    driveTo(BUTTON);
    complete(BUTTON);

    //Close SD log
    SD.CloseLog();
}
