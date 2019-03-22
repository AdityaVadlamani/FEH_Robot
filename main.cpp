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
#define WIDTH 8.76
#define HEIGHT 3.74
#define DEPTH 8.4
#define DISTANCE_TOLERANCE .05
#define ANGLE_TOLERANCE .01
#define ARM_SERVO_MIN 1680
#define ARM_SERVO_MAX 2450
#define CLAW_SERVO_MIN 560
#define CLAW_SERVO_MAX 1950
#define PLATFORM_SERVO_MIN 560
#define PLATFORM_SERVO_MAX 2500
#define INITIAL_PLATFORM_ANGLE 170.0
#define INITIAL_ARM_ANGLE 60.0
#define INITIAL_CLAW_ANGLE 0.0
#define TIME_FOR_DIST abs(dist)/(112.19*abs(motor_percent)*3.5*PI/6000)
#define TIME_FOR_TURN 1.5*((8.58*PI/4)/(112.19*abs(motor_percent)*3.5*PI/6000)) * (abs(angle)/90.0)
#define DRIVING_STATE arm_servo.SetDegree(15);Sleep(.5);platform_servo.SetDegree(170);

using namespace std;

/*
 * Declaring and initializing analog input pins for cds cell
 */

AnalogInputPin CdS_Cell(FEHIO::P0_0);

/*
 * Declaring and initializing the two motors in the front of the robot
 */

FEHMotor left_motor (FEHMotor::Motor0, 9.0);

FEHMotor right_motor (FEHMotor::Motor1, 9.0);

/*
 * Servo motor for the rotating platform holding the ar
 */

FEHServo platform_servo(FEHServo::Servo7);

/*
 * Servo motor for the bending of the arm
 */

FEHServo arm_servo(FEHServo::Servo6);

/*
 * Servo motor for the bending of the claw
 */

FEHServo claw_servo(FEHServo::Servo5);

/*
 * Enumerations of task
 */

enum Task{TOKEN,DDR,FOOSBALL,LEVER,BUTTON};


/*
 * Boolean to determine if it is start of run and if light has been detected.
 */
bool hasDoneDDR = false;
bool isStartOfRun = true;

/*
 * Helper function for distance
 */

float distance(float x, float y, float z, float w){

    return pow(pow(z - x,2) + pow(w - y,2),.5);
}

/*
 * Helper function to determine light color and global variable to store it in
 */
char* lightColor = "None";
char* const getLightColor(){

    if (CdS_Cell.Value() <= .3){

        return "Red";

    }else if(CdS_Cell.Value() > .3 && CdS_Cell.Value() <= .5){

        return "Blue";
    }

    return "None";

}


void turn(bool isLeft, float motor_percent, float angle, float expHeading[]){

    //Turn both motors on at given percent motor power.

    left_motor.SetPercent(motor_percent * pow(-1,(int)isLeft));

    right_motor.SetPercent(motor_percent * pow(-1,(int)!isLeft));

    //Get current orientation

    float initHeading = RPS.Heading();

    //Move until expected orientation is reached

    expHeading[0] = isLeft ? fmod((initHeading + angle+360), (float)360.0) : fmod((initHeading - angle+360), (float)360.0);

    float currTime = TimeNow();

    while(TimeNow() - currTime < TIME_FOR_TURN){}

    //Stop both motors.

    right_motor.Stop();
    left_motor.Stop();
}

void pulseTurn(float* expHeading){
    int numPulsesCW = 0;
    int numPulsesCCW = 0;
    float eH[1];
    if(*expHeading < 1.5 || *expHeading >= 358.5){
        if(RPS.Heading() < 358.5 && RPS.Heading() > 270){
            turn(true,30, abs((360*(*expHeading < 1.5 && *expHeading > 0) + *expHeading) - RPS.Heading())/90,eH);
        }else if(RPS.Heading() > 1.5  && RPS.Heading() < 90){
            turn(false,30, abs((360*(*expHeading < 360 && *expHeading >358.5) + RPS.Heading()) - *expHeading)/90,eH);
        }
    }else{
        float angleOff = fmod(RPS.Heading() - *expHeading + 360.0,360.0) < abs(RPS.Heading() - *expHeading) ? fmod(RPS.Heading() - *expHeading + 360.0,360.0):abs(RPS.Heading() - *expHeading);

        int k = 1;
        while ((RPS.Heading() > *expHeading + 1.5 || RPS.Heading() < *expHeading - 1.5) && !(numPulsesCCW > 2 && numPulsesCW > 2))
        {
            SD.Printf("%f\n",RPS.Heading() - *expHeading);

            if(fmod(RPS.Heading() +angleOff+360.0,360.0) != *expHeading){
                turn(false,30,angleOff/pow(1.5,k) + 2.25,eH);
                numPulsesCCW++;
            }else{
                turn(false,30,(angleOff/pow(1.5,k)) + 2.25,eH);
                numPulsesCW++;
            }
            Sleep(.4);
            k++;
        }
    }
}

//void keepStraight(float motor_percent, float* expHeading){
//    right_motor.SetPercent(motor_percent);
//    left_motor.SetPercent(motor_percent);
//    float eH[1];
//    if(*expHeading < 1.25 || *expHeading >= 358.75){
//        float angle = abs((360*(*expHeading < 1.25 && *expHeading > 0) + *expHeading) - RPS.Heading())/90;
//        motor_percent+=2.5;
//        if(RPS.Heading() < 358.5 && RPS.Heading() > 270){
//            right_motor.SetPercent(motor_percent);
//        }else if(RPS.Heading() > 1.5  && RPS.Heading() < 90){
//            left_motor.SetPercent(motor_percent);
//        }
//        Sleep(TIME_FOR_TURN);
//        motor_percent-=2.5;
//        Sleep(.5);

//    }else{
//        while (RPS.Heading() > *expHeading + 1.5 || RPS.Heading() < *expHeading - 1.5)
//        {
//            float angle = 3;
//            motor_percent+=2.5;
//            if (RPS.Heading() > *expHeading)
//            {
//                right_motor.SetPercent(motor_percent);
//            }

//            else if(RPS.Heading() < *expHeading)
//            {
//                left_motor.SetPercent(motor_percent);

//            }
//            Sleep(TIME_FOR_TURN);
//            motor_percent-=2.5;
//            Sleep(.5);
//        }
//    }
//}

void driveStraight(float motor_percent, float dist, float f[]){
    //do{

    //Get initial x and y coordinates from RPS and store in initCoords
    f[0] = RPS.X();
    f[1] = RPS.Y();
    //}while(RPS.X() < 0 || RPS.Y() < 0);

    //Turn both motors on at given percent motor power.


    float h[] = {RPS.Heading()};

    // Test to allow the robot to drive over the initial red light
    if(isStartOfRun){
        Sleep(.5);
        isStartOfRun = false;
    }

    //Less reliant on RPS

    float currTime = TimeNow();
    float timeToRun = TIME_FOR_DIST;


    //SD.Printf("Time for distance: %f\n",timeToRun);
    while(TimeNow() - currTime < timeToRun ){
        if(!hasDoneDDR){
            lightColor = getLightColor();

            if(strcmp("None" , lightColor) != 0){
                left_motor.Stop();
                right_motor.Stop();
                hasDoneDDR = true;
            }
        }
        //SD.Printf("Time Elapsed: %f\n", TimeNow() - currTime);
        //keepStraight(motor_percent, h);
        //        currTime = TimeNow();
        //        timeToRun -= elapsedTime;
        left_motor.SetPercent(motor_percent);
        right_motor.SetPercent(motor_percent);
    }
    //SD.Printf("\n\n\n");
    //Stop both motors.
    right_motor.Stop();
    left_motor.Stop();
}

void pulseDrive(float initCoord[], float dist,bool isForward){
    int numPulsesForward = 0;
    int numPulsesBack = 0;
    int k = 1;
    float distOff = (dist - distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()));
    while((distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) < dist - .5 || distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) > dist + .5) && !(numPulsesBack > 2 && numPulsesForward > 2)){
        float temp[2];
        LCD.WriteLine(dist - distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()));
        if(dist - distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) > 0){
            driveStraight(30 * pow(-1,(int)!isForward),distOff/pow(2,k) + .5,temp);
            numPulsesForward += 1*isForward;
            numPulsesBack += 1*!isForward;

        }else if(dist - distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) < 0){
            driveStraight(-30 * pow(-1,(int)!isForward),distOff/pow(2,k) + .5,temp);
            numPulsesForward += 1*!isForward;
            numPulsesBack += 1*isForward;
        }
        Sleep(.4);
        k++;
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


void driveTo(Task task){
    switch(task){

    case TOKEN:
    {
        float initCoord[2];
        float expHeading[2];
        driveStraight(30,7.25,initCoord);
        pulseDrive(initCoord,7.25,true);
        turn(true,30,45,expHeading);
        pulseTurn(expHeading);
        driveStraight(30,10.5,initCoord);
        pulseDrive(initCoord,10.5,true);

    }
        break;

    case DDR:

        //        driveStraight(30,4.8);
        //        //driveStraightWithTime(30,.9);
        //        turn(false,30,95);
        //        //turnWithTime(false,true,30,95,2);
        //        driveStraight(30,6);
        //        //driveStraightWithTime(30,2);
        break;

    case FOOSBALL:{
        //        char* c = getLightColor();
        //        if(strcmp("Red",c) == 0){
        //            driveStraightWithTime(-30,1.7);
        //            turnWithTime(false,false,30,45,1.25);
        //            driveStraightWithTime(-30,.25);
        //            turnWithTime(false,false,30,45,1.25);
        //            driveStraightWithTime(30,.5);
        //        }else{
        //            driveStraightWithTime(-30,.5);
        //            turnWithTime(false,false,30,45,1.25);
        //        }


        //        turnWithTime(true,true,30,80,2.5);

        //        driveStraightWithTime(35,7.25);
        //        turnWithTime(true,true,30,25,1.2);
        //        driveStraightWithTime(35,1.35);

        float initCoord[2];
        float expHeading[1];
        driveStraight(30,7,initCoord);
        pulseDrive(initCoord,7,true);
        turn(false,30,40,expHeading);
        expHeading[0] = 0;
        pulseTurn(expHeading);
        driveStraight(30,14,initCoord);
        pulseDrive(initCoord,14,true);
        LCD.WriteLine("About to turn onto ramp");

        //        platform_servo.SetDegree(22);
        //        Sleep(1.0);
        //        arm_servo.SetDegree(0);
        //        Sleep(4.0);
        //        DRIVING_STATE
        //                driveStraight(-30,6,initCoord);
        //        pulseDrive(initCoord,6,false);
        //        turn(true,30,30,expHeading);
        //        pulseTurn(expHeading);
        //        driveStraight(30,12,initCoord);
        //        pulseDrive(initCoord,12,true);
        turn(true,30,90,expHeading);
        expHeading[0] = 85.0;
        pulseTurn(expHeading);
        driveStraight(40,47.5,initCoord);
        turn(true,30,30,expHeading);
        driveStraight(30,5.5,initCoord);
        turn(false,30,30,expHeading);
        platform_servo.SetDegree(10);
        Sleep(500);
        arm_servo.SetDegree(5.0);
        Sleep(500);
        claw_servo.SetDegree(90);
        Sleep(500);
        driveStraight(30,2.25,initCoord);
        Sleep(500);
        claw_servo.SetDegree(30);
        Sleep(500);
        driveStraightWithTime(-30,.15);
        for(int i = 1; i < 8;i++){
            platform_servo.SetDegree(10+10*i);
            Sleep(500);
        }


    }
        break;

    case LEVER:
    {
        float eh[1];
        float ic[2];
        driveStraight(-30,12,ic);
        pulseDrive(ic,12,false);
        turn(false,30,95,eh);
        eh[0] = 0;
        pulseTurn(eh);
        driveStraight(30,13.5,ic);
        pulseDrive(ic,13.5,true);
        turn(true,30,90,eh);
        pulseTurn(eh);
        driveStraight(30,20,ic);
        turn(true,30,45,eh);
        //driveStraight(30,3,ic);

    }
        break;

    case BUTTON:

        break;

    default:

        LCD.WriteLine("This message should not appear. You goofed.");

    }

}

void complete(Task t){

    switch(t){

    case TOKEN:
        platform_servo.SetDegree(0);
        Sleep(.5);
        arm_servo.SetDegree(45);
        Sleep(.5);
        claw_servo.SetDegree(80);
        platform_servo.SetDegree(15);
        Sleep(1.0);
        platform_servo.SetDegree(0);
        Sleep(1.0);
        claw_servo.SetDegree(0);
        break;

    case DDR:
    {
        //        char* c = getLightColor();
        //        if(strcmp("Red",c) == 0){

        //            LCD.Clear(RED);
        //            //turn(false,false,30,50);
        //            turnWithTime(false,false,30,98,1.5);
        //            driveStraightWithTime(30,.4);
        //            turnWithTime(false,true,30,50,2.2);
        //            driveStraightWithTime(30,.5);


        //        }else if(strcmp("Blue",c) == 0){

        //            LCD.Clear(BLUE);

        //            turnWithTime(false,false,30,50,1.7);
        //            driveStraightWithTime(30,.98);
        //            turnWithTime(false,true,30,50,1.2);
        //            //driveStraight(30,.7);
        //            driveStraightWithTime(30,.5);

        //        }else{

        //            LCD.Clear(BLACK);

        //            LCD.WriteLine("CdS cell didn't read anything. It goofed.");
        //            turnWithTime(false,false,30,50,1.7);
        //            driveStraightWithTime(30,.95);
        //            turnWithTime(false,true,30,50,1.2);
        //            //driveStraight(30,.7);
        //            driveStraightWithTime(30,.5);
        //        }

        //        Sleep(5.0);

        //        //arm_servo.SetDegree(120);
    }
        break;

    case FOOSBALL:

        break;

    case LEVER:

        break;

    case BUTTON:

        break;

    default:

        LCD.WriteLine("This message should not appear. You goofed.");

    }

}

/*
 * Initializing robot state
 */

void initialState(){

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

    /*
     * Initial menu
     */

    LCD.Clear(BLACK);

    //    FEHIcon::Icon start_button;

    //    start_button.SetProperties("START", 150, 120, 80, 50, WHITE, WHITE);

    //    start_button.Draw();

    //    float xTouch, yTouch;

    //    LCD.Touch(&xTouch,&yTouch);

    //    while(0 == start_button.Pressed(xTouch,yTouch,0));

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
            //Perform all possible 90 degree turns with 1/2 second gaps in between each
            float eh[1];
            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,eh);

            LCD.WriteLine("Turn");
            pulseTurn(eh);
            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,eh);

            LCD.WriteLine("Turn");
            pulseTurn(eh);
            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,eh);

            LCD.WriteLine("Turn");
            pulseTurn(eh);
            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,eh);

            LCD.WriteLine("Turn");
            pulseTurn(eh);
            LCD.WriteLine("Pulsed");

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


    /*
     * Initialize the RPS
     */
    RPS.InitializeTouchMenu();
    SD.OpenLog();

    /*
     * Initialize robot
     */

    initialState();

    //Start robot

    int currTime = TimeNow();

    while(/*!strcmp("Red",getLightColor()) && */TimeNow() - currTime < 10);
    driveTo(FOOSBALL);

    //  testMethods();

    SD.CloseLog();

}
