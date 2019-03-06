/*
 * Fixed: Drive Straight and pulsing with drive straight (should test more)
 * Need to fix ASAP: turn pulse method and turn
 * To Do: driveTo and complete token (PPT3)
 */

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
#define ARM_SERVO_MIN 1900
#define ARM_SERVO_MAX 2500
#define CLAW_SERVO_MIN 525
#define CLAW_SERVO_MAX 1230
#define PLATFORM_SERVO_MIN 563
#define PLATFORM_SERVO_MAX 2275
#define INITIAL_PLATFORM_ANGLE 165.0
#define INITIAL_ARM_ANGLE 0.0
#define INITIAL_CLAW_ANGLE 0.0
#define TIME_FOR_DIST dist/(112.19*abs(motor_percent)*3.5*PI/6000)
#define TIME_FOR_TURN ((8.58*PI/4)/(112.19*abs(motor_percent)*3.5*PI/6000)) * (angle/90.0)

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



void driveStraight(float motor_percent, float dist, float f[]){

    //Get initial x and y coordinates from RPS and store in initCoords
    f[0] = RPS.X();
    f[1] = RPS.Y();

    //Turn both motors on at given percent motor power.
    left_motor.SetPercent(motor_percent);
    right_motor.SetPercent(motor_percent);

    // Test to allow the robot to drive over the initial red light
    if(isStartOfRun){
        Sleep(.5);
        isStartOfRun = false;
    }

    //    //Time to wait before stopping the motors
    //    while(distance(initX, initY, RPS.X(), RPS.Y()) < dist - 1 || distance(initX, initY, RPS.X(), RPS.Y()) > dist + 1){

    //        if(!hasDoneDDR){
    //            lightColor = getLightColor();

    //            if(strcmp("None" , lightColor) != 0){
    //                left_motor.Stop();
    //                right_motor.Stop();
    //                hasDoneDDR = true;
    //            }
    //        }
    //    }

    //Less reliant on RPS

    float currTime = TimeNow();

    while(TimeNow() - currTime < TIME_FOR_DIST ){
        if(!hasDoneDDR){
            lightColor = getLightColor();

            if(strcmp("None" , lightColor) != 0){
                left_motor.Stop();
                right_motor.Stop();
                hasDoneDDR = true;
            }
        }
    }

    //Stop both motors.
    right_motor.Stop();
    left_motor.Stop();
}

void pulseDrive(float initCoord[], float dist,bool isForward){
    LCD.WriteLine(*initCoord);
    LCD.WriteLine(*(initCoord+1));
    while(distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) < dist - 1 || distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) > dist + 1){
        LCD.WriteLine(distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()));
        float temp[2];
        if(distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) < dist){
            driveStraight(30 * pow(-1,(int)!isForward),.5,temp);
        }else if(distance(*initCoord,*(initCoord+1), RPS.X(),RPS.Y()) > dist){
            driveStraight(-30 * pow(-1,(int)!isForward),.5,temp);
        }
        Sleep(.5);
    }
}

//void check_x_minus(float x_coordinate) //using RPS while robot is in the -x direction
//{
//    //check whether the robot is within an acceptable range
//    while(RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1)
//    {
//        if(RPS.X() > x_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction
//            driveStraight(30,.5);

//        }
//        else if(RPS.X() < x_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction

//            driveStraight(-30,.5);
//        }
//        Sleep(.5);
//    }
//}

//void check_x_plus(float x_coordinate) //using RPS while robot is in the +x direction
//{
//    //check whether the robot is within an acceptable range
//    while(RPS.X() < x_coordinate - 1 || RPS.X() > x_coordinate + 1)
//    {
//        if(RPS.X() > x_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction
//            driveStraight(-30,.5);


//        }
//        else if(RPS.X() < x_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction

//            driveStraight(30,.5);
//        }
//        Sleep(.5);
//    }
//}

//void check_y_minus(float y_coordinate) //using RPS while robot is in the -y direction
//{
//    //check whether the robot is within an acceptable range
//    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
//    {
//        if(RPS.Y() > y_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction

//            driveStraight(30,.25);
//        }
//        else if(RPS.Y() < y_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction

//            driveStraight(-30,.25);
//        }
//    }
//}

//void check_y_plus(float y_coordinate) //using RPS while robot is in the +y direction
//{
//    //check whether the robot is within an acceptable range
//    while(RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1)
//    {
//        if(RPS.Y() > y_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction

//           driveStraight(-30,.25);
//        }
//        else if(RPS.Y() < y_coordinate)
//        {
//            //pulse the motors for a short duration in the correct direction

//            driveStraight(30,.25);
//        }
//    }
//}




/*
 * Change drive method to match Eploration 3 since recursion isn't working
 */

void driveStraightWithTime(float motor_percent, float t){
    left_motor.SetPercent(motor_percent);

    right_motor.SetPercent(motor_percent);
    Sleep(t);
    right_motor.Stop();
    left_motor.Stop();
}

void turnWithTime(bool isLeft, bool isFront, float motor_percent, float angle, float t){
    left_motor.SetPercent(motor_percent * pow(-1,(int)isLeft) * pow(-1,(int)!isFront));

    right_motor.SetPercent(motor_percent * pow(-1,(int)!isLeft) * pow(-1,(int)!isFront));

    Sleep(t);
    right_motor.Stop();
    left_motor.Stop();
}


void turn(bool isLeft, float motor_percent, float angle, float initHeading[], float expHeading[]){

    //Turn both motors on at given percent motor power.

    left_motor.SetPercent(motor_percent * pow(-1,(int)isLeft));

    right_motor.SetPercent(motor_percent * pow(-1,(int)!isLeft));


    //Using RPS

    //Get current orientation

    initHeading[0] = RPS.Heading();

    //Move until expected orientation is reached

    //SD.Printf("\n\n\n");
    expHeading[0] = isLeft ? fmod((initHeading[0] + angle+360), (float)360.0) : fmod((initHeading[0] - angle+360), (float)360.0);

    float currTime = TimeNow();

    while(TimeNow() - currTime < TIME_FOR_TURN){}

    /*    //Calculate the point to which the robot needs to travel based on decision to turn
     *     Get initial position of robot QR code

        float intX = RPS.X();

        float intY = RPS.Y();

        if(RPS.Heading() % 360 >= 359 && RPS.Heading() % 360 <= 1 || RPS.Heading() % 360 >= 179 && RPS.Heading() % 360 <= 181){

             float dist = (distance(WIDTH*sin(angle * PI/180)*pow(-1,(int)!isFront) + intX ,

                                    intY - (WIDTH - WIDTH*cos(angle * PI/180))*pow(-1,(int)isLeft), RPS.X(), RPS.Y()));

        }else if(RPS.Heading() % 360 >= 89 && RPS.Heading() % 360 <= 91 || RPS.Heading() % 360 >= 269 && RPS.Heading() % 360 <= 271){

               float dist = (distance(intY - (5 - 5*cos(angle * PI/180))*pow(-1,(int)isLeft),

                                      5*sin(angle * PI/180)*pow(-1,(int)!isFront) + intX, RPS.X(), RPS.Y()));

        }

         //Horizontal and vertical will be determined somehow based on parallel to certain side

         while(!(dist >= 0 && dist <= .005));*/

    //Stop both motors.

    right_motor.Stop();
    left_motor.Stop();
}

void pulseTurn(float* initHeading, float* expHeading){
    float iH[1];float eH[1];
    if(*expHeading < 2 || *expHeading >= 358){
        if(RPS.Heading() < 358 && RPS.Heading() > 270){
            turn(true,30, abs((360*(*expHeading < 360 && *expHeading > 358) + RPS.Heading()) - *expHeading)/90,iH,eH);
        }else if(RPS.Heading() > 2  && RPS.Heading() < 90){
            turn(false,30, abs((360*(*expHeading < 2 && *expHeading > 0) + RPS.Heading()) - *expHeading)/90,iH,eH);
        }
        Sleep(.5);
    }else{
        while (RPS.Heading() > *expHeading + 2 || RPS.Heading() < *expHeading - 2)
        {

            if (RPS.Heading() > *expHeading)
            {

                turn(false,30,1,iH,eH);
            }

            else if(RPS.Heading() < *expHeading)
            {
                turn(true,30,1,iH,eH);
            }
            Sleep(.5);
        }
    }
}

//void driveTo(Task t){

//    platform_servo.SetDegree(90);

//    Sleep(.5);

//    arm_servo.SetDegree(180);

//    switch(t){

//    case TOKEN:

//        break;

//    case DDR:
//        driveStraight(30,4.8);
//        //driveStraightWithTime(30,.9);
//        turn(false,30,95);
//        //turnWithTime(false,true,30,95,2);
//        driveStraight(30,6);
//        //driveStraightWithTime(30,2);
//        break;

//    case FOOSBALL:{
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
//    }
//        break;

//    case LEVER:

//        break;

//    case BUTTON:

//        break;

//    default:

//        LCD.WriteLine("This message should not appear. You goofed.");

//    }

//}

//void complete(Task t){

//    switch(t){

//    case TOKEN:

//        break;

//    case DDR:
//    {
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
//    }
//        break;

//    case FOOSBALL:

//        break;

//    case LEVER:

//        break;

//    case BUTTON:

//        break;

//    default:

//        LCD.WriteLine("This message should not appear. You goofed.");

//    }

//}

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
            float ih[1], eh[1];
            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,ih,eh);

            LCD.WriteLine("Turn");
            pulseTurn(ih,eh);
            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,ih,eh);

            LCD.WriteLine("Turn");
            pulseTurn(ih,eh);
            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,ih,eh);

            LCD.WriteLine("Turn");
            pulseTurn(ih,eh);
            LCD.WriteLine("Pulsed");

            LCD.WriteLine(RPS.Heading());
            LCD.WriteLine("");
            turn(true,30,90,ih,eh);

            LCD.WriteLine("Turn");
            pulseTurn(ih,eh);
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
    SD.OpenLog();

    LCD.Clear(BLACK);

    /*
     * Initialize the RPS
     */

    RPS.InitializeTouchMenu();

    //    /*
    //     * Initialize robot
    //     */


    initialState();

    /*
     * Method to test various features of the robot
     */

    testMethods();
    //claw_servo.TouchCalibrate();

    /*
     * Reading CdS_Cell values
     */

    //               while(true){

    //                   LCD.WriteLine(CdS_Cell.Value());

    //                   Sleep(1.0);

    //               }

    //    int currTime = TimeNow();

    //    while(strcmp("Red",getLightColor()) != 0 && TimeNow() - currTime < 10);

    //    driveTo(DDR);
    //    Sleep(50);
    //    complete(DDR);
    //driveTo(FOOSBALL);
    SD.CloseLog();

}
