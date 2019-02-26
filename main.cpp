#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>
#include <cstring>
#define PI 3.14159265
#define WIDTH 8.76
#define HEIGHT 3.74
#define DEPTH 8.4
#define TOLERANCE .005
#define ARM_SERVO_MIN 800
#define ARM_SERVO_MAX 1450
#define CLAW_SERVO_MIN 535
#define CLAW_SERVO_MAX 2460
#define PLATFORM_SERVO_MIN 563
#define PLATFORM_SERVO_MAX 2275
#define INITIAL_PLATFORM_ANGLE 165.0
#define INITIAL_ARM_ANGLE 90.0
#define INITIAL_CLAW_ANGLE 90.0
#define LOW_RIGHT_THRESHOLD 2.5
#define HIGH_RIGHT_THRESHOLD 2.5
#define LOW_LEFT_THRESHOLD 2.5
#define HIGH_LEFT_THRESHOLD 2.5
#define COUNT_PER_INCH 8/(3.5*PI)

using namespace std;

/*
 * Declaring and initializing analog input pins for cds cell and optosensors
 */

AnalogInputPin CdS_Cell(FEHIO::P0_0);

AnalogEncoder right_encoder(FEHIO::P1_0);
AnalogEncoder left_encoder(FEHIO::P1_7);

AnalogInputPin right_encoder_value(FEHIO::P1_0);
AnalogInputPin left_encoder_value(FEHIO::P1_7);

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
 * Thresholds form optosensors
 */

float RIGHT_THRESHOLD = 2.5;
float LEFT_THRESHOLD = 2.5;

/*
 * Enumerations of task
 */

enum Task{TOKEN,DDR,FOOSBALL,LEVER,BUTTON};


/*
 * Helper function for distance
 */

float distance(float x, float y, float z, float w){
    return pow(pow(z - x,2) + pow(w - y,2),.5);
}

/*
 * Helper function for tolerance
 */

bool withinTolerance(float exp, float act){
    return abs(act - exp) < TOLERANCE;
}

/*
 * Helper function to determine light color
 */

char* const colorLight(){
    if (CdS_Cell.Value() <= .2){
        LCD.Clear(RED);
        return "Red";
    }else if(CdS_Cell.Value() > .2 && CdS_Cell.Value() <= .3){
        LCD.Clear(BLUE);
        return "Blue";
    }
    return "None";
}

void driveStraight(float motor_percent, float dist){

    //Get initial position of robot QR code

    float intX = RPS.X();

    float intY = RPS.Y();

    //Turn both motors on at given percent motor power.

    left_motor.SetPercent(motor_percent);

    right_motor.SetPercent(motor_percent);


    //Using RPS

    //     //Time to wait before stopping the motors

    //     while(!withinTolerance(distance(intX, intY, RPS.X(), RPS.Y()) , dist));


    //    //Using encoders

    //    right_encoder.ResetCounts();

    //    left_encoder.ResetCounts();


    //    while(right_encoder.Counts() + left_encoder.Counts() < 2*COUNT_PER_INCH * dist){

    //        LCD.WriteLine("Left: ");

    //        LCD.WriteLine(left_encoder.Counts());

    //        LCD.WriteLine("Right: ");

    //        LCD.WriteLine(right_encoder.Counts());

    //        Sleep(.50);

    //    }

    //Stop both motors.

    right_motor.Stop();
    left_motor.Stop();
}



void turn(bool isLeft, bool isFront, float motor_percent, float angle){

    //Get initial position of robot QR code

    //    float intX = RPS.X();

    //    float intY = RPS.Y();



    //Turn both motors on at given percent motor power.



    left_motor.SetPercent(motor_percent * !isLeft * pow(-1,(int)!isFront));

    right_motor.SetPercent(motor_percent * isLeft * pow(-1,(int)!isFront));


    //    //Using encoders

    //    left_encoder.ResetCounts();

    //    right_encoder.ResetCounts();

    //    if(isLeft){


    //        while(right_encoder.Counts() <= (8*(WIDTH)/7.0)*(angle/90.)){

    //            LCD.WriteLine("Right: ");

    //            LCD.WriteLine(right_encoder_value.Value());

    //            Sleep(.50);

    //        }

    //    }else{



    //        while(left_encoder.Counts() <= (8*(WIDTH)/7.0)*(angle/90.)){

    //            LCD.WriteLine("Left: ");

    //            LCD.WriteLine(left_encoder_value.Value());

    //            Sleep(.50);

    //        }

    //    }

    //Using RPS



    //     //Get current orientation

    //     float currentHeading = RPS.Heading();



    //    //Move until expected orientation is reached

    //     if(isLeft){

    //         while(!withinTolerance(RPS.Heading(), fmod((currentHeading - angle), (float)360.0)));

    //     }else{

    //         while(!withinTolerance(RPS.Heading(), fmod((currentHeading + angle), (float)360.0)));

    //     }


    //    //Calculate the point to which the robot needs to travel based on decision to turn

    //    if(RPS.Heading() % 360 >= 359 && RPS.Heading() % 360 <= 1 || RPS.Heading() % 360 >= 179 && RPS.Heading() % 360 <= 181){

    //         float dist = (distance(WIDTH*sin(angle * PI/180)*pow(-1,(int)!isFront) + intX ,

    //                                intY - (WIDTH - WIDTH*cos(angle * PI/180))*pow(-1,(int)isLeft), RPS.X(), RPS.Y()));

    //    }else if(RPS.Heading() % 360 >= 89 && RPS.Heading() % 360 <= 91 || RPS.Heading() % 360 >= 269 && RPS.Heading() % 360 <= 271){

    //           float dist = (distance(intY - (5 - 5*cos(angle * PI/180))*pow(-1,(int)isLeft),

    //                                  5*sin(angle * PI/180)*pow(-1,(int)!isFront) + intX, RPS.X(), RPS.Y()));

    //    }

    //     //Horizontal and vertical will be determined somehow based on parallel to certain side

    //     while(!(dist >= 0 && dist <= .005));



    //Stop both motors.

    right_motor.Stop();

    left_motor.Stop();

}



void driveTo(Task t){
    platform_servo.SetDegree(90);
    Sleep(.5);
    arm_servo.SetDegree(180);
    switch(t){
    case TOKEN:
        break;
    case DDR:
        driveStraight(30,10);
        turn(false,true,30,90);
        driveStraight(30,10);
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

void complete(Task t){
    switch(t){
    case TOKEN:
        break;
    case DDR:
    {
        char* c = colorLight();
        if(strcmp("Red" , c) == 0){

        }else if(strcmp("Blue",c) == 0){

        }else{

        }
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

    arm_servo.SetDegree(INITIAL_ARM_ANGLE);

    claw_servo.SetDegree(INITIAL_CLAW_ANGLE);

    platform_servo.SetDegree(INITIAL_PLATFORM_ANGLE);

    /*
     * Set up encoder thresholds
     */
    left_encoder.SetThresholds(LEFT_THRESHOLD, LEFT_THRESHOLD);

    right_encoder.SetThresholds(RIGHT_THRESHOLD, RIGHT_THRESHOLD);

    /*
     * Initial menu
     */
    LCD.Clear(BLACK);

    FEHIcon::Icon start_button;

    start_button.SetProperties("START", 150, 120, 80, 50, WHITE, WHITE);

    start_button.Draw();

    float xTouch, yTouch;

    while(!LCD.Touch(&xTouch,&yTouch));

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

            //Perform all possible 90 degree turns with 1/2 second gaps in between each

            turn(true,true,30,60);

            Sleep(500);

            turn(false,true,30,30);

            Sleep(500);

            //turn(false,true,30,90);

            //Sleep(500);

            //turn(false,false,30,90);

            //Sleep(500);

            //turn(true,false,30,90);

            //Sleep(500);

            //turn(true,true,30,90);

            //Sleep(500);

            //turn(false,false,30,90);

            //Sleep(500);

            //turn(false,true,30,90);

            //Sleep(500);

        }else if (arm_button.Pressed(x,y,0)) {

            testingServo(arm_servo);

        }else if (claw_button.Pressed(x,y,0)) {

            testingServo(claw_servo);

        }else if (platform_button.Pressed(x,y,0)) {

            testingServo(platform_servo);

        }else if (drive_button.Pressed(x,y,0)){

            driveStraight(25,10);

            //driveStraight(-25,5);

        }

    }

}

int main(void){

    LCD.Clear(BLACK);

    /*
     * Initialize the RPS
     */

    //RPS.InitializeTouchMenu();

    /*
     * Initialize robot
     */

    //initialState();

    /*
     * Method to test various features of the robot
     */

    testMethods();

    //arm_servo.TouchCalibrate();

    /*
     * Reading CdS_Cell values
     */

    //    while(true){

    //        LCD.WriteLine(CdS_Cell.Value());

    //        Sleep(1.0);

    //    }
//    int currTime = TimeNow();
//    while(strcmp("Red",colorLight()) != 0 || TimeNow() - currTime > 30);
//    driveTo(DDR);
//    complete(DDR);

}

