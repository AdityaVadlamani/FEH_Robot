#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>
#define PI 3.14159265
#define WIDTH 8.76
#define HEIGHT 3.74
#define DEPTH 8.4
#define TOLERANCE .005
//#define ARM_SERVO_MIN 500
//#define ARM_SERVO_MAX 2300
//#define CLAW_SERVO_MIN 500
//#define CLAW_SERVO_MAX 2300
//#define PLATFORM_SERVO_MIN 500
//#define PLATFORM_SERVO_MAX 2300
#define INITIAL_ANGLE 90.0
#define LOW_THRESHOLD 3.15
#define HIGH_THRESHOLD 3.3
#define COUNT_PER_INCH 8/(3.5*PI)
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
 * Servo motor for the rotating platform holding the arm
 */
FEHServo platform_servo(FEHServo::Servo0);

/*
 * Servo motor for the bending of the arm
 */
FEHServo arm_servo(FEHServo::Servo1);

/*
 * Servo motor for the bending of the claw
 */
FEHServo claw_servo(FEHServo::Servo2);

/*
 * Struct for each task
 */
struct Task{
private:
    float x;
    float y;
public:
    Task(float a, float b){
        x = a;
        y = b;
    }
};

/*
 * Declaration of task obejcts
 */

/*
 * Helper function for distance
 */
float distance(float x, float y, float z, float w){
    return pow(pow(z - x,2) + pow(w - y,2),.5);
}

/*
 * Helper function for tolerance
 */
bool withinTolerance(float act, float exp){
    return (act > exp - TOLERANCE*exp) && (act < exp + TOLERANCE*exp);
}

/*
 * Helper function to determine if light is red
 */
bool isRed(){
    return CdS_Cell.Value() <= .175;
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


     //Using encoders
//      right_encoder.ResetCounts();
//      left_encoder.ResetCounts();

//      while(!withinTolerance(dist,(right_encoder.Counts() + left_encoder.Counts()) / 2*COUNT_PER_INCH));
        Sleep(2.0);

     //Stop both motors.
     right_motor.Stop();
     left_motor.Stop();
}

void turn(bool isLeft, bool isFront, float motor_percent, float angle){
    //Get initial position of robot QR code
//    float intX = RPS.X();
//    float intY = RPS.Y();

    //Turn both motors on at given percent motor power.

     left_motor.SetPercent(motor_percent * !isLeft * pow(-1,(int)!isFront) + 10*isLeft);
     right_motor.SetPercent(motor_percent * isLeft * pow(-1,(int)!isFront) + 10*!isLeft);

     //Using encoders

     if(isLeft){
         right_encoder.ResetCounts();
         while(right_encoder.Counts() <= (8*9.45/7.0)*sin(angle*PI/180.));
     }else{
         left_encoder.ResetCounts();
         while(left_encoder.Counts() <= (8*9.45/7.0)*sin(angle*PI/180.));
     }
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

/*
 * Reset arm
 */
void resetArm(){
    arm_servo.SetDegree(INITIAL_ANGLE);
    claw_servo.SetDegree(INITIAL_ANGLE);
    platform_servo.SetDegree(INITIAL_ANGLE);
}

void driveTo(Task t){
    float intX = RPS.X();
    float intY = RPS.Y();
}

void completeDDR(){

}

void completeFoosball(){

}

void completeLever(){

}

void completeToken(){

}

void completeFinalButton(){

}

/*
 * Testing methods
 */

void testingServo(FEHServo s){
    float deg = 0.00;
    while(deg <= 180.00){
        s.SetDegree(deg);
        deg+=.01;
    }
    while(deg > 0){
        deg-=.01;
        s.SetDegree(deg);
    }
    resetArm();
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

    while(true){
        float x, y;
        LCD.Touch(&x,&y);
        if(turn_button.Pressed(x,y,0)){
            //Perform all possible 90 degree turns with 1/2 second gaps in between each
            turn(true,true,30,90);
            Sleep(500);
            turn(true,false,30,90);
            Sleep(500);
            turn(false,true,30,90);
            Sleep(500);
            turn(false,false,30,90);
            Sleep(500);
            turn(true,false,30,90);
            Sleep(500);
            turn(true,true,30,90);
            Sleep(500);
            turn(false,false,30,90);
            Sleep(500);
            turn(false,true,30,90);
            Sleep(500);
        }else if (arm_button.Pressed(x,y,0)) {
            testingServo(arm_servo);
        }else if (claw_button.Pressed(x,y,0)) {
            testingServo(claw_servo);
        }else if (platform_button.Pressed(x,y,0)) {
            testingServo(platform_servo);
        }
    }
}
int main(void)
{
    LCD.Clear(BLACK);
    /*
     * Initialize the RPS
     */
   // RPS.InitializeTouchMenu();

    /*
     * Initialize servos
     */
    //arm_servo.Calibrate();
    //arm_servo.SetMin(ARM_SERVO_MIN);
    //arm_servo.SetMax(ARM_SERVO_MAX);

    /*
     * Method to test various features of the robot
     */
    //testMethods();

    while(true){
        LCD.WriteLine("Left: ");
        LCD.WriteLine(left_encoder_value.Value());
        LCD.WriteLine("");
        LCD.WriteLine("Right: ");
        LCD.WriteLine(right_encoder_value.Value());
       // LCD.WriteLine("Right: ");
       // LCD.WriteLine(right_encoder_value.Value());
       // LCD.WriteLine(CdS_Cell.Value());
        if(isRed()){
            driveStraight(25,5);
        }

        Sleep(1.0);
    }
}
