/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       woojin                                                    */
/*    Created:      Mon Mar 07 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

//ASSUMES YOU HAV MOTOR GROUPINGS FOR LEFT SIDE, RIGHT SIDE, AND WHOLE DRIVETRAIN

using namespace vex;
double driveRemainder=0;
//parameters: linear distance wanted in INCHES
void drivePD(double finalDrive){
  /*
  TUNING INSTRUCTIONS:
  1. set all values to 0 and tune the kP (proportionality) until it begins to overshoot.
    You know it's overshooting if the "Done!" message is not printed at the end of drive
  2. Add some kD (derivative) to ensure a consistent and smooth drive. If already smooth, 
    still add a small amount to help with integral tuning. Adding derivative helps the robot
    slow down more for more accuracy
  3. Add some kI (integral). BE CAREFUL, integral is a bitch to tune sometimes. too much 
    and you will inconsistently overshoot. Too little and it provides insignificant power.
    Aside from changing integral values, adding more derivative or decreasing kP will help 
    mitigate integral and the inverse is also true.Properly tuned integral should be almost
    unnoticeable without resistance but provide an extra burst of power when theres resistance.
  */

  double drivekP=.0177;
  double drivekD=.00037;
  double drivekI=0.0022;
  //README: replace with motor encoders
  parallelSensor.resetPosition();
  double driveError=0;
  double drivePreviousError=0;
  //converts inches to degrees
  double driveFinalValue=(finalDrive*(360/(3.25*M_PI))+driveRemainder);
  double totalError=0;
  driveError=driveFinalValue;
  Brain.Screen.clearScreen();
  while ((fabs(driveError)>=10)){

    /* IMPORTANT***********:
    replace parallelSensor with ur chassis motor encoder values 
    and then get the average of all of them
    */
    double drivePosition=parallelSensor.position(deg);
  
    //proportional
    driveError=driveFinalValue-drivePosition;

    //derivative
    double driveDerivative=driveError-drivePreviousError;

    //integral
    if(fabs(driveError)<50){
      totalError+=driveError;
    }
    else{
      totalError=0;
    }

    //sets up the motor power for each side
    double driveMotorPower=driveError*drivekP+driveDerivative*drivekD+totalError*drivekI;

    rightSide.spin(fwd,driveMotorPower,volt);
    leftSide.spin(fwd,driveMotorPower,volt);

    //updates the previousErrors for the next while loop iteration for accurate derivatives
    drivePreviousError=driveError;
    //this helps you see any undershooting or overshooting
    Brain.Screen.printAt(20,20,"%f",driveError);

    //Sleep the PD for a short amount of time to prevent wasted resources.
    wait(20,msec);
  }
  driveRemainder=driveError;
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(20,20,"Done!");
  //stops motors once error is within margin of error
  driveTrain.stop(brake);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
