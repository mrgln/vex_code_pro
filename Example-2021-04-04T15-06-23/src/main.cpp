/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Margulan                                         */
/*    Created:      Sun Apr 04 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LF                   motor         3               
// RF                   motor         5               
// right_intake         motor         6               
// FlyWheel             motor         7               
// RB                   motor         15              
// LB                   motor         16              
// MiddleRollers        motor         18              
// left_intake          motor         21              
// GyroB                gyro          B               
// EncoderG             encoder       G, H            
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

#include "cmath"
#include "string"

using namespace vex;

competition Competition;

motor_group driveMotors = motor_group(RF, RB, LF, LB);

motor_group intakeMotors = motor_group(right_intake,left_intake);

//settings
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

//Autonomous Settings
int desiredValue = 200;
int desiredTurnValue = 0;

int error; //Current Value - Desired Value -> Positional Value (delta)
int preError = 0; // Position 20 ms ago
int derivative; // error - preError -> Speed
int totalError=0; // totalError = totalError + error

int turnError; //Current Value - Desired Value -> Positional Value (delta)
int turnPreError = 0; // Position 20 ms ago
int turnDerivative; // error - preError -> Speed
int turnTotalError=0; // totalError = totalError + error

bool resetDriveSensors = false;

bool enableDrivePid = true;


int drivePid()
{
  while(enableDrivePid)
  {
    if(resetDriveSensors)
    {
      resetDriveSensors = false;

      LF.setPosition(0,degrees);
      RF.setPosition(0,degrees);

    }


    //Get position of motors 
    int leftMotorPosition = LF.position(degrees);
    int rightMotorPosition = RF.position(degrees);




    /////////////////////////////////////////////////////
    /// Lateral Movement PID
    /////////////////////////////////////////////////////////////////////////

    int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //Potential
    error = averagePosition - desiredValue;

    //Derivative
    derivative = error - preError;

    //Integral
    totalError += error;

    double lateralMotorPower = (error * kP + derivative * kD + totalError * kI);






    /////////////////////////////////////////////////////
    /// Turning Movement PID
    //////////////////////////////////////////////////////////////////////////
    
    int turnDifference = leftMotorPosition - rightMotorPosition;

    //Potential
    turnError = turnDifference - desiredTurnValue;

    //Derivative
    turnDerivative = turnError - turnPreError;

    //Integral
    turnTotalError += turnError;

    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI);



    preError = error;
    turnPreError = turnError;
    vex::task::sleep(20);
  }

  return 1;
}

void pre_auton(void){
  vexcodeInit();
}

void autonomous(void)
{
  vex::task skills(drivePid);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(1000);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;


  //setvelocity
  FlyWheel.setVelocity(100,percent);
  right_intake.setVelocity(100,percent);
  left_intake.setVelocity(100,percent);
  driveMotors.setVelocity(100,percent);

  //programm
  FlyWheel.spinFor(forward,360,degrees);
  intakeMotors.spinFor(reverse,100,degrees);
  driveMotors.spinFor(forward,360,degrees);
}

//driving 
void usercontrol(void)
{
  enableDrivePid = false;
  
  while (1) {
    
    //main control
    LB.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::voltageUnits::volt);
    RB.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::voltageUnits::volt);
    RF.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::voltageUnits::volt);
    LF.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::voltageUnits::volt);

    //MOVE TO RIGHT 
    if(Controller1.ButtonY.pressing())
    {
      RF.setVelocity(100, percent);
      LF.setVelocity(100, percent);
      RB.setVelocity(100, percent);
      LB.setVelocity(100, percent);
      RF.spin(reverse);
      LF.spin(forward);
      RB.spin(forward);
      LB.spin(reverse);
    }

    //MOVE TO LEFT
    else if(Controller1.ButtonRight.pressing()){
      RF.setVelocity(100, percent);
      LF.setVelocity(100, percent);
      RB.setVelocity(100, percent);
      LB.setVelocity(100, percent);
      RF.spin(forward);
      LF.spin(reverse);
      RB.spin(reverse);
      LB.spin(forward);
    }


    FlyWheel.stop();
    MiddleRollers.stop();
    left_intake.stop(hold);
    right_intake.stop(hold);


    if(Controller1.ButtonL1.pressing()){
        FlyWheel.setVelocity(100, percent);
        MiddleRollers.setVelocity(100, percent);
        FlyWheel.spin(forward);
        MiddleRollers.spin(forward);
  
    }

    else if(Controller1.ButtonL2.pressing()){
        FlyWheel.setVelocity(100, percent);     
        right_intake.setVelocity(70, percent);
        left_intake.setVelocity(70, percent);   
        MiddleRollers.setVelocity(100, percent);
        FlyWheel.spin(reverse);
        MiddleRollers.spin(reverse);  
        right_intake.spin(reverse);
        left_intake.spin(reverse);     
    }

    else if(Controller1.ButtonR1.pressing()){
        FlyWheel.setVelocity(100, percent);
        MiddleRollers.setVelocity(100, percent);
        right_intake.setVelocity(100, percent);
        left_intake.setVelocity(100, percent);
        FlyWheel.spin(forward);
        MiddleRollers.spin(forward);
        right_intake.spin(forward);
        left_intake.spin(forward);    
    }

    else if(Controller1.ButtonR2.pressing()){

        FlyWheel.setVelocity(100, percent);
        MiddleRollers.setVelocity(100, percent);
        right_intake.setVelocity(70, percent);
        left_intake.setVelocity(70, percent);

        FlyWheel.spin(reverse);
        MiddleRollers.spin(reverse);
        right_intake.spinFor(reverse,1,turns);
        left_intake.spinFor(reverse,1,turns) ; 
    }

    //Controller1.ButtonR2.pressed(gg);
    //Controller1.ButtonL2.pressed(gg);
    wait(20, msec); 
  }
}

//int main
int main(){

Competition.autonomous(autonomous);
Competition.drivercontrol(usercontrol);

pre_auton();

while(true){wait(100,msec);}
}
