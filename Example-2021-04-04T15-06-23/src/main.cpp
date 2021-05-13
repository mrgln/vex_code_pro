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
// FlyWheel             motor         7               
// RB                   motor         15              
// LB                   motor         16              
// MiddleRollers        motor         18              
// left_intake          motor         21              
// GyroB                gyro          B               
// Controller1          controller                    
// right_intake         motor         11              
// DistanceSensor       distance      6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

#include "cmath"
#include "string"

using namespace vex;

competition Competition;

void pre_auton(void){
  vexcodeInit();
}

void drive_PID(float target, int dir) {

  RB.resetRotation(); RF.resetRotation(); LB.resetRotation(); LF.resetRotation();

  int curPosition = (RB.rotation(rotationUnits::deg) + LB.rotation(rotationUnits::deg) + RF.rotation(rotationUnits::deg) + LF.rotation(rotationUnits::deg)) / 4;

  int error = target - curPosition;
  float Kp = 0.5;
  float Kd = 0.5;

  int derivative = 0;
  int lastError = error;
  
  if(dir == 1)
  {
    while(error > 5) {
    curPosition = (RB.rotation(rotationUnits::deg) + LB.rotation(rotationUnits::deg) + RF.rotation(rotationUnits::deg) + LF.rotation(rotationUnits::deg)) / 4;
    
    error = target - curPosition;
    derivative = error - lastError;

    RB.setVelocity(((error * Kp) + (derivative *Kd)), rpm);
    LB.setVelocity(((error * Kp) + (derivative *Kd)), rpm);
    RF.setVelocity(((error * Kp) + (derivative *Kd)), rpm);
    LF.setVelocity(((error * Kp) + (derivative *Kd)), rpm);

    RB.spin(forward);
    LB.spin(forward);
    RF.spin(forward);
    LF.spin(forward);

    lastError = error;

    vex::task::sleep(10);
    }
  }
  else
  {
    while(error > 5) {
    curPosition = (RB.rotation(rotationUnits::deg) + LB.rotation(rotationUnits::deg) + RF.rotation(rotationUnits::deg) + LF.rotation(rotationUnits::deg)) / 4;
    
    error = target - std::abs(curPosition);

    RB.setVelocity((error * Kp), rpm);
    LB.setVelocity((error * Kp), rpm);
    RF.setVelocity((error * Kp), rpm);
    LF.setVelocity((error * Kp), rpm);

    RB.spin(reverse);
    LB.spin(reverse);
    RF.spin(reverse);
    LF.spin(reverse);

    lastError = error;

    vex::task::sleep(10);
    }
  }

  RB.stop();
  LB.stop();
  RF.stop();
  LF.stop();
}

void turn_PID(float target, int dir)
{
  RB.resetRotation(); RF.resetRotation(); LB.resetRotation(); LF.resetRotation();

  float curPosition = (RB.rotation(rotationUnits::deg) + RF.rotation(rotationUnits::deg))/2 - (LF.rotation(rotationUnits::deg)+LB.rotation(rotationUnits::deg))/2;

  float error = target-curPosition;
  float lastError = error;
  float derivative = 0;

  float Kp = 0.3; 
  float Kd = 0.01;
  
  if(dir==1)
  {
    while(error>5){
    curPosition = (RB.rotation(rotationUnits::deg) + RF.rotation(rotationUnits::deg))/2 - (LF.rotation(rotationUnits::deg)+LB.rotation(rotationUnits::deg))/2;
    error = target - curPosition;
    derivative = error - lastError;

    RF.setVelocity((error*Kp)+(derivative*Kd),rpm);
    RB.setVelocity((error*Kp)+(derivative*Kd),rpm);
    LF.setVelocity((error*Kp)+(derivative*Kd),rpm);
    LB.setVelocity((error*Kp)+(derivative*Kd),rpm);

    RB.spin(forward);
    RF.spin(forward);
    LB.spin(reverse);
    LF.spin(reverse);

    lastError = error;

    vex::task::sleep(10);
  }
  }
  else
  {
    while(error>5){
    curPosition = (RB.rotation(rotationUnits::deg) + RF.rotation(rotationUnits::deg))/2 - (LF.rotation(rotationUnits::deg)+LB.rotation(rotationUnits::deg))/2;
    error = target - std::abs(curPosition);
    derivative = error - lastError;

    RF.setVelocity((error*Kp)+(derivative*Kd),rpm);
    RB.setVelocity((error*Kp)+(derivative*Kd),rpm);
    LF.setVelocity((error*Kp)+(derivative*Kd),rpm);
    LB.setVelocity((error*Kp)+(derivative*Kd),rpm);

    RB.spin(reverse);
    RF.spin(reverse);
    LB.spin(forward);
    LF.spin(forward);

    lastError = error;

    vex::task::sleep(10);
  }
  }
  RB.stop();
  LB.stop();
  RF.stop();
  LF.stop();
}


void autonomous(void)
{
  drive_PID(1000,1);
  task::sleep(10);
  turn_PID(899.5,2);
  task::sleep(10);
  // while(DistanceSensor.objectDistance(mm)>300)
  // {
  //   RB.spin(reverse);
  //   RF.spin(reverse);
  //   LB.spin(reverse);
  //   LF.spin(reverse);
  // }
  //     RB.stop();
  //     RF.stop();
  //     LB.stop();
  //     LF.stop();
  drive_PID(1000,1);
  turn_PID(1799,2);

  //  while(DistanceSensor.objectDistance(mm)>300)
  // {
  //   RB.spin(reverse);
  //   RF.spin(reverse);
  //   LB.spin(reverse);
  //   LF.spin(reverse);
  // }
  //     RB.stop();
  //     RF.stop();
  //     LB.stop();
  //     LF.stop();
}

//driving 
void usercontrol(void)
{
  
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
        right_intake.spin(reverse);
        left_intake.spin(reverse) ; 
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
