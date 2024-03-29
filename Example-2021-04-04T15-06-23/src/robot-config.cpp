#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LF = motor(PORT3, ratio18_1, false);
motor RF = motor(PORT5, ratio18_1, true);
motor FlyWheel = motor(PORT7, ratio18_1, true);
motor RB = motor(PORT15, ratio18_1, true);
motor LB = motor(PORT16, ratio18_1, false);
motor MiddleRollers = motor(PORT18, ratio18_1, false);
motor left_intake = motor(PORT21, ratio18_1, true);
gyro GyroB = gyro(Brain.ThreeWirePort.B);
controller Controller1 = controller(primary);
motor right_intake = motor(PORT11, ratio18_1, false);
distance DistanceSensor = distance(PORT6);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}