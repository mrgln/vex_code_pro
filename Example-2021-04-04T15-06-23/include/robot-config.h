using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LF;
extern motor RF;
extern motor right_intake;
extern motor FlyWheel;
extern motor RB;
extern motor LB;
extern motor MiddleRollers;
extern motor left_intake;
extern gyro GyroB;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );