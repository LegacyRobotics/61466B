using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor Rfront;
extern motor Lfront;
extern motor Rback;
extern motor Lback;
extern inertial Inertial;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );