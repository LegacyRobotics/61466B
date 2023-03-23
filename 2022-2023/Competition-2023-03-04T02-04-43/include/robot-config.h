using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern controller Controller2;
extern motor Rfront;
extern motor Lfront;
extern motor Lback;
extern motor Rback;
extern motor_group Flywheel;
extern rotation FlywheelSpeed;
extern digital_out Indexer;
extern motor_group Intake;
extern digital_out Endgame;
extern inertial Gyro;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );