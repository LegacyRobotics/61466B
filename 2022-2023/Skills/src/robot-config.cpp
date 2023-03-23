#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor Rfront = motor(PORT10, ratio18_1, true);
motor Lfront = motor(PORT1, ratio18_1, false);
motor Lback = motor(PORT2, ratio18_1, false);
motor Rback = motor(PORT9, ratio18_1, true);
motor FlywheelMotorA = motor(PORT3, ratio18_1, false);
motor FlywheelMotorB = motor(PORT4, ratio18_1, true);
motor_group Flywheel = motor_group(FlywheelMotorA, FlywheelMotorB);
rotation FlywheelSpeed = rotation(PORT5, false);
digital_out Indexer = digital_out(Brain.ThreeWirePort.A);
motor IntakeMotorA = motor(PORT11, ratio18_1, true);
motor IntakeMotorB = motor(PORT20, ratio18_1, true);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);
digital_out Endgame = digital_out(Brain.ThreeWirePort.H);
inertial Gyro = inertial(PORT19);

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