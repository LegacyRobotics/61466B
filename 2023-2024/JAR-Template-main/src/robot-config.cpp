#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Rfront = motor(PORT10, ratio18_1, true);
motor Lfront = motor(PORT1, ratio18_1, false);
motor Rback = motor(PORT9, ratio18_1, true);
motor Lback = motor(PORT2, ratio18_1, false);
inertial Inertial = inertial(PORT19);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}