#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

motor MotorTest = motor(PORT9, ratio18_1, true);
encoder EncoderTest = encoder(Brain.ThreeWirePort.A);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
    // Nothing to initialize
}