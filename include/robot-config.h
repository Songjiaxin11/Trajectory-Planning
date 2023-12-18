#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_
#include "vex.h"

using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial Inertial;
// Base motors
// Left/Right Front/Back Up/Down
extern motor MotorTest;

extern controller Controller;

extern vex::distance Distance;

extern encoder EncoderTest;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

#endif