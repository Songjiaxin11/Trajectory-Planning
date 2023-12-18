#include <math.h>
#include <iostream>
#include "MyEncoder.h"
#include "robot-config.h"
#include "vex.h"
#include "myMotor.h"
using namespace vex;
using namespace std;
void move1(double w);
void test1();

int main() {
    MotorTest.setPosition(0, deg);
    controller Controller = controller(primary);
    int A = Controller.ButtonA.pressing();
    int last_A;
    vexcodeInit();
    // test1();
    move1(15);
}

void test1() {
    vex::thread EncoderUpdate(update);
    MotorTest.setPosition(0, deg);
    EncoderTest.setPosition(0, deg);
    vexDelay(100);
    for (int v = 400; v <= 12700; v = v + 50) {
        MotorTest.spin(fwd, v, vex::voltageUnits::mV);
        vexDelay(5000);
        double res = 0;
        for (int i = 0; i < 20; i++) {
            res += myEncoder::getInstance()->getCurAngleSpeed();
            vexDelay(10);
        }
        res /= 20;
        cout << v << " " << res << endl;
    }
    MotorTest.stop();
}

void move1(double w) {  // 输入的w为角速度 单位:rad/s
    vex::thread encoderUpdate(update);
    vex::thread motorUpdate(updateMotorSpeed);
    vexDelay(100);
    double startTime;
    startTime = Brain.Timer.value();
    myMotor::getInstance()->setMotorSpeed(w);
    // double V = (w + 0.65) / 0.0022;
    // MotorTest.spin(fwd, V, vex::voltageUnits::mV);
    // while (test.getCurAngleSpeed() <= w) {
    //     test.updateSpeed();
    //     cout << Brain.Timer.value() - startTime << " " << test.getCurAngleSpeed() << endl;
    //     vexDelay(sampleTime);
    // }
    for (int i = 0; i < 800; i++) {
        cout << Brain.Timer.value() - startTime << " " << myEncoder::getInstance()->getCurAngleSpeed() << endl;
        vexDelay(10);
    }
    myMotor::getInstance()->stop();
}