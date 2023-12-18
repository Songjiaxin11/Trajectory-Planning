#include "MyEncoder.h"

#include <cmath>
#include <iostream>

#include "robot-config.h"
#include "vex.h"
using namespace vex;
void myEncoder::updateAngle() {
    double encoderAngle = EncoderTest.position(deg);
    double newdeviationForEncoder = sqrt(deviation * deviation + filterEncoderError * filterEncoderError);  // 高斯噪声
    double motorAngle = MotorTest.position(deg);
    double newdeviationForMotor = filterMotorError;
    double Kg = sqrt(newdeviationForEncoder * newdeviationForEncoder /
                     (newdeviationForEncoder * newdeviationForEncoder + newdeviationForMotor * newdeviationForMotor));
    curAngle = encoderAngle + Kg * (encoderAngle - motorAngle);
    deviation = sqrt((1 - Kg) * newdeviationForEncoder * newdeviationForEncoder);
}

void myEncoder::updateMileage() {
    lastMileage = curMileage;
    updateAngle();
    curMileage = 2 * 3.14159 * curAngle * radius / 360;  // 3.492为轮子半径
}

void myEncoder::updateSpeed() {
    lastSpeed = curSpeed;
    updateMileage();
    double ret = (curMileage - lastMileage) * 1000 / sampleTime;  // 前向差分法
    if (abs(ret) > 1000 || abs(ret) < 0.001) ret = 0;

    // 滤波前的速度
    Speed[2] = Speed[1];
    Speed[1] = Speed[0];
    Speed[0] = ret;

    // 滤波后的速度
    filtSpeed[2] = filtSpeed[1];
    filtSpeed[1] = filtSpeed[0];
    filtSpeed[0] = (filter_b[0] * Speed[0] + filter_b[1] * Speed[1] + filter_b[2] * Speed[2] -
                    filter_a[1] * filtSpeed[1] - filter_a[2] * filtSpeed[2]) /
                   filter_a[0];
    if (abs(filtSpeed[0]) < 0.001) filtSpeed[0] = 0;
    curSpeed = filtSpeed[0];
    curAngleSpeed = curSpeed / radius;
}

void update() {
    while (true) {
        myEncoder::getInstance()->updateSpeed();
        this_thread::sleep_for(sampleTime);
    }
}