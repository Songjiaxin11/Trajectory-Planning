#ifndef _MYENCODER
#define _MYENCODER
#include <cmath>
#include <iostream>

#include "robot-config.h"
#include "vex.h"
using namespace vex;

const int sampleTime = 50;
class myEncoder {
private:
    /// @todo
    double Speed[3] = {0, 0, 0};
    double filtSpeed[3] = {0, 0, 0};  // 轮子转速角速度
    double lastAngle, curAngle;       // 轮子转过角度
    const double radius = 3.492;
    /// @todo
    double filterMotorError = 8, filterEncoderError = 8;  // 两个encoder的误差
    double deviation = 0;                                 // 偏差
    double curMileage = 0, lastMileage = 0;
    double lastSpeed = 0, curSpeed = 0;
    double curAngleSpeed = 0;
    double filter_b[3] = {0.0675, 0.1349, 0.0675};
    double filter_a[3] = {1, -1.143, 0.4128};

public:
    myEncoder(double errorEncoder = 8, double errorMotor = 8) {
        filterEncoderError = errorEncoder;
        filterMotorError = errorMotor;
    };
    void updateAngle();
    void updateMileage();
    void updateSpeed();
    double getCurAngle() { return curAngle; };
    double getCurSpeed() { return curSpeed; };
    double getCurAngleSpeed() { return curAngleSpeed; };  // 单位弧度每s


    static myEncoder *getInstance() {
        static myEncoder *e = NULL;
        if (e == NULL){
            e = new myEncoder();
        }
        return e;
    };
};

void update();

#endif // _MYENDOER