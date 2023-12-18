#ifndef _MYMOTOR
#define _MYMOTOR
#include "MyEncoder.h"
#include "PID.h"
#include "vex.h"

class myMotor {
private:
    myEncoder *encoder = myEncoder().getInstance();
    double speed = 0;  // rad/s
    motor Motor = motor(PORT9, ratio18_1, true);
    PID pid;

public:
    myMotor() { pid.setCoefficient(350, 0, 315 + 100); }

    static myMotor *getInstance() {
        static myMotor *instance = NULL;
        if (instance == NULL) {
            instance = new myMotor();
        }
        return instance;
    }

    void setMotorSpeed(double _speed) {
        speed = _speed;
        pid.setTarget(speed);
    };
    double getTarSpeed() { return speed; };
    double getCurSpeed() { return encoder->getCurAngleSpeed(); };
    double speedToVoltage(double speed) { return ((speed + 0.65) / 0.0022); };
    void stop() { speed = 0; }
    void updatePID() { pid.update(getCurSpeed()); }
    double PID_Output() { return pid.getOutput(); }
};

void updateMotorSpeed();

#endif  // _MYMOTOR