#include "MyMotor.h"

void updateMotorSpeed() {
    while (true) {
        myMotor *m = myMotor::getInstance();
        double tarSpeed = m->getTarSpeed();
        double voltage = m->speedToVoltage(tarSpeed);
        m->updatePID();
        
        if (tarSpeed == 0) {
            MotorTest.stop(coast);
            continue;
        }

        voltage += m->PID_Output();

        MotorTest.spin(fwd, voltage, vex::voltageUnits::mV);
        this_thread::sleep_for(10);
    }
}