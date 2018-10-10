#ifndef SLEW_H
#define SLEW_H

#include "api.h"
#include <vector>

class motorSlews{
public:
    motorSlews(pros::Motor m, int a) : mot(m), amnt(a), goal(0) {}
    pros::Motor mot;
    int goal;
    int amnt;
    bool moveByVel = false;
    float getOldPower(){
        //get_voltage is from 12000 to -12000, and analogous to 127 to -127 power from v4, so scale is 94.5:1
        return mot.get_voltage()/94.5;
    }
};

void MotorSlewRateTask(void * param);
//TaskHandle SlewRateMotorTask = taskCreate(MotorSlewRateTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);

#endif
