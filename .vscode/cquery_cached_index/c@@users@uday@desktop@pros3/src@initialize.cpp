#include "api.h"
#include "lowLevel.h"
#include <string>
using namespace pros;

#define reversed true
#define encoderLTop_Port  1
#define encoderLBott_Port 2

#define encoderRTop_Port  3
#define encoderRBott_Port 4

#define encoderMTop_Port  5
#define encoderMBott_Port 6

#define encoderFlywheelTop  7
#define encoderFlywheelBott 8

//defining motor ports:
#define flywheel1_port   3
#define flywheel2_port   4

#define intake_Port      19
#define indexer_Port     5

#define motorLFront_Port 7
#define motorLRear_Port  8
#define motorRFront_Port 9
#define motorRRear_Port  20

void initMotors(){//i wonder... do i even need this?
    Motor LFrontBase_initializer (motorLFront_Port, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
    Motor LRearBase_initializer (motorLRear_Port, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
    Motor RFrontBase_initializer (motorRFront_Port, E_MOTOR_GEARSET_18, reversed, E_MOTOR_ENCODER_DEGREES);
    Motor RRearBase_initializer (motorRRear_Port, E_MOTOR_GEARSET_18, reversed, E_MOTOR_ENCODER_DEGREES);
    //flywheel
    Motor flywheel1_initializer (1, E_MOTOR_GEARSET_06);
    Motor flywheel2_initializer (2, E_MOTOR_GEARSET_06);
    //indexer
    Motor indexer_initializer (indexer_Port, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
    Motor intake_initializer (intake_Port, E_MOTOR_GEARSET_18, reversed, E_MOTOR_ENCODER_DEGREES);
}
void initialize() {
    lcd::initialize();
    initMotors();
}

void disabled() {}


void competition_initialize() {}
