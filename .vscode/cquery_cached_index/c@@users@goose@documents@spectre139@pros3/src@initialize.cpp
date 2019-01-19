#include "api.h"
#include "lowLevel.h"
#include <string>
using namespace pros;

#define reversed true


#define RFront   	2
#define LFront    14
#define LBack     13
#define RBack     12
#define LDiff     1
#define RDiff     11

#define LeftBAvg avg(Motor(LFront).get_position(), Motor(LBack).get_position())
#define RightBAvg avg(Motor(RFront).get_position(), Motor(RBack).get_position())


void initMotors(){
  Motor RFrontMot_init (RFront, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
  Motor LFrontMot_init (LFront, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
  Motor LBackMot_init  (LBack, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
  Motor RBackMot_init  (RBack, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
  Motor LDiffMot_init  (LDiff, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
  Motor RDiffMot_init  (RDiff, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
  //USE E_MOTOR_GEARSET_06 for the 600rpm mots
}
void initialize() {
    lcd::initialize();
    initMotors();
}

void disabled() {}


void competition_initialize() {}
