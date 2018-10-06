#include "main.h"
#include <string>
using namespace pros;

#define reversed true

void initSensors(){
	/*ADIEncoder encoderL (encoderLTop_Port, encoderLTop_Port, false);
	ADIEncoder encoderR (encoderRTop_Port, encoderRTop_Port, false);
	ADIEncoder encoderM (encoderMTop_Port, encoderMTop_Port, false);
	encoderL.reset();
	encoderR.reset();
	encoderM.reset();*/
  pros::ADIPort sensor1 (1, E_ADI_DIGITAL_IN);
  pros::ADIPort sensor2 (2, E_ADI_DIGITAL_IN);
  pros::ADIPort sensor3 (3, E_ADI_DIGITAL_IN);
  pros::ADIPort sensor4 (4, E_ADI_DIGITAL_IN);
  pros::ADIPort sensor5 (5, E_ADI_DIGITAL_IN);
  pros::ADIPort sensor6 (6, E_ADI_DIGITAL_IN);
  pros::ADIPort sensor7 (7, E_ADI_DIGITAL_IN);
  pros::ADIPort sensor8 (8, E_ADI_DIGITAL_IN);

  pros::ADIEncoder encoderL (encoderLTop_Port, encoderLBott_Port, false);
  pros::ADIEncoder encoderR (encoderRTop_Port, encoderRBott_Port, true);
  pros::ADIEncoder encoderM (encoderMTop_Port, encoderMBott_Port, false);
  encoderL.reset();
  encoderR.reset();
  encoderM.reset();
 }
 void initMotors(){//i wonder... do i even need this?
	 Motor LFrontBase_initializer (motorLFront_Port, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
	 Motor LRearBase_initializer (motorLRear_Port, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
	 Motor RFrontBase_initializer (motorRFront_Port, E_MOTOR_GEARSET_18, reversed, E_MOTOR_ENCODER_DEGREES);
	 Motor RRearBase_initializer (motorRRear_Port, E_MOTOR_GEARSET_18, reversed, E_MOTOR_ENCODER_DEGREES);
	 //sporkets
	 Motor flywheel1_initializer (flywheel1_port, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
	 Motor flywheel2_initializer (flywheel2_port, E_MOTOR_GEARSET_18, reversed, E_MOTOR_ENCODER_DEGREES);
	 //indexer
	 Motor indexer_initializer (indexer_Port, E_MOTOR_GEARSET_18, !reversed, E_MOTOR_ENCODER_DEGREES);
 }
void initialize() {
	lcd::initialize();
//	lcd::set_text(1, "Welcome 139A Gods");
	initSensors();
	initMotors();
	//Task odometryCalculations(calculatePos, &text);
	//Task my_task(my_task_fn, &text);


}

void disabled() {}


void competition_initialize() {}
