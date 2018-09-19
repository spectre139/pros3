#include "main.h"

using namespace pros;

void opcontrol() {
  Controller master (E_CONTROLLER_MASTER);
  //have to add these next lines for EVERY scope per instance of sensor use... ugh ok.
  Motor example_motor (motorLFront_Port);
  ADIEncoder encoderL (encoderLTop_Port, encoderLTop_Port);
  ADIEncoder encoderR (encoderRTop_Port, encoderRTop_Port);
  ADIEncoder encoderM (encoderMTop_Port, encoderMTop_Port);

  while (true) {
	//analog stick test
	example_motor = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
	//digital button test
	if(master.get_digital(E_CONTROLLER_DIGITAL_A))
		example_motor = 127;
	else example_motor = 0;
	//digital button test & motorMove test
	if(master.get_digital(E_CONTROLLER_DIGITAL_B))
		example_motor.move(127);
	else example_motor.move(0);
	if(encoderR.get_value() > 100) example_motor = 50;
    delay(2);
  }
}
