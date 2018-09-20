#include "main.h"

using namespace pros;

int capAt(int max, int min, int amnt){
  if(amnt > max) return max;
  if(amnt < min) return min;
  return amnt;
}
void opcontrol() {
  Controller master (E_CONTROLLER_MASTER);
  //have to add these next lines for EVERY scope per instance of sensor use... ugh ok.
  Motor example_motor (motorLFront_Port);
  ADIEncoder encoderL (encoderLTop_Port, encoderLTop_Port);
  ADIEncoder encoderR (encoderRTop_Port, encoderRTop_Port);
  ADIEncoder encoderM (encoderMTop_Port, encoderMTop_Port);

  while (true) {
  	//analog stick test
  	example_motor.move(capAt(127, -127, master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)));
  	//digital button test
  	if(master.get_digital(E_CONTROLLER_DIGITAL_A) || master.get_digital(E_CONTROLLER_DIGITAL_R1))
  		example_motor = 50;
  	//digital button test & motorMove test
  	if(master.get_digital(E_CONTROLLER_DIGITAL_B) || master.get_digital(E_CONTROLLER_DIGITAL_LEFT))
  		example_motor.move(-50);


    delay(2);
  }
}
