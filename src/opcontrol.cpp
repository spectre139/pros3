#include "main.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
//have to add these next lines for EVERY scope per instance of sensor use... ugh ok.
Motor LFrontBase (motorLFront_Port);
Motor LRearBase (motorLRear_Port);
Motor RFrontBase (motorRFront_Port);
Motor RRearBase (motorRRear_Port);
Motor sprocket1 (sprocket1_Port);
Motor sprocket2 (sprocket2_Port);
Motor indexer (indexer_Port);

ADIEncoder encoderL (encoderLTop_Port, encoderLTop_Port);
ADIEncoder encoderR (encoderRTop_Port, encoderRTop_Port);
ADIEncoder encoderM (encoderMTop_Port, encoderMTop_Port);

void drive(){//BASE
    int leftJoystick = clamp(127, -127, master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
    int rightJoystick = clamp(127, -127, master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

    LFrontBase.move(leftJoystick);
    LRearBase.move(leftJoystick);
    RFrontBase.move(rightJoystick);
    RRearBase.move(rightJoystick);
}
void flywheel(){
	if(master.get_digital(E_CONTROLLER_DIGITAL_L1)){
        sprocket1.move(127);
        sprocket2.move(127);
    }
	else if(master.get_digital(E_CONTROLLER_DIGITAL_L2)){
        sprocket1.move(-127);
        sprocket2.move(-127);
    }
	else{
        sprocket1.move(0);
        sprocket2.move(0);
    }
	indexer.move(127 * master.get_digital(E_CONTROLLER_DIGITAL_R1) - 127*master.get_digital(E_CONTROLLER_DIGITAL_R2));//INDEXER
}

void opcontrol() {
  while (true) {
      drive();
      flywheel();

    string getMotorPos = string("Position: ") + std::to_string((int)LFrontBase.get_position());
	lcd::print(0, getMotorPos.c_str());
	string getMotorVel = string("Velocity: ") + std::to_string((int)LFrontBase.get_actual_velocity());
	lcd::print(1, getMotorVel.c_str());
    delay(2);
  }
}
