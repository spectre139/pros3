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
void calculatePos(void* param){
    Robot* r = (Robot*) param;
	for(;;){

		float dR = encoderDistInch(encoderR.get_value())  - r->odom.lastR;//change in right encoder
		float dL = encoderDistInch(encoderL.get_value())  - r->odom.lastL;//change in left
		float dM = encoderDistInch(encoderM.get_value())  - r->odom.lastM;//change in middle

		r->odom.lastR += dR;//updates "last" values
		r->odom.lastL += dL;//updates "last" values
		r->odom.lastM += dM;//updates "last" values

		float dCentral = avg(dR, dL);//average of deltas
		float dHeading = toDeg(dR - dL) / r->wheelWidth;//change in angle
		float avgHeading = normAngle(r->pos.heading + dHeading / 2.0);  // Angle r is assumed to have been facing when moving dS.

		float radHeading = boundAngle(toRad(avgHeading));
		// Update current r position.
		r->t_pos.heading += dHeading;
		r->t_pos.X += dCentral * cos(radHeading) + dM * sin(radHeading);
		r->t_pos.Y += dCentral * sin(radHeading) - dM * cos(radHeading);
		//add little vector after calculating H mech's position
		const float distToCenter = 2.18524;//inches the center of the H mech to the center of r's rotation
		r->pos.heading = normAngle(r->t_pos.heading + 90);
		r->pos.X = r->t_pos.X + distToCenter * cos(radHeading);
		r->pos.Y = r->t_pos.Y + distToCenter * sin(radHeading) - distToCenter;
		//little delays
        delay(1);
	}
}
void pidTest(void* param){
    Robot* r = (Robot*) param;
    while(false){
        r->distPID.goal = 506.5;
        //LFrontBase = r->distPID.pidCompute(LFrontBase.get_position());
        delay(20);
    }
}

void opcontrol() {
    Robot rob = Robot(
    	Position(0, 0, 0),//init position
    	Position(0, -10, 0),//distance to trackers
    	Odometry(),//blank odometry
    	PIDs(1.1, 0.3, 0.0, 3.0, 10, false, true),//angle PID
    	PIDs(1.5, 0.0, 0.0, 0.75, 10, true, true),//drive PID
    	8.35//wheel width (in)
    );
    //Position realPos(0, 0, 0);
    Task odometryCalculations(calculatePos, &rob);
    Task pidMotor(pidTest, &rob);
  rob.distPID.goal = 506.5;
  while (true) {
      //drive();
      flywheel();
      if(master.get_digital(E_CONTROLLER_DIGITAL_A)){
          rob.distPID.goal = rand() % 2000 - 1000;//new goal
      }
      float error = LFrontBase.get_position() - rob.distPID.goal;
      float PIDPower = rob.distPID.pidCompute(LFrontBase.get_position());
      LFrontBase.move(clamp(127,-127,PIDPower));
           //debugs
      string getMotorPos = string("Position: ") + std::to_string(LFrontBase.get_position());
	  lcd::print(0, getMotorPos.c_str());
	  string getMotorVel = string("Velocity: ") + std::to_string(LFrontBase.get_actual_velocity());
	  lcd::print(1, getMotorVel.c_str());
      string errorPID = string("Error: ") + std::to_string(error);
	  lcd::print(2, errorPID.c_str());
      string goalPID = string("Goal: ") + std::to_string(rob.distPID.goal);
	  lcd::print(3, goalPID.c_str());
      string PIDPowerS = string("PIDPower: ") + std::to_string(PIDPower);
	  lcd::print(4, PIDPowerS.c_str());
      string percentError = string("error Percent: ") + std::to_string((LFrontBase.get_position() - rob.distPID.goal) / rob.distPID.goal);
	  lcd::print(5, percentError.c_str());
      delay(2);
  }
}
