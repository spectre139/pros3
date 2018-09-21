#include "main.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
//have to add these next lines for EVERY scope per instance of sensor use... ugh ok.

void calculatePos(void* param){
    Robot* r = (Robot*) param;
	for(;;){

		float dR = encoderDistInch(r->encoderR.get_value())  - r->odom.lastR;//change in right encoder
		float dL = encoderDistInch(r->encoderL.get_value())  - r->odom.lastL;//change in left
		float dM = encoderDistInch(r->encoderM.get_value())  - r->odom.lastM;//change in middle

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
        r->distPID.setGoal(506.5);
        //LFrontBase = r->distPID.pidCompute(LFrontBase.get_position());
        delay(20);
    }
}
void flywheelControl(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    while(true){
        if(master.btnL1){
            r->enableFlywheelPID(OFF);
            r->flywheelControl(127);
            r->flyWheelVelPID.setGoal(0);
        }
        else if(master.btnL2){
            r->enableFlywheelPID(OFF);
            r->flywheelControl(-127);
            r->flyWheelVelPID.setGoal(0);
        }
        else{
            r->enableFlywheelPID(ON);
            r->flywheelPID();
        }
        delay(50);
    }
}
void buttonTask(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    while(true){
        if(master.btnUP){
            r->flyWheelVelPID.setGoal(r->flyWheelVelPID.getGoal() + 5);//+=5
            delay(100);
        }
        if(master.btnDOWN) {
            r->flyWheelVelPID.setGoal(r->flyWheelVelPID.getGoal() - 5);//+=5
            delay(100);
        }
        if(master.btnRIGHT) {
            r->flyWheelVelPID.setGoal(0);//==0
            delay(100);
        }
        r->flyWheelVelPID.setGoal(clamp(127, -127, r->flyWheelVelPID.getGoal()));//clamps goal
        delay(50);
    }
}
void opcontrol() {
    Robot rob = Robot(
    	Position(0, 0, 0),//init position
    	Position(0, -10, 0),//distance to trackers
    	Odometry(),//blank odometry
    	PIDcontroller(1.1, 0.0, 0.0, 3.0, 10, false, false),//angle PID
        PIDcontroller(1.5, 0.0, 0.0, 0.75, 10, true, false),//drive PID
        PIDcontroller(1.5, 0.0, 0.0, 1.50, 10, false, true),//flywheel PID
    	8.35//wheel width (in)
    );
    //Position realPos(0, 0, 0);
    Task odometryCalculations(calculatePos, &rob);
    Task controlFlywheel(flywheelControl, &rob);
    Task taskButton(buttonTask, &rob);
  rob.distPID.setGoal(506.5);
  while (true) {
      if(master.btnR1){
          rob.indexerControl(127);
      }
      else if(master.btnR2){
          rob.indexerControl(-127);
      }
      else{
          rob.indexerControl(0);
      }

      rob.driveLR(master.leftY, master.rightY);
      //rob.LFrontBase.move(clamp(127,-127, PIDPower));
           //debugs

      string getFLywheelVel = string("motor1 Vel: ") + std::to_string(rob.sprocket1.get_actual_velocity());
      lcd::print(0, getFLywheelVel.c_str());
      string getFLywheel2Vel = string("motor2 Vel: ") + std::to_string(rob.sprocket2.get_actual_velocity());
      lcd::print(1, getFLywheel2Vel.c_str());
      string flywheelState;
      if(rob.flyWheelVelPID.getRunningState())
        flywheelState = string("PID Running: YES");
      else
        flywheelState = string("PID Running: NO");
      lcd::print(3, flywheelState.c_str());
      string flywheelGoal = string("PID Goal: ") + std::to_string(rob.flyWheelVelPID.getGoal());
      lcd::print(4, flywheelGoal.c_str());

    /* float error = rob.LFrontBase.get_position() - rob.distPID.getGoal();
     float PIDPower = rob.distPID.pidCompute(rob.LFrontBase.get_position());
     if(master.btnA){
         rob.distPID.setGoal( rand() % 2000 - 1000 );//new goal
     }
      string getMotorPos = string("Position: ") + std::to_string(rob.LFrontBase.get_position());
	  lcd::print(0, getMotorPos.c_str());
	  string getMotorVel = string("Velocity: ") + std::to_string(rob.LFrontBase.get_actual_velocity());
	  lcd::print(1, getMotorVel.c_str());
      string errorPID = string("Error: ") + std::to_string(error);
	  lcd::print(2, errorPID.c_str());
      string goalPID = string("Goal: ") + std::to_string(rob.distPID.getGoal());
	  lcd::print(3, goalPID.c_str());
      string PIDPowerS = string("PIDPower: ") + std::to_string(PIDPower);
	  lcd::print(4, PIDPowerS.c_str());
      string percentError = string("error Percent: ") + std::to_string((rob.LFrontBase.get_position() - rob.distPID.getGoal()) / rob.distPID.getGoal());
	  lcd::print(5, percentError.c_str());*/
      delay(2);
  }
}
