#include "main.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
//have to add these next lines for EVERY scope per instance of sensor use... ugh ok.

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
        delay(20);
    }
}
void buttonTask(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    while(false){
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
        r->flyWheelVelPID.setGoal(clamp(200, -200, r->flyWheelVelPID.getGoal()));//clamps goal
        delay(50);
    }
}
void updateSensor(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    const float delayAmnt = 10;
    while(true){
        r->flywheelVel = r->getFlywheelVel(delayAmnt);
        delay(delayAmnt);
    }
}
void opcontrol() {
    Robot rob = Robot(
    	Position(0, 0, 0),//init position
    	Position(0, -10, 0),//distance to trackers
    	Odometry(),//blank odometry
    	PIDcontroller(1.1, 0.0, 0.0, 3.0,  10, false, false),//angle PID
      PIDcontroller(1.5, 0.0, 0.0, 0.75, 10, true, false),//drive PID
      PIDcontroller(1.5, 0.0, 0.0, 1.50, 10, false, true),//flywheel PID
      PIDcontroller(2.0, 0.0, 0.0, 10,  10, true, true),//indexer PID
    	8.35//wheel width (in)
    );
    //Position realPos(0, 0, 0);
    Task odometryCalculations(calculatePos, &rob);
    Task controlFlywheel(flywheelControl, &rob);
    Task sensorUpdates(updateSensor, &rob);
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
      /*if(master.btnUP){
        rob.indexerPIDMove(300);
        delay(10);
      }
      else rob.indexerControl(0);/*
      else{
        if(master.btnR1){
            rob.indexerControl(127);
        }
        else if(master.btnR2){
            rob.indexerControl(-127);
        }
        else{
            rob.indexerControl(0);
        }
      }*/

      rob.driveLR(master.leftY, master.rightY);
      //rob.LFrontBase.move(clamp(127,-127, PIDPower));
           //debugs
     int i = 0;
       for(const string& s : rob.debugString()){
           lcd::print(i++, s.c_str());
       }

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
