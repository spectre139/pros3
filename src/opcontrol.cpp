#include "main.h"
#include "robot.h"
#include "api.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
//have to add these next lines for EVERY scope per instance of sensor use... ugh ok.


void flywheelControl(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    while(true){
        float goalVel = 0;
        if(master.btnL1){
            goalVel = 150;//150rpm for high/far flags
        }
        else if(master.btnL2){
            goalVel = 115;//115rpm for medium/close flags
        }
        else if(master.btnA){
            goalVel = 60;//60rpm for low power use
        }
        else if(master.btnB){
            goalVel = 0;//0rpm for off flywheel
        }
        r->flywheel.moveVel(goalVel);
        delay(20);
    }
}
void updateSensor(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    const float delayAmnt = 20;//ms delay for velocity calculations
    while(true){
        r->flywheel.computeVel();
        r->base.computeVel();
        r->base.computeRotVel();
        delay(delayAmnt);
    }
}
void updatePIDs(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    const float delayAmnt = 10;
    while(true){//nothing yet
        if(r->flywheel.pid.isRunning) r->flywheel.PID();
        if(r->indexer.pid.isRunning) r->indexer.PID();

        delay(delayAmnt);
    }
}
void opcontrol() {
    Robot rob = Robot();
    Task odometryCalculations(calculatePos, &rob.base.odom);
    Task controlFlywheel(flywheelControl, &rob);
    Task sensorUpdates(updateSensor, &rob);
    Task pidUpdates(updatePIDs, &rob);
  while (true) {
      rob.base.driveLR(master.leftY, master.rightY);
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
