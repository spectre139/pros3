#include "main.h"
#include "lowLevel.h"
#include "api.h"
#include "robot.h"
#include "odom.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
#define RFront   	4
#define LFront    5
#define LBack     6
#define RBack     7
#define LDiff     8
#define RDiff     9

void updatePIDs(void* param){//task test for flywheel PID
  Robot* r = (Robot*) param;
  const float delayAmnt = 10;
  while(true){//nothing yet
    //  if(r->indexer.pid.isRunning) r->indexer.PID();
    int i = 0;
    for(const string& s : r->debugString()){
      lcd::print(i++, s.c_str());
    }
    delay(delayAmnt);
  }
}
void updateSensor(void* param){//task test for flywheel PID
  Robot* r = (Robot*) param;
  const float delayAmnt = 20;//ms delay for velocity calculations
  while(true){
    //    r->flywheel.computeVel();
    r->base.computeVel();
    //    r->base.computeRotVel();
    delay(delayAmnt);

  }
}
int flyVel = 0;
int indexing = 1;


int brake = 1;
std::string brakeCheck(){
  if(brake == 1)  return "yes";
  else  return "no";
}

std::vector<pros::Motor> mots;
Robot rob = Robot();
void autonomous(){
  //add stuff here
}

float LeftBAvg, RightBAvg;

void opcontrol() {
  ADIPotentiometer lim (1);
  ADIPotentiometer pot (2);
  ADIEncoder enco (1, 2, false);
  Task sensorUpdates(updateSensor, &rob);
  Task odometryCalculations(calculatePosBASE, &rob.base.odom);
  rob.base.odom.resetEncoders = true;

  while (true) {
    LeftBAvg = avg(rob.base.mots[1].get_position(), rob.base.mots[1].get_position());//1, 2
    RightBAvg = avg(rob.base.mots[2].get_position(), rob.base.mots[2].get_position());//0, 3
    if(brake == 1) rob.base.driveLR(master.rightY, master.leftY);
    else rob.base.brakeHecka(pot.get_value());

    if(master.btnX) {
      brake = brake*-1;
      delay(300);
    }
    lcd::print(0, (string("Pos X: ") + std::to_string( rob.base.odom.pos.X)).c_str());
    lcd::print(1, (string("Pos Y: ") + std::to_string( rob.base.odom.pos.Y)).c_str());
    lcd::print(2, (string("Theta: ") + std::to_string( rob.base.odom.pos.heading)).c_str());
    lcd::print(3, (string("BLEnc: ") + std::to_string( LeftBAvg )).c_str());
    lcd::print(4, (string("BREnc: ") + std::to_string( RightBAvg )).c_str());
    lcd::print(5, (string("MEnc : ") + std::to_string( enco.get_value() )).c_str());
    delay(2);
  }
}
