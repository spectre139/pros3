#include "main.h"
#include "lowLevel.h"
#include "api.h"
#include "robot.h"
#include "odom.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
#define RFront   	2
#define LFront    14
#define LBack     13
#define RBack     12
#define LDiff     1
#define RDiff     11

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
int line;
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
int isPulled = 0;
int pleasePull = 3;
int line2;
int t = 0;
int kill = 0;
void cataPull(void* param){
  Robot* r = (Robot*) param;

while(true){
  if(kill == 0){
  if(pleasePull == 1){
if(line > 2500){
    r->cata.move(-127);
  }else{
    r->cata.move(0);
pleasePull = 2;
  }

}
else if(pleasePull == 0){
    if(t < 300){

        r->cata.move(-127);
        delay(1);
      t++;
    }else{
       r->cata.move(0);
}
  }
}else{
  r->cata.move(0);
}
}
}
float LeftBAvg, RightBAvg;
float encM, encL, encR;

void updateTask(void* param){//task test for flywheel PID
  Robot* r = (Robot*) param;
  ADIEncoder encMo (7, 8, false), encRo (5, 6, false), encLo (3, 4, false);
  ADILineSensor light (2);
  while(true){
    LeftBAvg = avg(r->base.mots[1].get_position(), r->base.mots[2].get_position());//1, 2
    RightBAvg = -avg(r->base.mots[0].get_position(), r->base.mots[3].get_position());//0, 3
    encM = encMo.get_value();
    encL = encLo.get_value();
    encR = -encRo.get_value();
  line = light.get_value();
    if(rob.base.odom.resetEncoders) {
      encMo.reset();  encLo.reset(); encRo.reset();
    }

    lcd::print(0, (string("Pos X: ") + std::to_string( rob.base.odom.pos.X)).c_str());
    lcd::print(1, (string("Pos Y: ") + std::to_string( rob.base.odom.pos.Y)).c_str());
    lcd::print(2, (string("Theta: ") + std::to_string( rob.base.odom.pos.heading)).c_str());
    lcd::print(3, (string("LEnc: ") + std::to_string( encL )).c_str());
    lcd::print(4, (string("REnc: ") + std::to_string( encR )).c_str());
    lcd::print(5, (string("MEnc : ") + std::to_string( encM )).c_str());
    lcd::print(6, (string("Line : ") + std::to_string( line )).c_str());
    lcd::print(7, (string("pp : ") + std::to_string( pleasePull )).c_str());

    delay(2);
  }
}
void opcontrol() {
  //ADIEncoder encMo (1, 2, false), encRo (3, 4, false), encLo (5, 6, false);
  pros::Vision vis (20);
  vis.clear_led();
  vision_object_s_t BLU = vis.get_by_sig(0, 1);
  Task sensorUpdates(updateSensor, &rob);
  Task taskUpdate(updateTask, &rob);
 Task cataControl(cataPull, &rob);
  Task odometryCalculations(calculatePosBASE, &rob.base.odom);
  rob.base.odom.resetEncoders = true;
  while (true) {

rob.intake.simpleControl(master.btnR2, master.btnR1);

    if(brake == 1) rob.base.driveLR(master.rightY, master.leftY);
    //else rob.base.brakeHecka(pot.get_value());

    if(master.btnL2) {
if((pleasePull == 0) || (pleasePull == 3)){
  pleasePull = 1;
}
else if((pleasePull == 1) || (pleasePull == 2)){
  t = 0;
  pleasePull = 0;
}
delay(300);
    }
    if(master.btnX) {
      brake = brake*-1;
      delay(300);
    }
    if(master.btnUP){
      rob.base.fwdsNEW(24);
    }
    if(master.btnY && master.btnB){
      kill = 1;
    }
    if(master.btnRIGHT){
      rob.base.odom.resetEncoders = true;
      rob.base.odom.resetAngleSentinel = 90;
    }
    if(master.btnDOWN){
      rob.base.turnNEW(90);
    }
    if(master.btnLEFT){
      rob.base.driveToPoint(-10, 10);
    }
    delay(2);
  }
}
