#include "main.h"
#include "lowLevel.h"
#include "api.h"
#include "robot.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
//have to add these next lines for EVERY scope per instance of sensor use... ugh ok.



void updatePIDs(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    const float delayAmnt = 10;
    while(true){//nothing yet
        if(r->flywheel.pid.isRunning) r->flywheel.PID();
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
void flywheelControl(void* param){//task test for flywheel PID
  /**  Robot* r = (Robot*) param;
    while(true){
        if(master.btnUP){
            r->FWVelGoal = 200;//150rpm for high/far flags}
          }
        else if(master.btnDOWN){
            r->FWVelGoal = 180;//115rpm for medium/close flags
        }
        else if(master.btnA){
            r->FWVelGoal = 110;//60rpm for low power use
        }
        else if(master.btnB){
            r->FWVelGoal = 0;//0rpm for off flywheel
        }
        r->flywheel.moveVel(r->FWVelGoal);
        delay(20);
        */

        Robot* r = (Robot*) param;
         while(true){
             if(master.btnUP){
                flyVel = 600;//150rpm for high/far flags}
               }
             else if(master.btnDOWN){
                flyVel= 500;//115rpm for medium/close flags
             }
             else if(master.btnA){
                flyVel = 300;//60rpm for low power use
             }
             else if(master.btnB){
                 flyVel = 0;//0rpm for off flywheel
             }
             if(master.btnR2){
              indexing = 0;
               }
               else{
                 indexing = 1;
               }
             delay(20);
    }
}

void fwPID(void* param){//task test for flywheel PID

        Robot* r = (Robot*) param;
         while(true){
          if(flyVel >0){
            if(indexing == 0)
            r->flywheel.skrrt2(flyVel);
            else
            r->flywheel.skrrt(flyVel);
          }else{
            if(indexing == 0)
            r->flywheel.no2();
            else
            r->flywheel.no();

          }
             delay(20);
    }
}


int brake = 1;
std::string brakeCheck(){
if(brake==1)
return "yes";
else
return "no";

}

std::vector<pros::Motor> mots;
Robot rob = Robot();
void opcontrol() {
  /**  Task odometryCalculations(calculatePos, &rob.base.odom);
    Task controlFlywheel(flywheelControl, &rob);

    Task pidUpdates(updatePIDs, &rob);
    ADIEncoder encoderL (1, 2, true), encoderR (3, 4, true), encoderM (5, 6, false);
    ADIEncoder FWenc (7, 8, false);
    rob.base.odom.pos.X = 0;
    rob.base.odom.pos.Y = 0;**/

    ADIPotentiometer lim (1);
    ADIPotentiometer pot (2);
      Task controlFlywheel(flywheelControl, &rob);
      Task skrrt(fwPID, &rob);
    Task sensorUpdates(updateSensor, &rob);

    while (true) {
      if(brake == 1){
      rob.base.driveLRPOT(master.rightY, master.leftY, pot.get_value(), 1770, lim.get_value());}
      else{
      rob.base.brakeHecka(pot.get_value());}


        if(master.btnX) {
          brake = brake*-1;
delay(300);
        }
      lcd::print(1, (string("Pot: ") + std::to_string( pot.get_value())).c_str() ); // 2000 2300
lcd::print(2, (string("lim: ") + std::to_string( lim.get_value())).c_str() );
    /**  lcd::print(1, (string("LeftD: ") + std::to_string( mots[4].get_actual_velocity())).c_str() );
      lcd::print(1, (string("RightD: ") + std::to_string( mots[5].get_actual_velocity())).c_str() );**/
        //rob.LFrontBase.move(clamp(127,-127, PIDPower));
        //debugs
        //lcd::print(1, (string("EncoderL6: ") + std::to_string( encoderR.get_value())).c_str() );
        //lcd::print(2, (string("EncoderR6: ") + std::to_string( encoderL.get_value())).c_str() );
        //lcd::print(3, (string("EncoderM6: ") + std::to_string( encoderM.get_value())).c_str() );


    /**    rob.intake.toggeru(master.btnL2);
        rob.indexer.simpleControl(master.btnR1, master.btnR2);
        if(rob.FWVelGoal == 0) {
            //only control the lift (same fw motres) when fw is off
            rob.lift.simpleControl(master.btnLEFT, master.btnRIGHT);
        }
        //if(master.btnUP) rob.testDriveFwds(15);
        //if(master.btnDOWN) rob.testRotation(90);
        if(master.btnX) rob.intro();
        if(master.btnL1) rob.base.fwdsAng(16.5, 400, rob.base.odom.pos.heading);
        //if(master.btnLEFT) rob.testMacro(115, 150);
        /*once all tests are done to satisfaction:
        if(master.btnUP) rob.part1();
        */
        delay(2);
    }
}
