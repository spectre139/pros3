#include "main.h"
#include "lowLevel.h"
#include "api.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
//have to add these next lines for EVERY scope per instance of sensor use... ugh ok.

void flywheelControl(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    while(true){
        if(master.btnUP){
            r->FWVelGoal = 150;//150rpm for high/far flags}
          }
        else if(master.btnDOWN){
            r->FWVelGoal = 90;//115rpm for medium/close flags
        }
        else if(master.btnA){
            r->FWVelGoal = 60;//60rpm for low power use
        }
        else if(master.btnB){
            r->FWVelGoal = 0;//0rpm for off flywheel
        }
        r->flywheel.moveVel(r->FWVelGoal);
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
        int i = 0;
        for(const string& s : r->debugString()){
            lcd::print(i++, s.c_str());
        }
        delay(delayAmnt);
    }
}
Robot rob = Robot();
void opcontrol() {
    Task odometryCalculations(calculatePos, &rob.base.odom);
    Task controlFlywheel(flywheelControl, &rob);
    Task sensorUpdates(updateSensor, &rob);
    Task pidUpdates(updatePIDs, &rob);
    ADIEncoder encoderL (1, 2, true), encoderR (3, 4, true), encoderM (5, 6, false);
    ADIEncoder FWenc (7, 8, false);
    rob.base.odom.pos.X = 0;
    rob.base.odom.pos.Y = 0;
    while (true) {
      rob.base.driveLR(master.rightY, master.leftY);
        //rob.LFrontBase.move(clamp(127,-127, PIDPower));
        //debugs
        //lcd::print(1, (string("EncoderL6: ") + std::to_string( encoderR.get_value())).c_str() );
        //lcd::print(2, (string("EncoderR6: ") + std::to_string( encoderL.get_value())).c_str() );
        //lcd::print(3, (string("EncoderM6: ") + std::to_string( encoderM.get_value())).c_str() );


        rob.intake.toggeru(master.btnL2);
        rob.indexer.simpleControl(master.btnR1, master.btnR2);
        if(rob.FWVelGoal == 0) {
            //only control the lift (same fw motres) when fw is off
            rob.lift.simpleControl(master.btnLEFT, master.btnRIGHT);
        }
        //if(master.btnUP) rob.testDriveFwds(15);
        //if(master.btnDOWN) rob.testRotation(90);
        if(master.btnX) rob.skillsPARK();//base.smoothDriveToPoint(10, 20, 0.5);
        if(master.btnY) rob.base.fwdsAng(16.5, 400, rob.base.odom.pos.heading);
        //if(master.btnLEFT) rob.testMacro(115, 150);
        /*once all tests are done to satisfaction:
        if(master.btnUP) rob.part1();
        */
        delay(2);
    }
}
