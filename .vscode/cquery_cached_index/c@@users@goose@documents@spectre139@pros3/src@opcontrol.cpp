#include "main.h"
#include "robot.h"
#include "api.h"

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
//have to add these next lines for EVERY scope per instance of sensor use... ugh ok.


void flywheelControl(void* param){//task test for flywheel PID
    Robot* r = (Robot*) param;
    float goalVel = 0;
    while(true){
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
        rob.base.driveLR(master.rightY, master.leftY);
        //rob.LFrontBase.move(clamp(127,-127, PIDPower));
        //debugs
        int i = 0;
        for(const string& s : rob.debugString()){
            lcd::print(i++, s.c_str());
        }
        rob.intake.simpleControl(master.btnR1, master.btnR2);
        rob.indexer.simpleControl(master.btnUP, master.btnDOWN);
        /*if(master.btnUP) rob.testDriveFwds(15);
        if(master.btnDOWN) rob.testRotation(90);
        if(master.btnRIGHT) rob.testCurve(Position(20, 15, 0), 0.5);
        if(master.btnLEFT) rob.testMacro(115, 150);*/
        /*once all tests are done to satisfaction:
        if(master.btnUP) rob.part1();
        */
        delay(2);
    }
}
