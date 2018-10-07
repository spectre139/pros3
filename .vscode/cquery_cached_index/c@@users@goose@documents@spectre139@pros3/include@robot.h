#ifndef ROBOT_H
#define ROBOT_H
#include "api.h"
#include "util.h"
#include <string>
#include <vector>
#include <cerrno>
#include "lowLevel.h"

using pros::Motor, pros::ADIEncoder, std::string;

#define encoderLTop_Port  1
#define encoderLBott_Port 2

#define encoderRTop_Port  3
#define encoderRBott_Port 4

#define encoderMTop_Port  5
#define encoderMBott_Port 6

#define encoderFlywheelTop  7
#define encoderFlywheelBott 8

//defining motor ports:
#define motorLFront_Port 7
#define motorLRear_Port  8
#define motorRFront_Port 9
#define motorRRear_Port  10

#define flywheel1_port   1
#define flywheel2_port   2

#define intake_Port      4
#define indexer_Port     5

using namespace pros;

ADIEncoder FWenc (7, 8, false);
class Robot{
public:
	//CONSTRUCTOR:
	Robot() :
    //mechanisms
    flywheel(
		{Motor(flywheel1_port), Motor(flywheel2_port)}, //motors
		{FWenc}, //encoder
		PIDcontroller(1.5, 0.0, 0.0, 1.50, 10, false, true)//PID
	),
    indexer(
		{ Motor(indexer_Port) }, //motors
		{},//no sensors for indexer, thus use indexer motor
		PIDcontroller(2.0, 0.0, 0.0, 10,  10, true, true)//PID
	),
		intake(
		{ Motor(intake_Port) }, //motors
		{},//no sensors for intake, thus use indexer motor
		PIDcontroller(2.0, 0.0, 0.0, 10,  10, true, true)//PID
	),
		lift(
		{ Motor(flywheel1_port), Motor(flywheel2_port) }, //motors
		{},//no sensors for lift, thus use indexer motor
		PIDcontroller(2.0, 0.0, 0.0, 10,  10, true, true)//PID
	),
    base(//motors
		{ Motor(motorRFront_Port), Motor(motorRRear_Port), Motor(motorLFront_Port), Motor (motorLRear_Port) },
		//drive PID, then angle PID
		{ PIDcontroller(0.015, 0.0, 0.0, 0.75, 10, true, false), PIDcontroller(1.7, 0.0, 0.0, 1.0,  10, true, false) },
		//Odometry
		Odometry(Position(0, 0, 0), Position(0, -10, 0)//,//actual position, tracker mech's position
			//Odom Sensors:
			//ADIEncoder(1, 2, true),//left encoder
		//	ADIEncoder(3, 4, false),//right encoder
		//	ADIEncoder(5, 6, true)//middle encoder
		)
	){}

    class mechanism flywheel, indexer, intake, lift;
    class chassis base;
public://higher level functions

	void indexerAdvance(int amntTicks = 100){//bring indexer ball up once (given number of encoder ticks)
		indexer.moveAmnt(amntTicks, 10);
	}
	//tests
	void testDriveFwds(float numInches){
		base.fwds(numInches);
	}void testRotation(float degrees){
		base.turn(degrees);
	}void testCurve(class Position g, float sharpness){
		base.smoothDriveToPoint(g, sharpness);
	}void testMacro(float firstVel, float secondVel){
		const float fwGR = 1.0/21.0;
		flywheel.moveVel(firstVel);
		while(abs(flywheel.velocity * fwGR - firstVel) > 5){//while not near target velocity
			delay(10);
		}
		delay(1000);//wait 1 second while at good velocity
		flywheel.moveVel(secondVel);
		while(abs(flywheel.velocity * fwGR - secondVel) > 5){//while not near target velocity
			delay(10);
		}
		delay(1000);//wait 1 second while at good velocity
		flywheel.moveVel(0);
		return;
	}
	//actual skills runs




	std::vector<string> debugString(){
		std::vector<string> ret;
		//ret.push_back(string("BATTERY percent:") + std::to_string( pros::battery::get_capacity()));
		/*ret.push_back(string("Flywheel1 Vel:") + std::to_string(sprocket1.get_actual_velocity()));
		ret.push_back(string("Flywheel1 Temp:") + std::to_string(sprocket1.get_temperature()));
		ret.push_back(string("Flywheel2 Vel:") + std::to_string(sprocket2.get_actual_velocity()));
		ret.push_back(string("Flywheel2 Temp:") + std::to_string(sprocket2.get_temperature()));
		if(flyWheelVelPID.getRunningState()) ret.push_back(string("PID Running: YES"));
		else ret.push_back(string("PID Running: NO"));
		ret.pus h_back(string("PID Goal:") + std::to_string(flyWheelVelPID.getGoal()));
		*/
		ret.push_back(string("EncoderL2: ") + std::to_string( encoderL.get_value()));
		ret.push_back(string("EncoderR2: ") + std::to_string( encoderR.get_value()));
		ret.push_back(string("EncoderM2: ") + std::to_string( encoderM.get_value()));
		ret.push_back(string("Pos X: ") + std::to_string( base.odom.pos.X) + string(" Pos Y: ") + std::to_string( base.odom.pos.Y));
		ret.push_back(string("Heading: ") + std::to_string( base.odom.pos.heading));
		//ret.push_back(string("FlywheelPos: ") + std::to_string( flywheelEnc.get_value()));
		ret.push_back(string("FlywheelVel(rpm): ") + std::to_string( flywheel.velocity) + string(" Motors: ") + std::to_string( round(flywheel.getMotorVel()) ));
		ret.push_back(string("Error Val: ") + strerror(errno));
		return ret;
	}
};

/*void fwdsDrive(int speed, const float angle = robot.pos.heading) {//drive base forwards
const float scalar = 5;//scalar for rotation
float dirSkew = limUpTo(30, scalar*normAngle(rob.pos.heading - angle));
speed = clamp(127, -127, speed);
skewTurnRL(speed - dirSkew, speed + dirSkew);
}
void curveDrive(int speed, const float angle = robot.pos.heading) {//drive base forwards
const float sharpness = 0.05;//GET RID OF THIS, MAKE IT A PID PARAMETER, wont need fwds
speed = clamp(127, -127, speed);
if(angle > normAngle(robot.pos.heading))
skewTurnRL(speed, speed*sharpness);
else skewTurnRL(speed*sharpness, speed);
}*/


#endif
