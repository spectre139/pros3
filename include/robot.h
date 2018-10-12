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
#define flywheel1_port   3
#define flywheel2_port   4

#define intake_Port      19
#define indexer_Port     5

#define motorLFront_Port 7
#define motorLRear_Port  8
#define motorRFront_Port 9
#define motorRRear_Port  20

#define BACK true

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
		//drive PID, then angle PID, then curve
		{ PIDcontroller(14.5, 0.0, 0.0, 1.75, 10, true, false), PIDcontroller(1.75, 0.0, 0.0, 2.0,  10, true, false), PIDcontroller(2.5, 0.0, 0.0, 1.0,  10, false, false) },
		//Odometry
		Odometry(Position(0, 0, 0), Position(0, 0, 0)//,//actual position, tracker mech's position
			//Odom Sensors:
			//ADIEncoder(1, 2, true),//left encoder
		//	ADIEncoder(3, 4, false),//right encoder
		//	ADIEncoder(5, 6, true)//middle encoder
		)
	){
		base.odom.pos.X = 0;
		base.odom.pos.Y = 0;
		base.odom.pos.heading = 90;
	}

    class mechanism flywheel, indexer, intake, lift;
    class chassis base;
	float FWVelGoal = 0;
	public://higher level functions

		void indexerAdvance(int amntTicks = 100){//bring indexer ball up once (given number of encoder ticks)
			indexer.moveAmnt(amntTicks, 10);
		}
		//tests
		void testDriveFwds(float numInches){
			base.fwds(numInches, 400);
		}void testRotation(float degrees){
			base.turn(degrees);
		}	void testMacro(float power = 60){//has flywheel running and stuff
			int t = 1;
			while(t < 300){//time based
				indexer.move(power);
				t++;
				delay(1);
			}
			indexer.move(-20);
			/*FWVelGoal = 150;
			delay(500);//wait 1 second while at good velocity
			while(abs((flywheel.velocity/2.0) / 21.0 - FWVelGoal) > 5){//while not near target velocity
				delay(10);
			}
			indexer.moveAmnt(500, 15);
			delay(500);
			FWVelGoal = 0;*/
			return;
		}
		void capFlip(){
			//intake.move(-127);
			//base.fwds(7);
			base.fwds(-3, 10);
			intake.move(127);
			delay(50);
			base.fwds(11, 50, 80);
		}

		//actual skills runs

	void circleTilebase(){
		base.fwds(50, 400);
		base.turn(-90);
		base.fwds(100, 400);
		base.turn(-90);
		base.fwds(50, 400);
		base.turn(-90);
		base.fwds(100, 400);
		base.turn(-90);
		return;
	}
	void skills(){
		//--reset odom? sometimes works... usually dosent... smh
		base.odom.pos.X = 0;
		base.odom.pos.Y = 0;
		//--get ball under cap
		intake.move(-60);//intake preload
		FWVelGoal = 60;//low power flywheel
		base.driveToPoint(0, 37);//drive fwds
		//--flip cap
		capFlip();
		base.driveToPoint(0, 5, BACK);//fwds(-43, 400);
		//indexer.moveAmnt(600, 10);//primes the balls
		base.turn(90, 500);
		indexer.moveAmnt(-400, 10);
		//--start driving to nearest flags
		FWVelGoal = 150;
		base.fwdsAng(60, 400, base.odom.pos.heading);//, 0.5);
		//base.turnTo(175);
		indexer.moveAmnt(500, 10);//primes the balls
		//--time basedd self correction for angle (idk if needed)
		int t = 0;
		while(t<1000){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 180)) * 20);
			t++;
			delay(1);
		}
		//--first ball (high flag)
		indexer.moveAmnt(350, 10);
		//indexer.moveTime(300, 60);
		//--turn to get second ball (medium flag needs special angle)
		base.turn(10, 500);//turn to hit low flag//NEEDS PID
		base.fwdsAng(17, 400, base.odom.pos.heading);//drive closer to flag to hit
		indexer.moveAmnt(350, 20);//shoot ball
		//--time to get the first low flag
		//----first turn to get a smooth curve to ram the low flag
		base.turn(10, 300);
		//--then curve into the ideal location
		base.smoothDriveToPoint(96, 0, 0.5);
		delay(500);
		base.driveToPoint(72, 0, BACK);
		base.turnTo(90, 500);//reset position. can probs ram against fence idk.
		/*
		base.fwds(-10, 200);//drive back to not get stuck on flag
		FWVelGoal = 0;//turn off flywheel
		base.turn(10, 500);//turn to get sharper nice angle
		base.fwds(10, 200);//ram into flag
		*/
		delay(500);
		base.fwds(-20, 400);//finish auton... for now.
	}

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
			ret.push_back(string("EncL: ") + std::to_string( encoderL.get_value())
						 +string("; EncR: ") + std::to_string( encoderR.get_value())
						 +string("; EncM: ") + std::to_string( encoderM.get_value()));
			ret.push_back(string("Pos X: ") + std::to_string( base.odom.pos.X) + string(" Pos Y: ") + std::to_string( base.odom.pos.Y));
			ret.push_back(string("Heading: ") + std::to_string( base.odom.pos.heading));
			//ret.push_back(string("FlywheelPos: ") + std::to_string( flywheelEnc.get_value()));
			ret.push_back(string("FlywheelVel(rpm): ") + std::to_string( flywheel.velocity/2) + string(" Motors: ") + std::to_string( flywheel.getMotorVel() ));
			ret.push_back(string("base Vel: ") + std::to_string( base.driveVel) + string("; rot Vel: ") + std::to_string( base.rotVel));
			ret.push_back(string("Angle Err: ") + std::to_string( base.pid[ANGLE].error));

			//ret.push_back(string("Error Val: ") + strerror(errno));
			return ret;
		}
	};
	#endif
