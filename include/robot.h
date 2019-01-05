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
#define flywheel1_port   1
#define flywheel2_port   2

#define intake_Port      19
#define indexer_Port     5

#define motorLFront_Port 7
#define motorLRear_Port  8
#define motorRFront_Port 9
#define motorRRear_Port  20

#define BACK true

using namespace pros;

ADIEncoder FWenc (7, 8, false);
ADIPotentiometer pot (2);
class Robot{
public:
	//CONSTRUCTOR:
	Robot() :
    //mechanisms
    flywheel(
		{Motor(flywheel1_port, E_MOTOR_GEARSET_06), Motor(flywheel2_port, E_MOTOR_GEARSET_06)}, //motors
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
		{ Motor(4), Motor(5), Motor(6), Motor (7),Motor (8), Motor (9),},

		//drive PID, then angle PID, then curve
		{ PIDcontroller(9.5, 0.0, 0.0, 1.75, 10, true, false), PIDcontroller(1.75, 0.0, 0.0, 2.0,  10, true, false), PIDcontroller(2.5, 0.0, 0.0, 1.0,  10, false, false) },
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
			intake.move(-127);
			//base.fwds(7);
			base.fwds(-3, 10);
			delay(550);
			intake.move(127);
			base.fwds(11, 50, 80);
			delay(100);
		}
		void capFlipNoIntake(){
			intake.move(127);
			delay(350);
			base.fwds(10, 50, 80);
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
	void resetOdom(class Odometry* o){
		o->resetEncoders = true;
		o->lastL = o->lastM = o->lastR = 0;
		o->pos.X = 0;
		o->pos.Y = 0;
		o->pos.heading = 90;
	}







	void intro(){
		resetOdom(&base.odom);
		//--reset odom? sometimes works... usually dosent... smh
		base.odom.pos.X = 0;
		base.odom.pos.Y = 0;
		//--get ball under cap
		intake.move(-90);//intake preload
		FWVelGoal = 60;//low power flywheel
		base.driveToPoint(base.odom.pos.X, 37);//drive fwds
		//--flip cap
		capFlip();
		int t = 0;
		while(t < 500){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 90)) * 5);
			delay(1);
			t++;
		}
		base.driveToPoint(0, 4, BACK);//fwds(-43, 400);
		//indexer.moveAmnt(600, 10);//primes the balls
		base.turn(90.7, 400);
		indexer.moveAmnt(-200, 10);
		//--start driving to nearest flags
		intake.move(0);
		FWVelGoal = 150;
		indexer.moveAmnt(300, 10);//primes the balls
		//base.smoothDriveToPoint(-69, 6, 0.7);
		base.fwdsAng(62, 400, 180.3);//, 0.5);
		//base.turnTo(175);
		//--time basedd self correction for angle (idk if needed)
		t = 0;
		while(t < 500){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 179.5)) * 5);
			delay(1);
			t++;
		}
		//base.turnTo(180, 500);
		//--first ball (high flag)
		indexer.moveAmnt(170, 10);
		FWVelGoal = 170;
		//indexer.moveTime(300, 60);
		//--turn to get second ball (medium flag needs special angle)
	//	base.turn(8, 100);//turn to hit low flag//NEEDS PID
		delay(300);
		int t2 = 0;
		while(t2<500){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 171)) * 6);
			t2++;
			delay(1);
		}
		base.fwdsAng(16.5, 400, base.odom.pos.heading);//drive closer to flag to hit
		indexer.moveAmnt(350, 20);//shoot ball
		base.fwds(-8, 300);
		delay(500);
		//base.fwds(-10, 100);
		//--time to get the first low flag
		//----first turn to get a smooth curve to ram the low flag
		base.turn(20, 300);
		intake.move(20);


return;
}







	void skills(){
		resetOdom(&base.odom);
		//--reset odom? sometimes works... usually dosent... smh
		base.odom.pos.X = 0;
		base.odom.pos.Y = 0;
		//--get ball under cap
		intake.move(-90);//intake preload
		FWVelGoal = 60;//low power flywheel
		base.driveToPoint(base.odom.pos.X, 37);//drive fwds
		//--flip cap
		capFlip();
		int t = 0;
		while(t < 500){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 90)) * 5);
			delay(1);
			t++;
		}
		base.driveToPoint(0, 4, BACK);//fwds(-43, 400);
		//indexer.moveAmnt(600, 10);//primes the balls
		base.turn(90.7, 400);
		indexer.moveAmnt(-200, 10);
		//--start driving to nearest flags
		intake.move(0);
		FWVelGoal = 150;
		indexer.moveAmnt(300, 10);//primes the balls
		//base.smoothDriveToPoint(-69, 6, 0.7);
		base.fwdsAng(62, 400, 180.3);//, 0.5);
		//base.turnTo(175);
		//--time basedd self correction for angle (idk if needed)
		t = 0;
		while(t < 500){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 179.5)) * 5);
			delay(1);
			t++;
		}
		//base.turnTo(180, 500);
		//--first ball (high flag)
		indexer.moveAmnt(170, 10);
		FWVelGoal = 170;
		//indexer.moveTime(300, 60);
		//--turn to get second ball (medium flag needs special angle)
	//	base.turn(8, 100);//turn to hit low flag//NEEDS PID
		delay(300);
		int t2 = 0;
		while(t2<500){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 171)) * 6);
			t2++;
			delay(1);
		}
		base.fwdsAng(16.5, 400, base.odom.pos.heading);//drive closer to flag to hit
		indexer.moveAmnt(350, 20);//shoot ball
		base.fwds(-8, 300);
		delay(500);
		//base.fwds(-10, 100);
		//--time to get the first low flag
		//----first turn to get a smooth curve to ram the low flag
		base.turn(20, 300);
		intake.move(20);
		FWVelGoal = 50;
		//--then curve into the ideal location
		base.smoothDriveToPointTIME(-93, 0, 0.6, 1300);
		//base.fwds(10, 0);//(-93, 3);
		//base.fwds(, 100);
	/*	t = 0;
		while(t < 250){
			base.fwdsDrive(100);
			t++;
			delay(1);
		}*/

		delay(500);
		int t3 = 0;
		while(t3<500){
			base.pointTurn(-sign(normAngle(base.odom.pos.heading - 180)) * 8);
			t3++;
			delay(1);
		}
		base.driveToPointTIME(-54, 5, 2000, BACK);
		base.fwds(0, 0);
		delay(500);
		base.turnToKP(90, 1.5, 1500);//reset position. can probs ram against fence idk.
		delay(200);
		intake.move(-60);//intake preload
		FWVelGoal = 60;//low power flywheel
		base.fwds(40, 200);//, 200);//drive fwds

		//--flip cap
		capFlip();
		base.fwds(-11.5, 200);
		//base.fwds(10, 100);
		base.turn(90, 400);//aim at the flags
		//hit high flag
		t = 0;
		while(t < 400){
			base.fwdsDrive(-80);
			delay(1);
			t++;
		}//--HARD RESETs
		//resetOdom(&base.odom);
		intake.move(127);
		base.smoothDriveToPoint(base.odom.pos.X - 24, base.odom.pos.Y - 15, 0.75);
		base.fwds(4, 100, 70);
		//capFlip();
		//--drive to hit low flag 2
		intake.move(0);
		base.fwds(-16, 300);
		//base.turn(-120);
		base.driveToPointTIME(-90, 45, 1000);
		//base.fwds(0, 0);
		delay(400);
		FWVelGoal = 150;
		base.driveToPointTIME(-60, 47, 2000, BACK);
		base.turnTo(90+80, 300);
		indexer.moveAmnt(600, 10);
		FWVelGoal = 50;//low power
		intake.move(127);
		base.smoothDriveToPointTIME(base.odom.pos.X+13, 100, 0.95, 3000);
		delay(500);
		base.fwds(-15, 100);
		intake.move(0);
		base.driveToPointTIME(-90, 100, 1500);
		delay(300);
		base.driveToPoint(-8, 97);
		/*
		base.fwds(-48, 300);
		base.turnTo(90, 400);
		base.fwds(13, 300);
		base.turnTo(0, 400);
		base.fwds(48, 400);*/
		return;




		FWVelGoal = 150;//high power (soon gonna hit high flag pow)
		indexer.moveAmnt(650, 10);//shoot ball
		//--ram into low flag
		base.fwds(40, 200);
		delay(300);
		base.fwds(-20, 200);
		//--time to get that other cap
		base.turnTo(179, 500);
		intake.move(-60);//intake preload
		FWVelGoal = 60;//low power flywheel
		base.fwds(27, 200);//drive fwds
		//--flip cap
		capFlip();
		base.fwds(-10, 100);
		base.turnTo(0, 200);
		return;//done for now.
		/*
		base.fwds(-10, 200);//drive back to not get stuck on flag
		FWVelGoal = 0;//turn off flywheel
		base.turn(10, 500);//turn to get sharper nice angle
		base.fwds(10, 200);//ram into flag
		*/
	//	base.fwds(-20, 400);//finish auton... for now.
}
void skillsPARK(){
	base.fwds(-10, 100);
	base.driveToPointTIME(20, -74, 3000, BACK);
	base.turnTo(178.5, 800);
	int t = 0;
	while(t < 3000){
		base.fwdsDrive(127);
		t++;
		delay(1);
	}
	t = 0;
	while(t<50){
		base.fwdsDrive(-127);
		t++;
		delay(1);
	}
	base.fwdsDrive(0);
	return;
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
