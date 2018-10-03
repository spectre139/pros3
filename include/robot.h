#ifndef ROBOT_H
#define ROBOT_H
#include "api.h"
#include "util.h"
#include <string>
#include <vector>

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
#define motorLFront_Port 1
#define motorLRear_Port  2
#define motorRFront_Port 3
#define motorRRear_Port  4

#define sprocket1_Port   5
#define sprocket2_Port   6

#define indexer_Port     7

class Position {
public:
	Position(float x, float y, float head) : X(x), Y(y), heading(head) {}
	float X, Y, heading;
};
class Odometry {
public:
	Odometry() : centralAvg(0), lastL(0), lastR(0), lastM(0) {}//init constructor defaulted
	float centralAvg;
	float lastL, lastR, lastM;
};
class vec3 {
public:
	vec3(float x = 0.0, float y = 0.0, float z = 0.0) : X(x), Y(y), Z(z) {}
	float X, Y, Z;
	vec3 times(float f) { return vec3(X*f, Y*f, Z*f); }
	float distance(vec3 v) { return sqrt(sqr(v.X - X) + sqr(v.Y - Y)); }
	float distanceV3(vec3 v) {	return sqrt(sqr(v.X - X) + sqr(v.Y - Y) + sqr(v.Z - Z)); }
	vec3 operator+(vec3 v) { return vec3(X + v.X, Y + v.Y, Z + v.Z); }
};
class PIDcontroller {
public:
        PIDcontroller(float p, float i, float d, float t, float dT, bool rev, bool run) : kP(p), kI(i), kD(d), thresh(t), delayThresh(dT), isReversed(rev), isRunning(run), LastError(0), Integral(0), Derivative(0), goal(0) {}
private:
	float kP, kI, kD;
    float error;
	bool isReversed, isRunning;
	float Integral, Derivative, LastError;
	float thresh, delayThresh, goal;
public:
	float pidCompute(float current) {
        if(isRunning){
	    		error = current - goal;//calculate error
	    		int dir = 1;
	    		float power = 0;
	    		if (isReversed) dir = -1;
	    		const float untilIntegral = thresh * 7;//considered "low threshold"
	    		// calculate the integral
	    		if (kI != 0.0) {//calculates integral (only at very end)
	    			if (fabs(error) < untilIntegral) Integral += error;//used for averaging the integral amount, later in motor power divided by 25
	    			else Integral = 0.0;
	    			power += kI * limUpTo(50, Integral);
	    		}
	    		else Integral = 0.0;
	    		// calculate the derivative
	    		if (kD != 0.0) {
	    			Derivative = error - LastError;//change in errors
	    			LastError = error;
	    		}
	    		else Derivative = 0.0;
	    		power += kD * Derivative;
	    		//final proportional output
	    		power += kP * error;
	    		return dir * power;
        }
				return 0;
	}
	float pidComputeAngle(float currentAngle) {//assuming everything is in ANGLES
        if(isRunning){
	    		error = normAngle(currentAngle - goal);//calculate error
	    		int dir = 1;
	    		float power = 0;
	    		if (isReversed) dir = -1;
	    		const float untilIntegral = thresh * 7;//considered "low threshold"
	    		// calculate the integral
	    		if (kI != 0.0) {//calculates integral (only at very end)
	    			if (fabs(error) < untilIntegral) Integral += error;//used for averaging the integral amount, later in motor power divided by 25
	    			else Integral = 0.0;
	    			power += kI * limUpTo(50, Integral);
	    		}
	    		else Integral = 0.0;
	    		// calculate the derivative
	    		if (kD != 0.0) {
	    			Derivative = error - LastError;//change in errors
	    			LastError = error;
	    		}
	    		else Derivative = 0.0;
	    		power += kD * Derivative;
	    		//final proportional output
	    		power += kP * error;
	    		return dir * power;
        }
				return 0;
	}
    void setGoal(float val){
        goal = val;
    }float getGoal(){
        return goal;
    }void setRunningState(bool state){
        isRunning = state;
    }bool getRunningState(){
        return isRunning ;
    }
};
class Robot{
public:
    //CONSTRUCTOR:
	Robot( Position primaryPos, Position trackers, Odometry o, PIDcontroller a, PIDcontroller d, PIDcontroller fw, PIDcontroller iPID, float ww) :
    pos(primaryPos), t_pos(trackers), odom(o), anglePID(a), distPID(d), flyWheelVelPID(fw), indexerPID(iPID), wheelWidth(ww),
    //motors:
    LFrontBase(motorLFront_Port), LRearBase(motorLRear_Port), RFrontBase(motorRFront_Port), RRearBase (motorRRear_Port),
    sprocket1 (sprocket1_Port), sprocket2 (sprocket2_Port), indexer(indexer_Port),
    //sensors:
    encoderL (encoderLTop_Port, encoderLBott_Port, true),
		encoderR (encoderRTop_Port, encoderRBott_Port, false),
		encoderM (encoderMTop_Port, encoderMBott_Port, true),
		flywheelEnc (encoderFlywheelTop, encoderFlywheelBott, false)
    {
			encoderL.reset();
			encoderR.reset();
			encoderM.reset();
			flywheelEnc.reset();
			
		}
	class Position pos, t_pos;//tpos being the position of the trackers, not robot
	class Odometry odom;
	class PIDcontroller anglePID, distPID, flyWheelVelPID, indexerPID;
	float wheelWidth;//distance b/w L & R tracker wheels (inches)
	float lastFlywheelVel = 0;
	float flywheelVel = 0;
    //motors:
    Motor LFrontBase, LRearBase, RFrontBase, RRearBase,
          sprocket1, sprocket2, indexer;
    //sensors:
    ADIEncoder encoderL, encoderR, encoderM, flywheelEnc;
public:
    void driveLR(int speedL, int speedR){//low level
        speedL = clamp(127, -127, speedL);
        speedR = clamp(127, -127, speedR);
        RFrontBase.move(speedR);
        RRearBase.move(speedR);
        LFrontBase.move(speedL);
        LRearBase.move(speedL);
    }
    void fwds(int total){//BASE
        driveLR(total, total);
    }
    void pointTurn(int speed){//turn
        driveLR(speed, -speed);
    }
    void smoothDrive(int speed, const float angle, float sharpness = 1) {//drive base forwards
    	const float scalar = 5;//scalar for rotation
    	sharpness += 1;//parameter is from 0-1, do this addition to make sure it ranges from (1-2) [as explained below]
        speed = clamp(127, -127, speed);
    	//for sharpness: 2 is direct point turn, 1 is turning off one side...
    	//	it basically is just how much the different sides can be reversed to increase tha sharpness of the curve
    	float dirSkew = limUpTo(127 * sharpness, scalar*normAngle(pos.heading - angle));
        driveLR(speed + dirSkew, speed - dirSkew);
    }

    void flywheelControl(int power){
        sprocket1.move(clamp(127, -127, power));
        sprocket2.move(clamp(127, -127, power));
    }
		void fwVelTo(int vel){
			//positive velocity will power the lift, we dont like...
				sprocket1.move_velocity(clamp(200, -200, -vel));
				sprocket2.move_velocity(clamp(200, -200, -vel));
		}
    float getFlywheelVel(float delayAmnt){
			float velocity = ( flywheelEnc.get_value() - lastFlywheelVel) / (delayAmnt / 1000.0);
			lastFlywheelVel = flywheelEnc.get_value();
			return velocity / 2.0; //(converting ticks/sec to rot/min)
    }
    void indexerControl(int power){
        indexer.move(clamp(127, -127, power));
    }
		void ploomp(int amntTicks = 100){//bring indexer ball up once (given number of encoder ticks)
			int startingPos = indexer.get_position();
			while(fabs(indexer.get_position() - startingPos) < amntTicks){
				indexerControl(127);
			}
			indexerControl(-127);
			pros::delay(10);
			indexerControl(0);
		}
    std::vector<string> debugString(){
        std::vector<string> ret;
				ret.push_back(string("BATTERY percent:") + std::to_string( pros::battery::get_capacity()));
				/*ret.push_back(string("Flywheel1 Vel:") + std::to_string(sprocket1.get_actual_velocity()));
				ret.push_back(string("Flywheel1 Temp:") + std::to_string(sprocket1.get_temperature()));
        ret.push_back(string("Flywheel2 Vel:") + std::to_string(sprocket2.get_actual_velocity()));
				ret.push_back(string("Flywheel2 Temp:") + std::to_string(sprocket2.get_temperature()));
				if(flyWheelVelPID.getRunningState()) ret.push_back(string("PID Running: YES"));
        else ret.push_back(string("PID Running: NO"));
        ret.push_back(string("PID Goal:") + std::to_string(flyWheelVelPID.getGoal()));
				*/
				ret.push_back(string("EncoderL: ") + std::to_string( encoderL.get_value()));
				ret.push_back(string("EncoderR: ") + std::to_string( encoderR.get_value()));
				ret.push_back(string("EncoderM: ") + std::to_string( encoderM.get_value()));
				ret.push_back(string("Pos X: ") + std::to_string( pos.X));
				ret.push_back(string("Pos Y: ") + std::to_string( pos.Y));
				ret.push_back(string("Heading: ") + std::to_string( pos.heading));
				//ret.push_back(string("FlywheelPos: ") + std::to_string( flywheelEnc.get_value()));
				ret.push_back(string("FlywheelVel(rpm): ") + std::to_string( round(flywheelVel)) + string(" Motors: ") + std::to_string( round(avg(sprocket1.get_actual_velocity(), sprocket2.get_actual_velocity()) )));
				return ret;
    }
};
//robot functions


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
