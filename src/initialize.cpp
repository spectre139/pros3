#include "main.h"
#include <string>
using namespace pros;

class Position {
public:
	Position(float x, float y, float head) : X(x), Y(y), heading(head) {}
	float X, Y, heading;
};
class Odometry {
public:
	Odometry() : centralAvg(0), lastL(0), lastR(0), lastM(0) {}
	float centralAvg;
	float lastL, lastR, lastM;
};
class vec3 {
public:
	vec3(double x = 0.0, double y = 0.0, double z = 0.0) : X(x), Y(y), Z(z) {}
	double X, Y, Z;
	vec3 times(double f) { return vec3(X*f, Y*f, Z*f); }
	double distance(vec3 v) { return sqrt(sqr(v.X - X) + sqr(v.Y - Y)); }
	double distanceV3(vec3 v) {	return sqrt(sqr(v.X - X) + sqr(v.Y - Y) + sqr(v.Z - Z)); }
	vec3 operator+(vec3 v) { return vec3(X + v.X, Y + v.Y, Z + v.Z); }
};

class PIDs {
public:
	PIDs(float p, float i, float d, float t, float dT, bool rev, bool run) : kP(p), kI(i), kD(d), thresh(t), delayThresh(dT), isReversed(rev), isRunning(run), LastError(0), Integral(0), Derivative(0) {}
	float kP, kI, kD;
	bool isReversed, isRunning, closeEnough;
	float Integral, Derivative, LastError;
	float thresh, delayThresh, goal;
	float pidCompute(float current) {
		float error = current - goal;//calculate error
		int dir = 1;
		float power = 0;
		if (isReversed) dir = -1;
		const float untilIntegral = thresh * 7;//considered "low threshold"
		// calculate the integral
		if (kI != 0.0) {//calculates integral (only at very end)
			if (abs(error) < untilIntegral) Integral += error;//used for averaging the integral amount, later in motor power divided by 25
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
	float pidComputeAngle(float currentAngle) {//assuming everything is in ANGLES
		float error = normAngle(currentAngle - goal);//calculate error
		int dir = 1;
		float power = 0;
		if (isReversed) dir = -1;
		const float untilIntegral = thresh * 7;//considered "low threshold"
		// calculate the integral
		if (kI != 0.0) {//calculates integral (only at very end)
			if (abs(error) < untilIntegral) Integral += error;//used for averaging the integral amount, later in motor power divided by 25
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
};
class Robot{
public:
	Robot( Position main, Position trackers, Odometry o, PIDs angle, PIDs dist, float ww) : main_pos(main), tracker_pos(trackers), odom(o), anglePID(angle), distPID(dist), wheelWidth(ww) {}
	Position main_pos, tracker_pos;//tpos being the position of the trackers, not robot
	Odometry odom;
	PIDs anglePID;
	PIDs distPID;
	float wheelWidth;//distance b/w L & R tracker wheels
};
Robot robot(
	Position(0, 0, 0),
	Position(0, -10, 0),//distance to trackers
	Odometry(),
	PIDs(1.1, 0.3, 0.0, 3.0, 100, false, true),
	PIDs(9.5, 0.1, 0.0, 0.75, 100, true, true),
	8.35
);
Position realPos(0, 0, 0);


void initSensors(){
	ADIEncoder encoderL (encoderLTop_Port, encoderLTop_Port, false);
	ADIEncoder encoderR (encoderRTop_Port, encoderRTop_Port, false);
	ADIEncoder encoderM (encoderMTop_Port, encoderMTop_Port, false);
	encoderL.reset();
	encoderR.reset();
	encoderM.reset();
 }
 void initMotors(){
	 Motor LFrontBase_initializer (motorLFront_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	 Motor LRearBase_initializer (motorLRear_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	 Motor RFrontBase_initializer (motorRFront_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	 Motor RRearBase_initializer (motorRRear_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	 //sporkets
	 Motor sprocket1_initializer (sprocket1_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	 Motor sprocket2_initializer (sprocket2_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	 //indexer
	 Motor indexer_initializer (indexer_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
 }
void initialize() {
	lcd::initialize();
	lcd::set_text(1, "Welcome 139A Gods");
	initSensors();
	initMotors();


}

void disabled() {}


void competition_initialize() {}
