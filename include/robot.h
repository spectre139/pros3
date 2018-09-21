#if !defined(ROBOT_H)
#define ROBOT_H

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
	Robot( Position main, Position trackers, Odometry o, PIDs angle, PIDs dist, float ww) : pos(main), t_pos(trackers), odom(o), anglePID(angle), distPID(dist), wheelWidth(ww) {}
	Position pos, t_pos;//tpos being the position of the trackers, not robot
	Odometry odom;
	PIDs anglePID;
	PIDs distPID;
	float wheelWidth;//distance b/w L & R tracker wheels (inches)

    void skewTurnRL(int speedR, int speedL){
        speedR = clamp(127, -127, speedR);
        speedL = clamp(127, -127, speedL);
        //RFrontBase.move(speedR);
    //    RRearBase.move(speedR);//FIX THIS
    //    LFrontBase.move(speedL);
    //    LRearBase.move(speedL);
    }
    void pointTurn(int speed){
        skewTurnRL(-speed, speed);
    }
    void steadyDrive(int speed, const float angle, float sharpness = 1) {//drive base forwards
    	const float scalar = 5;//scalar for rotation
    	sharpness += 1;//parameter is from 0-1, do this addition to make sure it ranges from (1-2) [as explained below]
        speed = clamp(127, -127, speed);
    	//for sharpness: 2 is direct point turn, 1 is turning off one side...
    	//	it basically is just how much the different sides can be reversed to increase tha sharpness of the curve
    	float dirSkew = limUpTo(127 * sharpness, scalar*normAngle(pos.heading - angle));
        skewTurnRL(speed - dirSkew, speed + dirSkew);
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
