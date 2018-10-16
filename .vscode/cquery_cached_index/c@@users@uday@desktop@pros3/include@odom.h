#ifndef ODOM_H
#define ODOM_H

void calculatePos(void* param);


/*
void calculatePos(void* param){
	for(;;){

		float dR = encoderDistInch(SensorValue[Right]) - robot.odom.lastR;//change in right encoder
		float dL = encoderDistInch(SensorValue[Left])  - robot.odom.lastL;//change in left
		float dM = encoderDistInch(SensorValue[Midd])  - robot.odom.lastM;//change in middle

		robot.odom.lastR += dR;//updates "last" values
		robot.odom.lastL += dL;//updates "last" values
		robot.odom.lastM += dM;//updates "last" values

		float dCentral = avg(dR, dL);//average of deltas
		float dHeading = toDeg(dR - dL) / robot.wheelWidth;//change in angle
		float avgHeading = normAngle(robot.pos.heading + dHeading / 2.0);  // Angle robot is assumed to have been facing when moving dS.

		float radHeading = boundAngle(toRad(avgHeading));
		// Update current robot position.
		robot.Tpos.heading += dHeading;
		robot.Tpos.X += dCentral * cos(radHeading) + dM * sin(radHeading);
		robot.Tpos.Y += dCentral * sin(radHeading) - dM * cos(radHeading);
		//add little vector after calculating H mech's position
		const float distToCenter = 2.18524;//inches the center of the H mech to the center of robot's rotation
		robot.pos.heading = normAngle(robot.Tpos.heading + 90);
		robot.pos.X = robot.Tpos.X + distToCenter * cos(radHeading);
		robot.pos.Y = robot.Tpos.Y + distToCenter * sin(radHeading) - distToCenter;
		//little delays
        delay(1);
	}
}
//ANGLE PIDS
void checkAngleCloseness(void* param){
	for(;;){
		if(withinAngleBounds(robot.pos.heading, robot.anglePID.goal, robot.anglePID.thresh)){
			clearTimer(T1);
			bool currentlyWithinBounds = true;
			while(time1[T1] < robot.anglePID.delayThresh){//checking for continuous withinBounds() (else instantanious refresh)
				currentlyWithinBounds = withinAngleBounds(robot.pos.heading, robot.anglePID.goal, robot.anglePID.thresh);//still isStalling
				if(!currentlyWithinBounds){
					robot.anglePID.closeEnough = false;//if fails, leave loop
					break;
				}
				//keep going until time limit
			}
			if(currentlyWithinBounds){//done waiting, final check
				robot.anglePID.closeEnough = true;//if so, consider it good enough
			}
		}
		else robot.anglePID.closeEnough = false;
		delay(50);
	}
}
void udayAngleMethod(void* param){
	//const int minPowerThresh = 20;
	for(;;){
		if(withinAngleBounds(robot.pos.heading, robot.anglePID.goal, robot.anglePID.thresh/2.0) ){
			 //|| abs(avg(motor[L_BaseF], motor[R_BaseF])) < minPowerThresh ){
			clearTimer(T1);
			while(time1[T1] < robot.anglePID.delayThresh * 6.5){continue;}
			robot.anglePID.closeEnough = true;
			return;
		}
	}
}
void turnTo(const float degrees){//assumed CW is true
	robot.anglePID.goal = degrees;
	startTask(udayAngleMethod);//300ms delay after first success
	robot.anglePID.kP = limUpTo(15, 97.0449 * pow(abs(normAngle(degrees - robot.pos.heading)), -1.29993) + 0.993483);
	//battery scaling
	robot.anglePID.kP *= 7387.0/(nImmediateBatteryLevel + 1);
	while(!robot.anglePID.closeEnough){
		pointTurn(pidComputeAngle(&robot.anglePID, robot.pos.heading));
		delay(10);
	}
	robot.anglePID.closeEnough = false;
	stopTask(checkAngleCloseness);
	//final check and correction
	const int minSpeed = 50;//slow speed for robot's slight correction
	while(abs(normAngle(robot.pos.heading - robot.anglePID.goal)) > robot.anglePID.thresh){
		pointTurn(sign(normAngle(robot.pos.heading - robot.anglePID.goal)) * minSpeed);
	}
	pointTurn(0);
	return;
}
void turn(int degrees){
	turnTo(normAngle(robot.pos.heading + degrees));
	//basically to turn a set amnt, not to turn TO an angle
}
//FWDS PIDS
void checkDistCloseness(void* param){
	const int initEncLeft = SensorValue[Left];
	const int initEncRight = SensorValue[Right];
	for(;;){
		volatile float currentDist = encoderDistInch( avg(SensorValue[Right] - initEncRight, SensorValue[Left] - initEncLeft));
		if(withinBounds(currentDist, robot.distPID.goal, robot.distPID.thresh)){
			clearTimer(T1);
			bool currentlyWithinBounds = true;
			while(time1[T1] < robot.distPID.delayThresh){//checking for continuous withinBounds() (else instantanious refresh)
				currentDist = encoderDistInch( avg(SensorValue[Right] - initEncRight, SensorValue[Left] - initEncLeft));
				currentlyWithinBounds = withinBounds(currentDist, robot.distPID.goal, robot.distPID.thresh);//still isStalling
				if(!currentlyWithinBounds) {
					robot.distPID.closeEnough = false;//if fails, leave loop
					break;
				}
				//keep going until time limit
			}
			if(currentlyWithinBounds){//done waiting, final check
				robot.distPID.closeEnough = true;//if so, consider it good enough
			}
		}
		else robot.distPID.closeEnough = false;
		delay(50);
	}
}
void udayDistMethod(void* param){
	const int initEncLeft = SensorValue[Left];
	const int initEncRight = SensorValue[Right];
	for(;;){
		volatile float currentDist = encoderDistInch( avg(SensorValue[Right] - initEncRight, SensorValue[Left] - initEncLeft));
		if(withinBounds(currentDist, robot.distPID.goal, 2*robot.distPID.thresh)){
			clearTimer(T1);
			while(time1[T1] < robot.anglePID.delayThresh * 4){continue;}
			robot.distPID.closeEnough = true;
			return;
		}
	}
}
void fwds(int amnt, const float angle = robot.pos.heading){//assumed CW is true
	const int initEncRight = SensorValue[Right];
	const int initEncLeft = SensorValue[Left];
	robot.distPID.goal = amnt;
	startTask(udayDistMethod);
	robot.distPID.kP = limUpTo(20, 28.0449 * pow(abs(amnt), -0.916209) + 2.05938);
	//battery scaling
	robot.distPID.kP *= 7683.0/(nImmediateBatteryLevel + 1);
	volatile float currentDist = 0.0;
	while( !robot.distPID.closeEnough ){
		currentDist = encoderDistInch( avg(SensorValue[Right] - initEncRight, SensorValue[Left] - initEncLeft));
		fwdsDrive(pidCompute(&robot.distPID, currentDist), angle);
		delay(10);
	}
	robot.distPID.closeEnough = false;
	stopTask(udayDistMethod);
	//final check and correction
	const int minSpeed = 40;//slow speed for robot's slight correction
	while(abs(currentDist - robot.distPID.goal) > robot.distPID.thresh){
		currentDist = encoderDistInch( avg(SensorValue[Right] - initEncRight, SensorValue[Left] - initEncLeft));
		fwdsDrive(-sign(currentDist - robot.distPID.goal) * minSpeed);
	}
	fwdsDrive(0);
	return;
}

//driveToPoint()
void driveToPoint(float x, float y){
	//first compute angle to goal
	//also divide by 0 is fine bc atan2 has error handling
	float phii = normAngle(toDeg(atan2((y - robot.pos.Y),(x - robot.pos.X))));
	//then compute distance to goal
	float dist = sqrt(sqr(y - robot.pos.Y) + sqr(x - robot.pos.X));
	turnTo(phii);
	fwds(dist, phii);
	return;
}
void curveToPoint(float x, float y){
	//first compute angle to goal
	float phi = normAngle(toDeg(atan2((y - robot.pos.Y),(x - robot.pos.X))));
	//then compute distance to goal
	//curve PID thing
	//fisrts curve to the goal
	while(abs(normAngle(robot.pos.heading - phi)) > 2){
		curveDrive(100, phi);
	}
	curveDrive(0);
	//then drive straight
	fwds(sqrt(sqr(y - robot.pos.Y) + sqr(x - robot.pos.X)), phi);
	return;
}
static void steadyDriveToPoint(float x, float y, float sharpness = 1){
	struct Position goal;
	goal.X = x;
	goal.Y = y;
	float error = distanceToPoint(goal, robot.pos);
	robot.distPID.goal = 0;//goal is to have no distance between goal and current
	robot.distPID.kP = limUpTo(20, 28.0449 * pow(abs(error), -0.916209) + 2.05938);
	//battery scaling
	robot.distPID.kP *= 7683.0/(nImmediateBatteryLevel + 1);

	while(error > 3){//kinda bad... can be retuned n stuff
		error = distanceToPoint(goal, robot.pos);
		//first compute angle to goal
		float angle = normAngle(toDeg(atan2((goal.Y - robot.pos.Y),(goal.X - robot.pos.X))));
		//then drive at that angle
		steadyDrive(-pidCompute(&robot.distPID, error), angle, sharpness);
	}
	steadyDrive(0, robot.pos.heading, 1);
	return;
}

//TUNE PID functions

float kP = 10;
float DkP = 2;
volatile int battery = nImmediateBatteryLevel;
//results in (97.0449 x^-1.29993 + 0.993483)
struct PIDs gen;
void tuneDistPID(float dist){
	const int maxWait = 200;//maximum time for PID to settle
	float lastResult = 0.0;
	const int initEncRight = SensorValue[Right];
	const int initEncLeft = SensorValue[Left];
	for(int j = 2; j <= 10; j++){
		DkP = 2;
		for(int a = 0; a < 15; a++){
			battery = nImmediateBatteryLevel;
			initPID(&gen, kP, 0.0, 0.0, 1.5, 300, true);//reinits PID
			gen.goal = dist*j;
			volatile float currentDist = 0.0;
			clearTimer(T3);
			while( time1[T3] < (maxWait * gen.goal)){
				currentDist = encoderDistInch( avg(SensorValue[Right] - initEncRight, SensorValue[Left] - initEncLeft));
				fwdsDrive(pidCompute(&gen, currentDist));
				delay(10);
			}
			delay(200);
			//change the k's
			if(currentDist > gen.goal) kP -= DkP;
			else {
				kP += DkP;
				DkP *= 0.5;//DkD gets smaller
			}
			lastResult = currentDist;
			//come back
			clearTimer(T3);
			gen.goal = 0;
			while(time1[T3] < maxWait * dist *j ){
				currentDist = encoderDistInch( avg(SensorValue[Right] - initEncRight, SensorValue[Left] - initEncLeft));
				fwdsDrive(pidCompute(&gen, currentDist));
			}
			delay(200);
			writeDebugStream("amnt: %.3f   &&    ", dist*j);  // writes the current value of int 'x' to the debug stream
			writeDebugStream("result: %.3f", lastResult);  // writes the current value of int 'x' to the debug stream
			writeDebugStreamLine("kP: %.3f", kP);  // writes the current value of int 'x' to the debug stream

		}
		writeDebugStream("FINAL amnt: %.3f   &&    ", dist*j);  // writes the current value of int 'x' to the debug stream
		writeDebugStreamLine("FINAL kP: %.3f", kP);  // writes the current value of int 'x' to the debug stream

	}
}
*/
#endif
