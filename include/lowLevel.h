#ifndef LOWLEVEL_H
#define LOWLEVEL_H
#include "api.h"
#include "util.h"
#include <string>
#include <vector>

#define ON true
#define OFF false
#define DRIVE 0
#define ANGLE 1

extern pros::ADIEncoder encoderL, encoderR, encoderM;


class Position {
public:
    Position() : X(0), Y(0), heading(0) {}
    Position(float x, float y, float head) : X(x), Y(y), heading(head) {}
    float X, Y, heading;
    float distanceToPoint(Position p1){
        return sqrt(sqr(p1.X - X) + sqr(p1.Y - Y) );
    }
};
class Odometry {
public:
    Odometry(Position primary, Position trackers) ://, pros::ADIEncoder L, pros::ADIEncoder R, pros::ADIEncoder M) :
    pos(primary), t_pos(trackers)
    {
    }//init constructor defaulted
    Position pos, t_pos;
    float wheelWidth = 8.55;//distance betweenn L&Rtrackers on base (inch)
    float lastL = 0, lastR = 0, lastM = 0;
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
    PIDcontroller(float p, float i, float d, float t, float dT, bool rev, bool run) :
    kP(p), kI(i), kD(d), thresh(t), delayThresh(dT), isReversed(rev), isRunning(run),
    LastError(0), Integral(0), Derivative(0), goal(0) {}
    float kP, kI, kD;
    float error;
    bool isReversed, isRunning;
    float Integral, Derivative, LastError;
    float thresh, delayThresh, goal;
    //functions
    float compute(float current) {
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
    float computeAngle(float currentAngle) {//assuming everything is in ANGLES
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
    float computeERR(float err) {
        if(isRunning){
            error = err;//calculate error
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
};

class mechanism{
public:
    mechanism(std::vector<pros::Motor> m, std::vector<pros::ADIEncoder>e, class PIDcontroller p) :
    mots(m), encs(e), pid(p)
    {
        if(!encs.empty()){
            for(const pros::ADIEncoder& e : encs){
                //e.set_value(0);//clears all encoders
            }
        }
    }
//private:
    std::vector<pros::Motor> mots;
    std::vector<pros::ADIEncoder> encs;
    float lastVel = 0;
public://functions
    class PIDcontroller pid;
    float velocity = 0;
    float getSensorVal(){
        if(!encs.empty()){
            float sumEncoders = 0;//average of all encoders in vector list
            for(const pros::ADIEncoder& e : encs){
                sumEncoders += e.get_value();
            }
            return sumEncoders / encs.size();//returns avg of all encoders in vector list
        }
        else {
            float sumMotEncoders = 0;//average of all MOTOR encoders in vector list
            for(const pros::Motor& m : mots){
                sumMotEncoders += m.get_position();
            }
            return sumMotEncoders / mots.size();//returns avg of all MOTOR encoders in vector list
        }
    }
    void move(float power){
        setPIDState(OFF);
        for(const pros::Motor& m : mots){//for each motor in mots
            m.move(power);
        }
    }
    void moveVel(float vel){
        setPIDState(OFF);
        for(const pros::Motor& m : mots){//for each motor in mots
            m.move_velocity(vel);
        }
    }
    void moveTo(float goal, float thresh, float power = 127){//simple encoder move
        float error = getSensorVal() - goal;
        while(abs(error) > thresh){
            move(sign(error) * power);
        }
        move(0);
    }
    void simpleControl(int buttonUp, int buttonDown, int power = 127){
        move(buttonUp*power - buttonDown*power);//simple up down control with 2 buttons (perf for indexer)
    }
    void moveAmnt(float amnt, float thresh, float power = 127){//simple encoder move
        float starting = getSensorVal();
        moveTo(starting + amnt, thresh, power);//moves to the position with AMNT as a constant quantity
    }
    void PID(){//does a PID move HAVE TO SET PID GOAL BEFOREHAND
        if(pid.isRunning) move(pid.compute(getSensorVal()));
        return;
    }
    void setPIDState(bool state){//ON = true, OFF = false
        pid.isRunning = state;
    }
    float computeVel(){//in rots/min
        float currentSensor = getSensorVal();
        const float delayAmnt = 20;
        velocity = ( currentSensor - lastVel) / (delayAmnt / 1000.0);///1000ms in 1 sec
        lastVel = currentSensor;
        return velocity / 2.0; //(converting ticks/sec to rot/min
            //[(ticks/sec) * (60sec/1min) * (1rev/360ticks)] * 3:1 (GR) = (1/6)*3 = 3/6 = 1/2)
        }
        float getMotorVel(){
            float sumMotVels = 0;//average of all MOTOR encoders in vector list
            for(const pros::Motor& m : mots){
                sumMotVels += m.get_actual_velocity();
            }
            return sumMotVels / mots.size();//returns avg of all MOTOR encoders in vector list

        }
    };

    class chassis{
    public:
        chassis(std::vector<pros::Motor> m, std::vector<PIDcontroller> p, Odometry o) :
        mots(m), pid(p), odom(o) {}
    //private:
        std::vector<pros::Motor> mots;//first 2 mots are RIGHT, second two are LEFT
        std::vector<PIDcontroller> pid;
        float lastDriveVel = 0, lastRotVel = 0;
    public://functions
        float driveVel = 0, rotVel = 0;
        class Odometry odom;
        void driveLR(int powerR, int powerL){//low level
            powerL = clamp(127, -127, powerL);
            powerR = clamp(127, -127, powerR);
            mots[0].move(powerR);
            mots[1].move(powerR);
            mots[2].move(powerL);
            mots[3].move(powerL);
        }
        void fwdsDrive(int power){//BASE
            driveLR(power, power);
        }
        void pointTurn(int speed){//turn
            driveLR(speed, -speed);
        }
        float computeVel(){
            float currentSensor = avg(encoderDistInch(encoderL.get_value()), encoderDistInch(encoderR.get_value()));
            const float delayAmnt = 20;
            driveVel = ( currentSensor - lastDriveVel) / (delayAmnt / 1000.0);///1000ms in 1 sec
            lastDriveVel = currentSensor;
            return driveVel;//converting inch/sec to ???
        }
        float computeRotVel(){
            float currentSensor = odom.pos.heading;
            const float delayAmnt = 20;
            rotVel = ( currentSensor - lastRotVel) / (delayAmnt / 1000.0);///1000ms in 1 sec
            lastRotVel = currentSensor;
            return rotVel;//converting degrees/sec to ???
        }
        void smoothDrive(int speed, const float angle, float sharpness = 1) {//drive base forwards
            const float scalar = 5;//scalar for rotation
            sharpness += 1;//parameter is from 0-1, do this addition to make sure it ranges from (1-2) [as explained below]
            speed = clamp(127, -127, speed);
            //for sharpness: 2 is direct point turn, 1 is turning off one side...
            //	it basically is just how much the different sides can be reversed to increase tha sharpness of the curve
            float dirSkew = limUpTo(127 * sharpness, scalar*normAngle(odom.pos.heading - angle));
            driveLR(speed + dirSkew, speed - dirSkew);
        }
        //higher levels
        void turnTo(const float degrees){//assumed CW is true
            pid[ANGLE].goal = degrees;
            pid[ANGLE].isRunning = true;//TURN ON PID
            //pid[ANGLE].kP = limUpTo(15, 97.0449 * pow(abs(normAngle(degrees - robot.pos.heading)), -1.29993) + 0.993483);FANCY
            while(abs(odom.pos.heading - pid[ANGLE].goal) > pid[ANGLE].thresh /*&& abs(rotVel) < 5*/){//waits for low velocity and close enoughness
                pointTurn(pid[ANGLE].computeAngle(odom.pos.heading));
                pros::delay(10);
            }
            pid[ANGLE].isRunning = false;//TURN OFF PID
            //final check and correction
            const int minSpeed = 50;//slow speed for robot's slight correction
            while(abs(normAngle(odom.pos.heading - pid[ANGLE].goal)) > pid[ANGLE].thresh){
                pointTurn(sign(normAngle(odom.pos.heading - pid[ANGLE].goal)) * minSpeed);
            }
            pointTurn(0);
            return;
        }
        void turn(const float degrees){
            turnTo(normAngle(odom.pos.heading + degrees));//basically turns to the current + increment
            return;
        }
        void fwds(const int amnt){//inches...ew //can TOTALLY use the odometry position vectors rather than encoders... smh
            const int initEncRight = encoderR.get_value();
            const int initEncLeft = encoderL.get_value();
            pid[DRIVE].goal = amnt;
            pid[DRIVE].isRunning = true;//TURN ON PID
            //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(amnt), -0.916209) + 2.05938);FANCY
            volatile float currentDist = 0.0;
            while(abs(currentDist - pid[DRIVE].goal) > pid[DRIVE].thresh && abs(driveVel) < 5){
                currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
                fwdsDrive(pid[DRIVE].compute(currentDist));
                pros::delay(10);
            }
            pid[DRIVE].isRunning = false;
            //final check and correction
            const int minSpeed = 40;//slow speed for robot's slight correction
            while(abs(currentDist - pid[DRIVE].goal) > pid[DRIVE].thresh){
                currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
                fwdsDrive(-sign(currentDist - pid[DRIVE].goal) * minSpeed);
            }
            fwdsDrive(0);
            return;
        }
        void driveToPoint(float x, float y){
            //first compute angle to goal
            //also divide by 0 is fine bc atan2 has error handling
            float phi = normAngle(toDeg(atan2((y - odom.pos.Y), (x - odom.pos.X))));
            //then compute distance to goal
            float dist = sqrt(sqr(y - odom.pos.Y) + sqr(x - odom.pos.X));
            turnTo(phi);//simple point turn`
            fwds(dist);//simple drive forwards
            return;
        }
        void smoothDriveToPoint(class Position goal, float sharpness = 1){
            float error = goal.distanceToPoint(odom.pos);
            pid[DRIVE].goal = 0;//goal is to have no distance between goal and current
            //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(error), -0.916209) + 2.05938);//fancy
            pid[DRIVE].isRunning = true;
            while(error > 3){//kinda bad... can be retuned n' stuff
            error = goal.distanceToPoint(odom.pos);
            //first compute angle to goal
            float phi = normAngle(toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
            //then drive at that angle
            smoothDrive(pid[DRIVE].computeERR(error), phi, sharpness);
        }
        smoothDrive(0, odom.pos.heading, 1);
        pid[DRIVE].isRunning = false;
        return;
    }
};
#endif
