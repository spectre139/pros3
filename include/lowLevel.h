#ifndef LOWLEVEL_H
#define LOWLEVEL_H
#include "api.h"
#include "util.h"
#include <string>
#include <vector>

#define ON true
#define OFF false

class Position {
public:
    Position() : X(0), Y(0), heading(0) {}
    Position(float x, float y, float head) : X(x), Y(y), heading(head) {}

    float X, Y, heading;
};
class Odometry {
public:
    Odometry(Position primary, Position trackers, pros::ADIEncoder L, pros::ADIEncoder R, pros::ADIEncoder M) :
    encoderL(L), encoderR(R), encoderM(M),
    pos(primary), t_pos(trackers)
    {
        encoderL.reset();
        encoderR.reset();
        encoderM.reset();
    }//init constructor defaulted
    class Position pos, t_pos;
    pros::ADIEncoder encoderL, encoderR, encoderM;
    float centralAvg = 0, wheelWidth = 8.35;//distance betweenn L&Rtrackers on base (inch)
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
};

class mechanism{
public:
    mechanism(std::vector<pros::Motor> m, std::vector<pros::ADIEncoder>e, class PIDcontroller p) :
    mots(m), encs(e), pid(p)
    {
        if(!encs.empty()){
            for(const pros::ADIEncoder& e : encs){
                e.reset();//clears all encoders
            }
        }
    }
private:
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
    void moveAmnt(float amnt, float thresh, float power = 127){//simple encoder move
        float starting = getSensorVal();
        moveTo(starting + amnt, thresh, power);//moves to the position with AMNT as a constant quantity
    }
    void PID(){//does a PID move HAVE TO SET PID GOAL BEFOREHAND
        if(pid.isRunning) move(pid.pidCompute(getSensorVal()));
        return;
    }
    void setPIDState(bool state){//ON = true, OFF = false
        pid.isRunning = state;
    }
    float computeVel(){
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
    private:
        std::vector<pros::Motor> mots;//first 2 mots are RIGHT, second two are LEFT
        std::vector<PIDcontroller> pid;
        public://functions
        class Odometry odom;
        void driveLR(int powerR, int powerL){//low level
            powerL = clamp(127, -127, powerL);
            powerR = clamp(127, -127, powerR);
            mots[0].move(powerR);
            mots[1].move(powerR);
            mots[2].move(powerL);
            mots[3].move(powerL);
        }
        void fwds(int power){//BASE
            driveLR(power, power);
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
            float dirSkew = limUpTo(127 * sharpness, scalar*normAngle(odom.pos.heading - angle));
            driveLR(speed + dirSkew, speed - dirSkew);
        }

    };
    #endif