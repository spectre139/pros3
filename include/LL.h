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
#define CURVE 2

extern pros::ADIEncoder encoderL, encoderR, encoderM;
extern float encM, encL, encR;

using namespace pros;

class Position {
  public:
  Position() : X(0), Y(0), heading(0) {}
  Position(float x, float y, float angle) : X(x), Y(y), heading(angle) {}
  float X = 0, Y = 0, heading = 0;
  float distanceToPoint(Position p1){
    return sqrt( sqr(p1.X - X) + sqr(p1.Y - Y) );
  }
};

class Odometry {
  public:
  Odometry(Position primary, Position trackers) : pos(primary), t_pos(trackers){
    pos.X = pos.X = pos.heading = 0.0;
    t_pos.X = t_pos.X = t_pos.heading = 0.0;
  }//init constructor defaulted
  Position pos, t_pos;
  volatile bool resetEncoders = false;
  const float wheelWidth = 7;//(8.75+1.25*2)*1.09;//distance betweenn L&Rtrackers on base (inch)
  float lastL = 0, lastR = 0, lastM = 0, resetAngleSentinel = -PI;
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
  float compute(float current, bool isAngle = false) {
    if(isRunning){
      if (!isAngle) error = current - goal;//calculate error
      else error = normAngle(current - goal);//calculate error

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
  { }
  //private:
  std::vector<pros::Motor> mots;
  std::vector<pros::ADIEncoder> encs;
  float lastVel = 0;
  int upDog = 0;
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
    while(abs(getSensorVal() - goal) > thresh){
      move(-sign(getSensorVal() - goal) * power);
    }
    move(0);
  }
  void simpleControl(int buttonUp, int buttonDown, int power = 127){
    move(buttonUp*power - buttonDown*power);//simple up down control with 2 buttons (perf for indexer)
  }
  void skrrt(int vel){
    mots[0].move_velocity(vel);
    mots[1].move_velocity(-vel);

  }
  void skrrt2(int vel){
    mots[0].move_velocity(vel);
    mots[1].move(127);
  }
  void no(){
    move(0);
  }
  void no2(){
    mots[0].move(0);
    mots[1].move(75);
  }
  int toggeru(int urMother){ // ask me what upDog is - Uday
    if(urMother == 1){

      if(upDog == 2){
        move(127);
        upDog = 0;
        delay(300);
        return 1;

      }
      if(upDog == 1){
        move(-127);
        upDog++;
        delay(300);
        return 1;
      }
      if(upDog == 0){
        move(0);
        upDog++;
        delay(300);
        return 1;
      }

    }
  }
  void moveAmnt(float amnt, float thresh, float power = 127){//simple encoder move
    float starting = getSensorVal();
    moveTo(starting + amnt, thresh, power);//moves to the position with AMNT as a constant quantity
  }
  void moveTime(float time, float power = 127){//simple time based move
    int t = 1;
    while (t<time){
      move(power);//moves to the position with TIME as a constant quantity
      delay(1);
      t++;
    }
    move(-power);
    delay(10);
    move(0);
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
    float yeet(float t){
      float power = 2;
      if(sign(t) > 0) return pow(t, power) / pow(127.0, power - 1.0);
      return -pow(t, power) / pow(127.0, power - 1.0);
    }
    void driveLRDEAD(int powerR, int powerL){//low level
      powerL = clamp(127, -127, (powerL));
      powerR = clamp(127, -127, (powerR));
      mots[0].move(-powerR); // port 4 right
      mots[1].move(powerL);//port 5 left
      mots[2].move(powerL); //port 6 left
      mots[3].move(-powerR);//port 7  right
      mots[4].move(powerL);//move_velocity((mots[1].get_actual_velocity()));//port 8 diff left
      mots[5].move(powerR);//move_velocity((-(mots[0].get_actual_velocity())));//port 9 diff right
    }
    int thing(int potVal, int potReq){
      return ((potVal - potReq));
    }
    void driveLR(int powerR, int powerL){//low level
      powerL = clamp(127, -127, yeet(powerL));
      powerR = clamp(127, -127, yeet(powerR));
      mots[0].move(-powerR); // port 4 right
      mots[1].move(powerL);//port 5 left
      mots[2].move(powerL); //port 6 left
      mots[3].move(-powerR);//port 7  right
      mots[4].move(powerL);//port 8 diff left
      mots[5].move(-powerR);//port 9 diff right
    }
    void brakeHecka(int potVal){//low level
      mots[0].move_velocity(0); // port 4 right
      mots[1].move_velocity(0);//port 5 left
      mots[2].move_velocity(0); //port 6 left
      mots[3].move_velocity(0);//port 7  right
        /**    mots[4].move(-3*thing( 2450, potVal));//port 8 diff left
        mots[5].move(-3*thing( 2450,potVal));//port 9 diff right
        **/
      mots[4].move(80);//port 8 diff left
      mots[5].move(80);//port 9 diff right
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
    void smoothDrive(float speed, const float angle, float sharpness = 1) {//drive base forwards
      const float scalar = 2;//scalar for rotation
      sharpness += 1;//parameter is from 0-1, do this addition to make sure it ranges from (1-2) [as explained below]
      speed = clamp(127, -127, speed);
      //for sharpness: 2 is direct point turn, 1 is turning off one side...
      //	it basically is just how much the different sides can be reversed to increase tha sharpness of the curve
      float dirSkew = limUpTo(127 * sharpness, scalar*normAngle(odom.pos.heading - angle));
      driveLR(speed - dirSkew, speed + dirSkew);
    }
    //higher levels
    void fwdsNEW(float amnt, int cap = 127){
      const float initEncL = encL;
      const float initEncR = encR;
      int t = 0;
      pid[DRIVE].goal = amnt;
      pid[DRIVE].isRunning = true;
      float currentDist = 0;
      while(t < 2000){
        currentDist = avg(encoderDistInch(encL - initEncL), encoderDistInch(encR - initEncR));
        fwdsDrive(clamp(cap, -cap, pid[DRIVE].compute(currentDist)));
        delay(1);
        t++;
      }
      fwdsDrive(0);
      pid[DRIVE].isRunning = false;
    }
    void turnTo(const float degrees, const int timeThresh = 400){//assumed CW is true
      float oldKP = pid[ANGLE].kP;
      pid[ANGLE].goal = degrees;
      pid[ANGLE].isRunning = true;//TURN ON PID
      //pid[ANGLE].kP = limUpTo(15, 97.0449 * pow(abs(normAngle(degrees - robot.pos.heading)), -1.29993) + 0.993483);FANCY
      pid[ANGLE].kP = clamp(15, 0.2, 97 * pow(abs(normAngle(degrees - odom.pos.heading)), -1.3) + 1.4703);
      while(abs(odom.pos.heading - pid[ANGLE].goal) > pid[ANGLE].thresh /*&& abs(rotVel) < 5*/){//waits for low velocity and close enoughness
        pointTurn(pid[ANGLE].compute(odom.pos.heading, true));
        delay(10);
      }
      int t = 0;
      while(t < timeThresh){
        pointTurn(pid[ANGLE].compute(odom.pos.heading, true));
        delay(1);
        t++;
      }
      pid[ANGLE].isRunning = false;//TURN OFF PID
      pid[ANGLE].kP = oldKP;

      pointTurn(0);
      return;
    }
    void turnToTIME(const float degrees, const int timeThresh ){//assumed CW is true
      float oldKP = pid[ANGLE].kP;
      pid[ANGLE].goal = degrees;
      pid[ANGLE].isRunning = true;//TURN ON PID
      //pid[ANGLE].kP = limUpTo(15, 97.0449 * pow(abs(normAngle(degrees - robot.pos.heading)), -1.29993) + 0.993483);FANCY
      pid[ANGLE].kP = clamp(15, 0.2, 97 * pow(abs(normAngle(degrees - odom.pos.heading)), -1.3) + 1.4703);
      int t = 0;
      while(t < timeThresh){
        pointTurn(pid[ANGLE].compute(odom.pos.heading, true));
        delay(1);
        t++;
      }
      pid[ANGLE].isRunning = false;//TURN OFF PID
      pid[ANGLE].kP = oldKP;
      pointTurn(0);
      return;
    }
    void turnToKP(const float degrees, const float newkP, const int timeThresh = 400){//assumed CW is true
      float oldKP = pid[ANGLE].kP;
      pid[ANGLE].goal = degrees;
      pid[ANGLE].isRunning = true;//TURN ON PID
      //pid[ANGLE].kP = limUpTo(15, 97.0449 * pow(abs(normAngle(degrees - robot.pos.heading)), -1.29993) + 0.993483);FANCY
      pid[ANGLE].kP = newkP;//clamp(15, 0.2, 97 * pow(abs(normAngle(degrees - odom.pos.heading)), -1.3) + 1.4703);
      int t = 0;
      while(t < timeThresh){
        pointTurn(pid[ANGLE].compute(odom.pos.heading, true));
        delay(1);
        t++;
      }
      pid[ANGLE].isRunning = false;//TURN OFF PID
      pid[ANGLE].kP = oldKP;

      pointTurn(0);
      return;
    }
    void turn(const float degrees, const int timeThresh = 400){
      turnTo(normAngle(odom.pos.heading + degrees), timeThresh);//basically turns to the current + increment
      return;
    }
    void fwds(const float amnt, const int timeThresh, float cap = 127){//inches...ew //can TOTALLY use the odometry position vectors rather than encoders... smh
      const int initEncRight = encoderR.get_value();
      const int initEncLeft = encoderL.get_value();
      pid[DRIVE].goal = amnt;
      pid[DRIVE].isRunning = true;//TURN ON PID
      //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(amnt), -0.916209) + 2.05938);FANCY
      volatile float currentDist = 0.0;
      while(abs(currentDist - pid[DRIVE].goal) > pid[DRIVE].thresh){
        currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
        fwdsDrive(clamp(cap, -cap, pid[DRIVE].compute(currentDist)));
        delay(1);
      }
      int t = 0;
      while(t < timeThresh){
        currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
        fwdsDrive(clamp(cap, -cap, pid[DRIVE].compute(currentDist)));
        delay(1);
        t++;
      }
      pid[DRIVE].isRunning = false;

      fwdsDrive(0);
      return;
    }
    void fwdsTIME(const float amnt, const int timeThresh, float cap = 127){//inches...ew //can TOTALLY use the odometry position vectors rather than encoders... smh
      const int initEncRight = encoderR.get_value();
      const int initEncLeft =  encoderR.get_value();
      pid[DRIVE].goal = amnt;
      pid[DRIVE].isRunning = true;//TURN ON PID
      //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(amnt), -0.916209) + 2.05938);FANCY
      volatile float currentDist = 0.0;
      int t = 0;
      while(t < timeThresh){
        currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
        fwdsDrive(clamp(cap, -cap, pid[DRIVE].compute(currentDist)));
        delay(1);
        t++;
      }
      pid[DRIVE].isRunning = false;
      fwdsDrive(0);
      return;
    }
    void fwdsAng(const float amnt, const int timeThresh, const float angle, float cap = 127){//inches...ew //can TOTALLY use the odometry position vectors rather than encoders... smh
      const int initEncRight = encoderR.get_value();
      const int initEncLeft = encoderL.get_value();
      pid[DRIVE].goal = amnt;
      pid[DRIVE].isRunning = true;//TURN ON PID
      //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(amnt), -0.916209) + 2.05938);FANCY
      volatile float currentDist = 0.0;
      while(abs(currentDist - pid[DRIVE].goal) > pid[DRIVE].thresh){
        currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
        smoothDrive(clamp(cap, -cap, pid[DRIVE].compute(currentDist)), angle);
        delay(1);
      }
      int t = 0;
      while(t < timeThresh){
        currentDist = avg(encoderDistInch(encoderL.get_value() - initEncLeft), encoderDistInch(encoderR.get_value()  - initEncRight));
        smoothDrive(clamp(cap, -cap, pid[DRIVE].compute(currentDist)), angle);
        delay(1);
        t++;
      }
      pid[DRIVE].isRunning = false;
      fwdsDrive(0);
      return;
    }
    void driveToPoint(float x, float y, bool isBackwards = false){
      //first compute angle to goal
      //also divide by 0 is fine bc atan2 has error handling
      float phi = normAngle(toDeg(atan2((y - odom.pos.Y), (x - odom.pos.X))));
      //then compute distance to goal
      float dist = sqrt(sqr(y - odom.pos.Y) + sqr(x - odom.pos.X));
      if(!isBackwards) {//normal turn to angle and drive
        turnTo(phi, 400);//simple point turn
        fwds(dist, 400);//simple drive forwards
      }
      else {//drive to point, but backwards, so back reaches the point first.
        turnTo(normAngle(phi + 180), 400);//simple point turn (but backwards)
        fwds(-dist, 400);//simple drive forwards
      }
      return;
    }
    void driveToPointTIME(float x, float y, float timeVar, bool isBackwards = false){
      //first compute angle to goal
      //also divide by 0 is fine bc atan2 has error handling
      timeVar*=0.5;//only give half to each
      float phi = normAngle(toDeg(atan2((y - odom.pos.Y), (x - odom.pos.X))));
      //then compute distance to goal
      float dist = sqrt(sqr(y - odom.pos.Y) + sqr(x - odom.pos.X));
      if(!isBackwards) {//normal turn to angle and drive
        turnToTIME(phi, timeVar);//simple point turn
        fwdsTIME(dist, timeVar);//simple drive forwards
      }
      else {//drive to point, but backwards, so back reaches the point first.
        turnToTIME(normAngle(phi + 180), timeVar);//simple point turn (but backwards)
        fwdsTIME(-dist, timeVar);//simple drive forwards
      }
      return;
    }
    void smoothDriveToPoint(float X, float Y, float sharpness, bool isBackwards = false){
      class Position goal(X, Y, 0);
      float error = goal.distanceToPoint(odom.pos);
      pid[CURVE].goal = 0;//goal is to have no distance between goal and current
      //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(error), -0.916209) + 2.05938);//fancy
      pid[CURVE].isRunning = false;
      while(error > 3){//kinda bad... can be retuned n' stuff
        error = goal.distanceToPoint(odom.pos);
        //first compute angle to goal
        if(!isBackwards) {//normal turn to angle and drive
          float phi = normAngle(toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
          //then drive at that angle
          smoothDrive(7.5*error, phi, sharpness);
        }
        else {//drive to point, but backwards, so back reaches the point first.
          float phi = normAngle(180 + toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
          //then drive at that angle
          smoothDrive(-7.5*error, phi, sharpness);
        }
      }
      fwds(0, 0);
      pid[CURVE].isRunning = false;
      return;
    }
    void smoothDriveToPointTIME(float X, float Y, float sharpness, float timevar, bool isBackwards = false){
      class Position goal(X, Y, 0);
      float error = goal.distanceToPoint(odom.pos);
      pid[CURVE].goal = 0;//goal is to have no distance between goal and current
      //pid[DRIVE].kP = limUpTo(20, 28.0449 * pow(abs(error), -0.916209) + 2.05938);//fancy
      pid[CURVE].isRunning = false;
      int t = 0;
      while(t < timevar){//kinda bad... can be retuned n' stuff
        error = goal.distanceToPoint(odom.pos);
        //first compute angle to goal
        if(!isBackwards) {//normal turn to angle and drive
          float phi = normAngle(toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
          //then drive at that angle
          smoothDrive(7.5*error, phi, sharpness);
        }
        else {//drive to point, but backwards, so back reaches the point first.
          float phi = normAngle(180 + toDeg(atan2((goal.Y - odom.pos.Y), (goal.X - odom.pos.X))));
          //then drive at that angle
          smoothDrive(-7.5*error, phi, sharpness);
        }
        t++;
        delay(1);
      }
      fwds(0, 0);
      pid[CURVE].isRunning = false;
      return;
  }

};

#endif
