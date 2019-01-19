#include "api.h"
#include "util.h"
#include "lowLevel.h"
#include "robot.h"

extern pros::ADIEncoder encoderL, encoderR, encoderM;

using namespace pros;

#define SENTINEL (-PI)
//defining motor ports:
#define RFront   	2
#define LFront    14
#define LBack     13
#define RBack     12
#define LDiff     1
#define RDiff     11

extern Robot rob;

extern float LeftBAvg, RightBAvg;
extern float encM, encL, encR;
using namespace pros;
void calculatePosBASE(void* param){
//  static ADIEncoder encM (1, 2, false), encR (3, 4, false), encL (5, 6, false);
  class Odometry* Odom = (Odometry*) param;
    for(;;){
      if(!Odom->resetEncoders){
        //float dR = encoderDistInchBASE( LeftBAvg )  - Odom->lastR;//change in right encoder
        //float dL = encoderDistInchBASE( RightBAvg )  - Odom->lastL;//change in left
        //float dM = encoderDistInch(encM.get_value())  - Odom->lastM;//change in middle
        float dR = encoderDistInch(encR)  - Odom->lastR;//change in right encoder
        float dL = encoderDistInch(encL)  - Odom->lastL;//change in left
        float dM = encoderDistInch(encM)  - Odom->lastM;//change in middle

        Odom->lastR += dR;//updates "last" values
        Odom->lastL += dL;//updates "last" values
        Odom->lastM += dM;//updates "last" values

        float dCentral = avg(dR, dL);//average of deltas
        float dHeading = toDeg(dR - dL) / Odom->wheelWidth;//change in angle
        float avgHeading = normAngle(Odom->pos.heading + dHeading / 2.0);  // Angle r is assumed to have been facing when moving dS.

        float radHeading = boundAngle(toRad(avgHeading));
        // Update current r position.
        Odom->t_pos.heading = (Odom->t_pos.heading - dHeading);//should this be normalized???
        Odom->t_pos.X += dCentral * cos(radHeading) + dM * sin(radHeading);
        Odom->t_pos.Y += dCentral * sin(radHeading) - dM * cos(radHeading);
        //add little vector after calculating H mech's position
        const float distToCenter = 4.45;//inches the center of the H mech to the center of r's rotation
        Odom->pos.heading = normAngle(Odom->t_pos.heading + 90);
        Odom->pos.X = Odom->t_pos.X + distToCenter * cos(radHeading);
        Odom->pos.Y = Odom->t_pos.Y + distToCenter * sin(radHeading) - distToCenter;
      }
        //for resetting
        if(Odom->resetEncoders){
            Motor(LFront).tare_position();
            Motor(LBack).tare_position();
            Motor(RFront).tare_position();
            Motor(RBack).tare_position();
            encM = 0;
            encL = 0;
            encR = 0;
            Odom->lastR = Odom->lastL = Odom->lastM = 0;
            Odom->pos.X = Odom->pos.Y = 0;
            Odom->t_pos.X = Odom->t_pos.Y = 0;
            Odom->resetEncoders = false;//no need to reset again
        }

        if(Odom->resetAngleSentinel != SENTINEL){//new
            Odom->t_pos.heading = Odom->resetAngleSentinel - 90;
            Odom->pos.heading = Odom->resetAngleSentinel;//should reset the angle...
            Odom->resetAngleSentinel = SENTINEL;
        }

        //little delays
        delay(1);
    }
}
