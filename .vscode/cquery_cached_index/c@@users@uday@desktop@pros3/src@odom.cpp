#include "api.h"
#include "util.h"
#include "lowLevel.h"

using namespace pros;
void calculatePos(void* param){
  ADIEncoder encL (1, 2, true), encR (3, 4, true), encM (5, 6, false);

  class Odometry* Odom = (Odometry*) param;
  const float scaleLeft = 1;//1.0436;
	for(;;){

		float dR = encoderDistInch(encR.get_value())*scaleLeft  - Odom->lastR;//change in right encoder
		float dL = encoderDistInch(encL.get_value())  - Odom->lastL;//change in left
		float dM = encoderDistInch(encM.get_value())  - Odom->lastM;//change in middle

		Odom->lastR += dR;//updates "last" values
		Odom->lastL += dL;//updates "last" values
		Odom->lastM += dM;//updates "last" values

		float dCentral = avg(dR, dL);//average of deltas
		float dHeading = toDeg(dR - dL) / Odom->wheelWidth;//change in angle
		float avgHeading = normAngle(Odom->pos.heading + dHeading / 2.0);  // Angle r is assumed to have been facing when moving dS.

		float radHeading = boundAngle(toRad(avgHeading));
		// Update current r position.
		Odom->t_pos.heading = (Odom->t_pos.heading + dHeading);//should this be normalized???
		Odom->t_pos.X += dCentral * cos(radHeading) + dM * sin(radHeading);
		Odom->t_pos.Y += dCentral * sin(radHeading) - dM * cos(radHeading);
		//add little vector after calculating H mech's position
		const float distToCenter = 6;//inches the center of the H mech to the center of r's rotation
		Odom->pos.heading = normAngle(Odom->t_pos.heading + 90);
		Odom->pos.X = Odom->t_pos.X + distToCenter * cos(radHeading);
		Odom->pos.Y = Odom->t_pos.Y + distToCenter * sin(radHeading) - distToCenter;
		//little delays
        delay(1);
    if(Odom->resetEncoders){
      encR.reset();
      encL.reset();
      encM.reset();
      Odom->resetEncoders = false;//no need to reset again
    }
	}
}
