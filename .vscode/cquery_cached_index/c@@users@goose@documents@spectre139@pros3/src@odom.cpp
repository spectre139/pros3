#include "main.h"

using namespace pros;

void calculatePos(void* param){
    Robot* r = (Robot*) param;
	for(;;){

		float dR = encoderDistInch(r->encoderR.get_value())  - r->odom.lastR;//change in right encoder
		float dL = encoderDistInch(r->encoderL.get_value())  - r->odom.lastL;//change in left
		float dM = encoderDistInch(r->encoderM.get_value())  - r->odom.lastM;//change in middle

		r->odom.lastR += dR;//updates "last" values
		r->odom.lastL += dL;//updates "last" values
		r->odom.lastM += dM;//updates "last" values

		float dCentral = avg(dR, dL);//average of deltas
		float dHeading = toDeg(dR - dL) / r->wheelWidth;//change in angle
		float avgHeading = normAngle(r->pos.heading + dHeading / 2.0);  // Angle r is assumed to have been facing when moving dS.

		float radHeading = boundAngle(toRad(avgHeading));
		// Update current r position.
		r->t_pos.heading += dHeading;
		r->t_pos.X += dCentral * cos(radHeading) + dM * sin(radHeading);
		r->t_pos.Y += dCentral * sin(radHeading) - dM * cos(radHeading);
		//add little vector after calculating H mech's position
		const float distToCenter = 2.18524;//inches the center of the H mech to the center of r's rotation
		r->pos.heading = normAngle(r->t_pos.heading + 90);
		r->pos.X = r->t_pos.X + distToCenter * cos(radHeading);
		r->pos.Y = r->t_pos.Y + distToCenter * sin(radHeading) - distToCenter;
		//little delays
        delay(1);
	}
}
