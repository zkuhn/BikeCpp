#include "stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <iomanip>

#include "BikeKinematics.h"
#include "estimated_pose.h"


BikeKinematics::BikeKinematics(float frontRadius, float rearRadius, float hubDistance) :
	frontWheelRadius{ frontRadius }, rearWheelRadius{ rearRadius }, hubToHubDistance{ hubDistance }
{
	currentRearHubX = 0;
	currentRearHubY = 0;
	currentHeading = 0.0f;
	ticksPerRevolution = 512;
}


BikeKinematics::~BikeKinematics()
{
}

float BikeKinematics::getX() {
	return currentRearHubX;

}

float BikeKinematics::getY() {
	return currentRearHubY;

}

float BikeKinematics::getFrontWheelTravel(int encoder_ticks) {
	//std::cout << "looking at ticks" << encoder_ticks << std::endl;
	float rotation = ((float)encoder_ticks / this->ticksPerRevolution);

	//std::cout << "ticksPer was " << this->ticksPerRevolution << std::endl;
	
	//std::cout << "rotation was " << rotation << std::endl;
	return  this->getFrontWheelCircumfrence() * rotation;
}

estimated_pose BikeKinematics::getPose()
{
	estimated_pose returnPose;

	returnPose.x = this->currentRearHubX;
	returnPose.y = this->currentRearHubY;
	returnPose.heading = this->currentHeading;

	return returnPose;
}

float BikeKinematics::getFrontWheelCircumfrence() {

	return this->getCircumfrenceForRadius(frontWheelRadius);
}

float BikeKinematics::getCircumfrenceForRadius(float radius) {
	return radius * 2 * (float)M_PI;
}

/**
 * To find the new heading and distance moved, we need to trace the center of the 
 * rear hub along a circular arc, the same number of radians that the front
 * wheel travelled around its crcular arc. 
 *
 * After finding the front turning radius, and then converting distance travveled to
 * radians, we find the rear hub circle of travel radius, and calculate a new position 
 * moved that many radians. The tangent formula tan = opp / adj gives us the rear wheel
 * radius when we take the hub to hub distance (opposite) and divide  it by the inverse
 * tangent.
 *
 * Heading will simply be old heading plus the turned radians.
 *
 * new X and Y position will simple formula based on starting position and updated circle following.
 * some care is taken since direction of travel is tangent to the center of rotation coordinate
 * system we use.
 */
estimated_pose BikeKinematics::estimate(float time, float steering_angle, int encoder_ticks, float angular_velocity) {

	float totalDistance = this->getFrontWheelTravel(encoder_ticks);

	//handle the straight ahead case
	if (steering_angle <= 0.001 && steering_angle >= -.001) {
		this->currentRearHubX += cos(currentHeading) * totalDistance;
		this->currentRearHubY += sin(currentHeading) * totalDistance;
		
		return getPose();
	}

	//std::cout << std::fixed << std::setw(11) << std::setprecision(6)
	//	<< std::setfill('0') <<std::endl;

	float steeringCircleRadius = this->getTurningRadius(steering_angle);

	float radiansTravelled = totalDistance / steeringCircleRadius;
	
	//heading is easy, just add the radians
	float newHeading = currentHeading + radiansTravelled;
	//std::cout << " : current Heading : " << currentHeading << std::endl;
	
	//std::cout << " : new Heading: " << newHeading << std::endl;

	float rearWheelTurnRadius = this->hubToHubDistance / tan(steering_angle);
	 
	//now travel the rear center wheel on that arc to find its new position. 
	//dont forget the wheel is moving at 90degrees (pi/2) angle to it's angle relative to circle center (x = sin y = -cos )
	float travelVectorX = ( sin(newHeading) - sin(this->currentHeading) ) * rearWheelTurnRadius;
	float travelVectorY = -( cos(newHeading) - cos(this->currentHeading) ) * rearWheelTurnRadius;

	//std::cout << "travelVectorX " << travelVectorX << std::endl;
	//std::cout << "travelVectorY " << travelVectorY << std::endl;

	//std::cout << "rear wheel turn radius: " << rearWheelTurnRadius << std::endl;
	//std::cout << "radians travelled" << radiansTravelled << std::endl;

	
	// take the new heading mod 2 pi in case we looped around.


	this->currentHeading = normalizeHeading(newHeading);
	this->currentRearHubX += travelVectorX;
	this->currentRearHubY += travelVectorY;

	
	return getPose();


	//std::cout << "front wheel distance travelled was" << totalDistance;
}

/*
 * Take a heading of an arbitrary number of radians and normalize it between -M_PI and M_PI
 * make sure to explicitly cast M_PI to float, as the double val is slightly larger than the float val 
 * of PI
 */
float BikeKinematics::normalizeHeading(float heading) {
    if (heading > (float)M_PI) {
        return normalizeHeading(heading - 2*(float)M_PI);
    }
    if(heading <= (float)-M_PI) {
        return normalizeHeading(heading + 2*(float)M_PI);
    }
    return heading;
    /*float returnHeading = fmod (heading, M_PI * 2);
    if(returnHeading > M_PI){
        return returnHeading - 2 * M_PI;
    }
    if (returnHeading <= -M_PI) {
        return returnHeading + 2 * M_PI;
    }
    return returnHeading;*/
}

/**
 * In trig, SOH CAH TOH - sin is the opposite divided by hypotenuse
 * since we are looking for hypotenuse (radius of turn), and we have
 * opposite (hub to hub distance) O * H/O = H 
 * but H/O = 1/sin (steering angle) so we have our formula
 *
 * You can see that the steering angle is the same as the angle 
 * from the center of turning between the front and back hubs because
 * the tangent line of the front turning angle is perpendicular to radius
 * 
 */
float BikeKinematics::getTurningRadius(float steering_angle)
{
	float sinSteer = sin(steering_angle);
	return this->hubToHubDistance / sinSteer;
}

void BikeKinematics::printPos() {
	std::cout << "new heading: " << currentHeading << " X: " << currentRearHubX << " Y: " << currentRearHubY << std::endl;
}



