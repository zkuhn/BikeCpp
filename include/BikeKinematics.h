#pragma once
#include "estimated_pose.h"

class BikeKinematics
{
	float frontWheelRadius;
	float rearWheelRadius;
	float hubToHubDistance;

	float currentRearHubX;
	float currentRearHubY;

	float currentHeading;

	unsigned ticksPerRevolution;

public:

	BikeKinematics(float frontWheelRadius, float rearWheelRadius, float hubToHubDistance);
	~BikeKinematics();

	/**
	 * This method has side effects! It will update the instance to a new heading and position so that subsequent postition update estimates
	 * will continue to move the model in 2d space.
	 *
	 * Model currently uses steering_angle and ticks . Assume positive ticks for forward motion and negative ticks for reverse motion
	 * time and angular velocity are invariants in this implementation and ignored.
	 *
	 * @param float time
	 * @param float steering_angle
	 * @param int encoder_ticks
	 * @param float angular_velocity
	 */
	estimated_pose estimate(float time, float steering_angle, int encoder_ticks, float angular_velocity);

	float getX();
	float getY();
	float getTurningRadius(float steering_angle);
	float getFrontWheelCircumfrence();

	float getCircumfrenceForRadius(float radius);

protected:
	float getFrontWheelTravel(int encoder_ticks);
	estimated_pose getPose();

	
	
	void printPos();
};


