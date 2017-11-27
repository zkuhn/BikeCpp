#include "stdafx.h"
#include "BikeTester.h"
#include "BikeKinematics.h"
#include "estimated_pose.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

BikeTester::BikeTester()
{
}


BikeTester::~BikeTester()
{
}

void BikeTester::testAll() {

	BikeKinematics bk(.2f, .2f, 1.0f);
	cout << "testing a bike kinematic " << endl;
	//cout << bk.getX() << " , " << bk.getY() << endl;

	estimated_pose outputPose = bk.estimate(0, 0.2f, 200, 0.01f);
	
	outputPose.printPos();

	outputPose = bk.estimate(0, 0.2f, 200, 0.01f);
	outputPose.printPos();

	//bk.updateMovement(0, 0.2f, 200, 0.01f);

	cout << "testing a second bike kinematic " << endl;

	BikeKinematics bk2(.2f, .2f, 1.0f);

	outputPose = bk2.estimate(0, 0.2f, 400, 0.01f);
	outputPose.printPos();

	cout << "testing a third bike kinematic - straight ahead" << endl;

	BikeKinematics bk3(.2f, .2f, 1.0f);

	outputPose = bk3.estimate(0, 0.0f, 400, 0.01f);
	outputPose.printPos();

	testLeftCircle();
	testRightCircle();
	testRightTurn();
	testBackupRightTurn();

}

void BikeTester::testLeftCircle() {

	BikeKinematics bk3(.2f, .2f, 1.0f);

	cout << "testing full left circle " << endl;

	float steering_angle = (float)M_PI / 4;
	float frontTurningRadius = bk3.getTurningRadius(steering_angle);
	float wheelCircumfrence = bk3.getFrontWheelCircumfrence();
	float frontWheelTravel = bk3.getCircumfrenceForRadius(frontTurningRadius);
	float wheelRevolutions = frontWheelTravel / wheelCircumfrence;

	//get the estimated pose after completing a left circle. it should be close to {0,0,0}
	estimated_pose returnPose = bk3.estimate(0.01f, steering_angle, (int)(wheelRevolutions * 512) , 0.01f);
	returnPose.printPos();
	
	
}

void BikeTester::testRightCircle() {

	BikeKinematics bk3(.2f, .2f, 1.0f);

	cout << "testing full right circle " << endl;

	float steering_angle = -(float)M_PI / 4;
	float frontTurningRadius = bk3.getTurningRadius(steering_angle);
	float wheelCircumfrence = bk3.getFrontWheelCircumfrence();
	float frontWheelTravel = bk3.getCircumfrenceForRadius(frontTurningRadius);
	float wheelRevolutions = frontWheelTravel / wheelCircumfrence;

	//get the estimated pose after completing a left circle. it should be close to {0,0,0}
	estimated_pose returnPose = bk3.estimate(0.01f, steering_angle, (int)(wheelRevolutions * 512), 0.01f);
	returnPose.printPos();


}

void BikeTester::testRightTurn() {
	cout << "testing right turn" << endl;
	BikeKinematics bk(.2f, .2f, 1.0f);

	estimated_pose outputPose = bk.estimate(0, -0.2f, 400, 0.01f);
	outputPose.printPos();
}

void BikeTester::testBackupRightTurn() {
	cout << "testing backup right turn" << endl;
	BikeKinematics bk(.2f, .2f, 1.0f);

	estimated_pose outputPose = bk.estimate(0, -0.2f, -400, 0.01f);
	outputPose.printPos();
}
