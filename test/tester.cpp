#include "BikeKinematics.h"
#include "estimated_pose.h"
#include <gtest/gtest.h>
#define _USE_MATH_DEFINES
#include <math.h>

namespace { 
/*
 * Test that moving forward in a slight right turn gives us
 * --  a slightly right heading
 * -- x position moved forward about a meter, and 
 * -- y position moved slightly down, but not more than .1 meters
 */
    TEST(CircumfrenceTest, ForwardRightTurn) { 
        BikeKinematics bk(0.2f, 0.2f, 1.0f);
        estimated_pose ep = bk.estimate(0, -0.2f, 400, 0.01f);
        ASSERT_GT(ep.heading, -0.195044f);
        ASSERT_LT(ep.heading, -0.195042f);
        ASSERT_GT(ep.x, 0.9f);
        ASSERT_GT(ep.y, -0.1f);
    }
    TEST(CircleTest, ForwardLeftCircle){
        BikeKinematics bk3(0.2f, 0.2f, 1.0f);

        //cout << "testing full left circle " << endl;

        float steering_angle = (float)M_PI / 4;
        float frontTurningRadius = bk3.getTurningRadius(steering_angle);
        float wheelCircumfrence = bk3.getFrontWheelCircumfrence();
        float frontWheelTravel = bk3.getCircumfrenceForRadius(frontTurningRadius);
        float wheelRevolutions = frontWheelTravel / wheelCircumfrence;

        //get the estimated pose after completing a left circle. it should be close to {0,0,0}
        estimated_pose returnPose = bk3.estimate(0.01f, steering_angle, (int)(wheelRevolutions * 512) , 0.01f);
        // X should be close to zero
        ASSERT_GT(returnPose.x, -0.01f);
        ASSERT_LT(returnPose.x,  0.01f);
        // Y should be close to zero
        ASSERT_GT(returnPose.y, -0.01f);
        ASSERT_LT(returnPose.y,  0.01f);
        // heading should be close to zero
        ASSERT_GT(returnPose.heading, -0.01f);
        ASSERT_LT(returnPose.heading,  0.01f);
    }

    // test that the heading normalizer is keeping headings between positive pi and negative PI.
    TEST(HeadingTest,MultipleValues){

        float tests[] = {7.0f, 0.1f, -7.0f, -70.0f, 5.4f, -5.4f, -70.0f};
        float returnHeading;
   
        BikeKinematics bk3(0.2f, 0.2f, 1.0f);
        for (float heading: tests){

            returnHeading = bk3.normalizeHeading(heading);
            ASSERT_GT(returnHeading, -M_PI);
            ASSERT_LE(returnHeading,  M_PI);

        }
        //Note that these tests cannot be expected to pass because floating point math can slightly alter the values
        //we could only expect the values to be close.
        ASSERT_NEAR(bk3.normalizeHeading(M_PI),M_PI, 0.00001f) ;
        ASSERT_NEAR(bk3.normalizeHeading(-M_PI),M_PI, 0.00001f) ;

    }

   
}
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
