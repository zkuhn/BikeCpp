#include "BikeKinematics.h"
#include "estimated_pose.h"
#include <gtest/gtest.h>


namespace { 
/*
 * Test that moving forward in a slight right turn gives us
 * --  a slightly right heading
 * -- x position moved forward about a meter, and 
 * -- y position moved slightly down, but not more than .1 meters
 */
    TEST(Circumfrencetest, ForwardRightTurn) { 
        BikeKinematics bk(.2f, .2f, 1.0f);
        estimated_pose ep = bk.estimate(0, -0.2f, 400, 0.01f);
        ASSERT_GT(ep.heading, -0.195044);
        ASSERT_LT(ep.heading, -0.195042);
        ASSERT_GT(ep.x, .9);
        ASSERT_GT(ep.y, -.1);
    }
}
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
