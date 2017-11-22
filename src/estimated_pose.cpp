#include "stdafx.h"
#include "estimated_pose.h"
#include <iostream>

void estimated_pose::printPos() {
	std::cout << "new heading: " << heading << " X: " << x << " Y: " << y << std::endl <<std::endl;
}
