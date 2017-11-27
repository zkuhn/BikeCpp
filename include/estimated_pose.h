#pragma once

struct estimated_pose {
	float x;       //in meters
	float y;       //in meters
	float heading; //in radians

	void printPos();

	estimated_pose();
        estimated_pose( estimated_pose && );
	estimated_pose& operator=(const estimated_pose&);

};
