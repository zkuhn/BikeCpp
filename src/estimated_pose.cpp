#include "stdafx.h"
#include "estimated_pose.h"
#include <iostream>

estimated_pose::estimated_pose(){}

//move constrctor
estimated_pose::estimated_pose( estimated_pose && other ) {
    //std::cout << "move construtor called";
    x = other.x;
    y = other.y;
    heading = other.heading;
}

estimated_pose& estimated_pose::operator=(const estimated_pose& other){

    //according to cpp standard
    if (this == &other)  
    {  
        return *this;
    }
 
    //std::cout << "move assignment oper called";
    x = other.x;
    y = other.y;
    heading = other.heading;
    return *this;
}

void estimated_pose::printPos() {
	std::cout << "new heading: " << heading << " X: " << x << " Y: " << y << std::endl <<std::endl;
}
