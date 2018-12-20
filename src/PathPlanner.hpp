#include "Eigen/Dense"
#include <iostream>
#include <stdio.h>
#include <vector>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using std;
using Eigen;
using ros;
using nav_msgs;

class PathPlanner {
public:
    PathPlanner():
    mapName("/rtabmap/proj_map")
    {}

    void init(){

    }

    void mapCb(){

    }

private:
    string  mapName;
    Subscriber  mapSub;
};
