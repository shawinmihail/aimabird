#include <iostream>
#include "AttControl.h"
#include "PathPlanner.hpp"


int main(int argc, char **argv)
{
    std::cout << "Aimabird planner started\n";
    ros::init(argc, argv, "aimabird_planner");
    PathPlanner pp;
    pp.init();
    return 0;
}
