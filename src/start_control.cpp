#include <iostream>
#include "AttControl.h"
#include "PathPlanner.hpp"


int main(int argc, char **argv)
{
    std::cout << "Aimabird control started!!!!\n";
    ros::init(argc, argv, "aimabird_control");
    AttControl attControl;
    //attControl.test();
    attControl.prepareToFly();
    return 0;
}
