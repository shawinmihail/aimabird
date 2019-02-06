#include <iostream>
#include "EstimationTest.h"


int main(int argc, char **argv)
{
    std::cout << "Aimabird test started\n";
    ros::init(argc, argv, "aimabird_test");
    EstimationTest estimationTest;
    estimationTest.prepareToFly();
    return 0;
}
