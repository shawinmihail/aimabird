#include <iostream>
#include "AttControl.h"
#include "PathPlanner.hpp"
#include <signal.h> //  our new library

AttControl* attControl;
void ctrl_c(int sig){ // can be called asynchronously
    attControl->stop();
    std::cout << "\ntry to stop...\n" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aimabird_control");
    attControl = new AttControl();
    signal(SIGINT, ctrl_c);
    std::cout << "Aimabird control started\n";
    attControl->prepareToFly();
    return 0;
}
