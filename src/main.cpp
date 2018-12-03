#include <iostream>
#include "AttControl.h"


int main(int argc, char **argv)
{
  std::cout << "I am a bird!\n";
  ros::init(argc, argv, "aimabird");
  AttControl attControl;
  attControl.prepareToFly();
  return 0;
}
