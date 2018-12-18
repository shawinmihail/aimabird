#include <iostream>
#include "AttControl.h"
#include "Replay.hpp"


int main(int argc, char **argv)
{
  std::cout << "Aimabird replay recalculation...\n";
  ros::init(argc, argv, "aimabird_replay");

  Replay rp;
  rp.ekfRecalk();

  return 0;
}
