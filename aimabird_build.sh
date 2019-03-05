pkill -9 aimabird
pkill -9 aimabird_control
cd ../..
catkin_make aimabird
catkin_make aimabird aimabird_control

#rosrun aimabird aimabird_control
