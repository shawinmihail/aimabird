rosparam set /mavros/setpoint_attitude/use_quaternion "true"
echo use quaternion:
echo $(rosparam get /mavros/setpoint_attitude/use_quaternion)

pkill -9 aimabird_control
rosrun aimabird aimabird_control
