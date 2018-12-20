#prepare
mkdir ~/1simflightlogs -- create log dir
catkin_make -- from catkin ws
# set up bila-sdk
add to files *.launch in .../bila-sdk/multiple-sitl/mavros
after string "<param name="target_component_id" value="1" />"
string "<param name="setpoint_attitude/use_quaternion" value="true" />"

add to white list of /bila-sdk/common/pluginlists.yaml
- setpoint_attitude
# start
rosrun aimabird aimabird_control
rosrun aimabird aimbird_example.py

# plot
use logplot.py to see logs
