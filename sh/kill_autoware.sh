echo ===== kill autoware
pkill ros2
pkill rviz2
pkill aggregator_node

# Send ctrl c to all "ros" named process
pgrep ros | awk '{ print "kill -2 $(pgrep -P ", $1, ") > /dev/null 2>&1" }' | sh
pgrep ros | awk '{ print "kill -2 ", $1, " > /dev/null 2>&1" }' | sh

ps aux | grep python3 | grep ros2 | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep python3 | grep rqt_reconfigure | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep component_container | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep robot_state_publisher | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep topic_tools/relay | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep "ros-args" | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep "AWSIM_demo.x86_64" | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep "rosbridge_websocket" | grep -v grep | awk '{ print "kill -9", $2 }' | sh

echo ===== daemon reboot
ros2 daemon stop
ros2 daemon start

echo ===== Force-kill all "ros" named process
pgrep ros | awk '{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }' | sh
sleep 1
pgrep ros | awk '{ print "kill -9 ", $1, " > /dev/null 2>&1" }' | sh

echo ===== check topic is still alive
ros2 topic list
