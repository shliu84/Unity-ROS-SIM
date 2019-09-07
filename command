source ~/catkin_ws/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch

rosrun image_view image_view image:=/unity_image _image_transport:=compressed

rostopic pub -1 /jeep/cmd_vel geometry_msgs/Twist -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

rostopic pub /jeep/cmd_vel geometry_msgs/Twist -r 1 -- '[0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'