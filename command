source ~/catkin_ws/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch

rosrun image_view image_view image:=/unity_image _image_transport:=compressed

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

rostopic pub /cmd_vel geometry_msgs/Twist -r 50 -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 


source ~/ws_2/devel/setup.bash
roslaunch roborts_bringup navigation_testing_with_simulation.launch gui:=true
rostopic echo /r1/scan 

roslaunch roborts_bringup unity_navigation.launch

rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_graph rqt_graph


catkin_create_pkg my_tf tf roscpp rospy nav_msgs
rosrun my_tf odom_listener




