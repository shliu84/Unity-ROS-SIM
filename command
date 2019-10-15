source ~/catkin_ws/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch

rosrun image_view image_view image:=/unity_image _image_transport:=compressed

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

rostopic pub /cmd_vel geometry_msgs/Twist -r 1 -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 


source ~/ws/devel/setup.bash
roslaunch roborts_bringup navigation_testing_with_simulation.launch gui:=true
rostopic echo /r1/scan 

roslaunch roborts_bringup unity_navigation.launch

rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_graph rqt_graph


catkin_create_pkg my_tf tf roscpp rospy nav_msgs
rosrun my_tf odom_listener

---
header: 
  seq: 14320
  stamp: 
    secs: 957
    nsecs: 675000000
  frame_id: "r1_tf/base_laser_link"
angle_min: -2.3561899662
angle_max: 2.3561899662
angle_increment: 0.035167016089
time_increment: 0.0
scan_time: 0.0
range_min: 0.15000000596
range_max: 8.0
ranges: [0.6645866632461548, 0.7076522707939148, 0.7458520531654358, 0.7183858752250671, 0.6967979073524475, 0.6743143796920776, 0.6848961710929871, 0.6309925317764282, 0.6355757713317871, 0.6125399470329285, 0.6023125052452087, 0.6008744239807129, 0.5946877002716064, 0.575767457485199, 0.559069037437439, 0.5674741864204407, 0.5423474907875061, 0.5557484030723572, 0.5486180782318115, 0.5436438322067261, 0.5628647804260254, 0.5512740612030029, 0.5316755175590515, 0.5547781586647034, 0.5484337210655212, 0.5508924126625061, 0.5503693222999573, 0.5507912635803223, 0.5495375394821167, 0.5443492531776428, 0.5724257826805115, 0.5608128309249878, 0.5722655653953552, 0.5694565773010254, 0.5686644315719604, 0.5905191898345947, 0.5848259329795837, 0.6165980100631714, 0.6084144711494446, 0.6324047446250916, 0.6585466265678406, 0.6737244129180908, 0.6620702743530273, 0.6936720609664917, 0.7383332848548889, 0.7639532685279846, 0.7254555225372314, 0.8075363636016846, 0.856956422328949, 0.8672882914543152, 0.8998633623123169, 0.9275450110435486, 0.9986284971237183, 1.0678013563156128, 1.1021265983581543, 1.2024084329605103, 1.29669189453125, 1.3810396194458008, 1.5223476886749268, 1.6649515628814697, 1.9026745557785034, 2.1369338035583496, 2.444087028503418, 2.8071541786193848, 2.769592761993408, 2.7880210876464844, 2.7671685218811035, 2.7538111209869385, 2.7573647499084473, 2.7382194995880127, 2.775207042694092, 2.7807087898254395, 2.769376754760742, 5.400730609893799, 7.654505252838135, 7.7158002853393555, 7.802998065948486, 7.863369941711426, 6.2551469802856445, 6.288963317871094, 6.421388626098633, 4.259162902832031, 3.954824924468994, 3.7021560668945312, 3.5544350147247314, 3.614748954772949, 7.661663055419922, 1.3971418142318726, 1.3683141469955444, 1.2770569324493408, 1.2553852796554565, 1.3051313161849976, 1.3287330865859985, 1.4221166372299194, 1.448736548423767, 1.5165270566940308, 1.5857030153274536, 1.69384765625, 1.8127095699310303, 1.9161652326583862, 3.1967103481292725, 3.528785467147827, 3.467452049255371, 3.4191813468933105, 3.3746511936187744, 3.351454257965088, 3.327965497970581, 3.395406723022461, 4.524834632873535, 4.525697231292725, 4.474876403808594, 4.468454360961914, 4.438070297241211, 4.433237075805664, 4.488364219665527, 4.4910407066345215, 4.482665061950684, 3.521165132522583, 2.830190658569336, 2.3417420387268066, 2.0382750034332275, 1.7853975296020508, 1.58722984790802, 1.4702037572860718, 1.3204913139343262, 1.2098280191421509, 1.1239322423934937, 1.0212888717651367, 0.9951539039611816, 0.9061341881752014, 0.8728886842727661, 0.8405873775482178, 0.802638590335846, 0.7539688348770142, 0.7135152816772461]
intensities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---


rostopic list
/clicked_point
/client_count
/clock
/connected_clients
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/los_path
/map
/map_metadata
/map_updates
/r1/amcl_pose
/r1/armor_detection_node_action/cancel
/r1/armor_detection_node_action/feedback
/r1/armor_detection_node_action/goal
/r1/armor_detection_node_action/result
/r1/armor_detection_node_action/status
/r1/cmd_vel
/r1/cmd_vel_acc
/r1/decision_costmap/decision_costmap/costmap
/r1/distance_map
/r1/field_bonus_status
/r1/field_supplier_status
/r1/game_status
/r1/gazebo_robot_pose
/r1/global_costmap/global_costmap/costmap
/r1/global_planner_node/path
/r1/global_planner_node_action/cancel
/r1/global_planner_node_action/feedback
/r1/global_planner_node_action/goal
/r1/global_planner_node_action/result
/r1/global_planner_node_action/status
/r1/imu
/r1/initialpose
/r1/joint_states
/r1/local_costmap/local_costmap/costmap
/r1/local_costmap/local_costmap/costmap_updates
/r1/local_planner_node/local_planner
/r1/local_planner_node_action/cancel
/r1/local_planner_node_action/feedback
/r1/local_planner_node_action/goal
/r1/local_planner_node_action/result
/r1/local_planner_node_action/status
/r1/mercure_camera/camera_info
/r1/mercure_camera/image_raw
/r1/mercure_camera/image_raw/compressed
/r1/mercure_camera/image_raw/compressed/parameter_descriptions
/r1/mercure_camera/image_raw/compressed/parameter_updates
/r1/mercure_camera/parameter_descriptions
/r1/mercure_camera/parameter_updates
/r1/move_base_simple/goal
/r1/odom
/r1/particlecloud
/r1/pose
/r1/roborts_omni_move/config/parameter_descriptions
/r1/roborts_omni_move/config/parameter_updates
/r1/robot_bonus
/r1/robot_damage
/r1/robot_status
/r1/scan
/r1/trajectory
/r3/cmd_vel
/r3/gazebo_robot_pose
/r3/imu
/r3/joint_states
/r3/mercure_camera/camera_info
/r3/mercure_camera/image_raw
/r3/mercure_camera/image_raw/compressed
/r3/mercure_camera/image_raw/compressed/parameter_descriptions
/r3/mercure_camera/image_raw/compressed/parameter_updates
/r3/mercure_camera/parameter_descriptions
/r3/mercure_camera/parameter_updates
/r3/odom
/r3/roborts_omni_move/config/parameter_descriptions
/r3/roborts_omni_move/config/parameter_updates
/r3/scan
/rosout
/rosout_agg
/tf
/tf_static

/gazebo/apply_body_wrench
/gazebo/apply_joint_effort
/gazebo/clear_body_wrenches
/gazebo/clear_joint_forces
/gazebo/delete_light
/gazebo/delete_model
/gazebo/get_joint_properties
/gazebo/get_light_properties
/gazebo/get_link_properties
/gazebo/get_link_state
/gazebo/get_loggers
/gazebo/get_model_properties
/gazebo/get_model_state
/gazebo/get_physics_properties
/gazebo/get_world_properties
/gazebo/pause_physics
/gazebo/reset_simulation
/gazebo/reset_world
/gazebo/set_joint_properties
/gazebo/set_light_properties
/gazebo/set_link_properties
/gazebo/set_link_state
/gazebo/set_logger_level
/gazebo/set_model_configuration
/gazebo/set_model_state
/gazebo/set_parameters
/gazebo/set_physics_properties
/gazebo/spawn_sdf_model
/gazebo/spawn_urdf_model
/gazebo/unpause_physics
/gazebo_gui/get_loggers
/gazebo_gui/set_logger_level
/gazebo_gui/set_parameters
/map_server/get_loggers
/map_server/set_logger_level
/r1/global_planner_node/get_loggers
/r1/global_planner_node/set_logger_level
/r1/local_planner_node/get_loggers
/r1/local_planner_node/set_logger_level
/r1/localization_node/get_loggers
/r1/localization_node/set_logger_level
/r1/mercure_camera/image_raw/compressed/set_parameters
/r1/mercure_camera/set_camera_info
/r1/mercure_camera/set_parameters
/r1/navigation_test_decision/get_loggers
/r1/navigation_test_decision/set_logger_level
/r1/roborts_omni_move/config/set_parameters
/r1/robot_state_publisher/get_loggers
/r1/robot_state_publisher/set_logger_level
/r1/rviz/get_loggers
/r1/rviz/reload_shaders
/r1/rviz/set_logger_level
/r1/vel_converter_node/get_loggers
/r1/vel_converter_node/set_logger_level
/r3/mercure_camera/image_raw/compressed/set_parameters
/r3/mercure_camera/set_camera_info
/r3/mercure_camera/set_parameters
/r3/roborts_omni_move/config/set_parameters
/r3/robot_state_publisher/get_loggers
/r3/robot_state_publisher/set_logger_level
/rosapi/action_servers
/rosapi/delete_param
/rosapi/get_loggers
/rosapi/get_param
/rosapi/get_param_names
/rosapi/get_time
/rosapi/has_param
/rosapi/message_details
/rosapi/node_details
/rosapi/nodes
/rosapi/publishers
/rosapi/search_param
/rosapi/service_host
/rosapi/service_node
/rosapi/service_providers
/rosapi/service_request_details
/rosapi/service_response_details
/rosapi/service_type
/rosapi/services
/rosapi/services_for_type
/rosapi/set_logger_level
/rosapi/set_param
/rosapi/subscribers
/rosapi/topic_type
/rosapi/topics
/rosapi/topics_for_type
/rosbridge_websocket/get_loggers
/rosbridge_websocket/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/static_map

/gazebo
/gazebo_gui
/map_server
/r1/global_planner_node
/r1/local_planner_node
/r1/localization_node
/r1/navigation_test_decision
/r1/robot_state_publisher
/r1/rviz
/r1/vel_converter_node
/r3/robot_state_publisher
/rosapi
/rosbridge_websocket
/rosout


