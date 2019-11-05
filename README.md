# Unity_ROS_SIM
## Installation and Configuration
### Install Unity
Download Unity for ubuntu <https://forum.unity.com/threads/unity-hub-v2-0-0-release.677485/?_ga=2.241212994.1757428817.1572957430-1380361049.1571101592>
### Install ROS
1. Install the ros-kinetic-desktop-full here <http://wiki.ros.org/kinetic/Installation/Ubuntu>
2. Install RoboRTS following the step. <https://xinzhangk.github.io/ROS-docs/sim.html>
3. Install ROS bridge server
```
sudo apt-get install ros-kinetic-rosbridge-suite
```
### Configuration
These files are in RoboRTS_Config dolder
1. add decision.prototxt to 

```
~/ws/src/RoboRTS-v2/roborts_decision/config/
```

2. add unity_navigation.launch to 
```
~/ws/src/RoboRTS-v2/roborts_bringup/launch/
```
3. replace static_tf.launch in 
```
~/ws/src/RoboRTS-v2/roborts_bringup/launch/
```

## Run Navigation Simulation
1. Run ROS bridge server

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

2. Open a NEW terminal and source the setup file

```
source $HOME/roborts_ws/devel/setup.bash
```

3. Open Unity projoect and run
4. Run Unity navigation simulation launch file
```
roslaunch roborts_bringup unity_navigation.launch
```
5. Set the initial pose in Rriz
6. Test navigation
