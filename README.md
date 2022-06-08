# ros3djs_react

## Install ROS dependencies

```
sudo apt -y install ros-${ROS_DISTRO}-joint-state-publisher-gui
sudo apt -y install ros-${ROS_DISTRO}-tf2-web-republisher
sudo apt -y install ros-${ROS_DISTRO}-rosbridge-websocket
```

## Build universal robot packages

```
cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/universal_robot.git -b melodic-devel
```

## Prepare ROS nodes

- run the following source command in every ROS execution terminal
```
source /opt/ros/${ROS_DISTRO}/setup.bach
source ~/catkin_ws/devel/setup.bash
```

- 1st terminal
```
roslaunch ur_description ur5_upload.launch
```
- 2nd terminal
```
rosrun robot_state_publisher robot_state_publisher
```
- 3rd terminal
```
rosrun joint_state_publisher_gui joint_state_publisher_gui
```
- 4th terminal
```
rosrun tf2_web_republisher tf2_web_republisher
```
- 5th terminal
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

## Run web app
```
yarn
yarn start
```
