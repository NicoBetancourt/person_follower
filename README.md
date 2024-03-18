# Person-following Python ROS 2 template

We assume that [ROS 2](https://docs.ros.org/) and [Webots](https://cyberbotics.com/) are installed in the system. 

For the steps below we use ROS2 Foxy and Webots R2022b.

1. Install the prerequisites
```
sudo apt install ros-foxy-webots-ros2-turtlebot
```
2. Create a ROS 2 workspace
```
mkdir -p ~/ros2_ws/src
```
3. Clone this repository and build the package
```
cd ~/ros2_ws/src
git clone https://github.com/NicoBetancourt/person_follower.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```
4. Copy the Webots world file to the ROS package folder
```
sudo cp ~/ros2_ws/src/person_follower/webots/*.wbt \
        /opt/ros/foxy/share/webots_ros2_turtlebot/worlds/.
```
5. Run the person-following node
```
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 run person_follower person_follower 
```
6. In a new terminal, launch the Webots simulator

In a room with walls:
```
export WEBOTS_HOME=~/webots-R2022b
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch webots_ros2_turtlebot robot_launch.py \
  world:=turtlebot3_burger_pedestrian_simple.wbt
```

Or a room without walls:
```
export WEBOTS_HOME=~/webots-R2022b
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch webots_ros2_turtlebot robot_launch.py \
  world:=turtlebot3_burger_pedestrian_no_walls.wbt
```

7. In a new terminal, launch RViz
```
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1
rviz2 -d ~/ros2_ws/src/person_follower/webots/config.rviz
```

## Bot execution

1. Run in the terminal

```
ssh user@192.168.0.22X
```
2. Run in the ssh terminal

```
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2X
ros2 launch kobuki_node kobuki_node-launch.py 
```
3. To operate manually the bot
```
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=2X
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity
```

4. Run the ROS Lidar node:

In an ssh terminal execute the commands:
```
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2X
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar
```
The Lidar should start to turn. In another ssh terminal execute the commands:
```
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=2X
ros2 run tf2_ros static_transform_publisher 0 0 0 3.141592 0 0 base_footprint laser
```
5. To check the scan topic from a terminal
```
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=2X
ros2 topic hz /scan
```
6. To visualize the bot

Download the next file and run in the same folder:

[tb2.rviz](https://aulavirtual.uji.es/pluginfile.php/6837268/mod_resource/content/1/tb2.rviz)
```
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=2X
rviz2 -d tb2.rviz
```
