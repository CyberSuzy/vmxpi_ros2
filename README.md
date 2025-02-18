# vmxpi_ros2

### Dependency
```bash
sudo apt install ros-humble-backward-ros
sudo apt install ros-humble-hardware-interface
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-plugins
sudo apt install ros-humble-xacro
```


for  

ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py gui:=true use_gazebo_classic:=true


in VMX 
```bash
cd ~/ros_ws
sudo su 
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_hardware:=true
```

update source file for root user
```bash
cp /etc/skel/.bash* ~
```
add the following lines in .profile

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
```

To cotrol the robot

ros2 topic pub --rate 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.5
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5"



ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True --remap cmd_vel:=/diffbot_base_controller/cmd_vel

Depug 

ros2 topic echo /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped

ros2 topic echo /joint_states

ros2 topic info -v /diffbot_base_controller/cmd_vel