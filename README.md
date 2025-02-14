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


To cotrol the robot

ros2 topic pub --rate 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.8
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