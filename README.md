# vmxpi_ros2


for testing Diffbot with Gazebo

ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_gazebo_classic:=true gui:=true


To cotrol the robot

ros2 topic pub --rate 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5"



ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diffbot_base_controller/cmd_vel