# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "use_gazebo_classic",
            default_value="false",
            description="Start robot with Gazebo Classic mirroring command to its states.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gazebo_classic = LaunchConfiguration("use_gazebo_classic")
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') # Default to false for hardware
 
    world_file = PathJoinSubstitution([FindPackageShare("vmxpi_ros2"), "description/gazebo/worlds", "diff_drive_world.world"])

# Gazebo Configration 
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_vmxpi_ros2_gazebo = get_package_share_directory('vmxpi_ros2')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "vmxpi_ros2"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_vmxpi_ros2_gazebo, 'description/models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={
            "verbose": "false",
            "world": world_file,
            "use_sim_time": LaunchConfiguration('use_sim_time', default='true'), # Explicitly set to true for Gazebo
        }.items(),
        condition=IfCondition(use_gazebo_classic),
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("vmxpi_ros2"), "description/urdf", "diffbot.urdf.xacro"]),
            " ",
            "use_gazebo_classic:=", use_gazebo_classic,
            " ",
            "use_hardware:=", use_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content, "use_sim_time": use_sim_time} 

    rviz_config_file = PathJoinSubstitution([FindPackageShare("vmxpi_ros2"), "description/diffbot/rviz", "diffbot.rviz"])
          
    robot_controllers = PathJoinSubstitution([FindPackageShare("vmxpi_ros2"), "config", "diffbot_controllers.yaml"])

    

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "diffbot_system_position"],
        output="screen",
        condition=IfCondition(use_gazebo_classic),
    )

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         robot_description,
    #         PathJoinSubstitution([FindPackageShare("vmxpi_ros2"), "config", "diffbot_controllers.yaml"]),
    #         {"use_sim_time": use_sim_time},
    #     ],
    #     output="screen",
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    control_node = Node(
        package='controller_manager', # Or the correct package name for your control_node executable
        executable='ros2_control_node', # Or the correct executable name
        namespace='',
        parameters=[robot_description, robot_controllers], # robot_controllers is your PathJoinSubstitution to the YAML file
        output='screen',
        condition=UnlessCondition(use_gazebo_classic) # Launch only when not using Gazebo Classic
    )


    nodes = [
        gazebo,
        control_node,
        node_robot_state_publisher,
        spawn_entity,
        # controller_manager,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)