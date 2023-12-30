import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Configure variables
    package_name = 'simulation'
    model_path = 'car.urdf'

    # Setup project paths
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    abs_model_path = os.path.join(pkg_share, f'urdf/{model_path}')

    # Declare rviz config
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(os.path.join(pkg_share ,'rviz/urdf.rviz')),
        description='Absolute path to rviz config file'
    )

    # Create ROS Nodes
    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_descrption': Command(['xacro ', abs_model_path])}],
        arguments=[abs_model_path]
    )

    # For publishing and controlling the robot pose, we need joint states of the robot
    # Configure the robot model by adjusting the joint angles using the GUI slider
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[abs_model_path]
    )

    # Visualize in RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        rviz_arg,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz
    ])