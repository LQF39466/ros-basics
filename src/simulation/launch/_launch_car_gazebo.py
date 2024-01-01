import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # Configure variables
    package_name = 'simulation'
    model_path = 'car_gazebo.urdf'

    # Setup project paths
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    abs_model_path = os.path.join(pkg_share, f'urdf/{model_path}')

    # Declare rviz config
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(os.path.join(pkg_share ,'rviz/urdf.rviz')),
        description='Absolute path to rviz config file'
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': os.path.join(
            pkg_share,
            'gazebo/worlds',
            'car.sdf'
        )}.items(),
    )
    
    # For publishing and controlling the robot pose, we need joint states of the robot
    # Configure the robot model by adjusting the joint angles using the GUI slider
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[abs_model_path],
        output=['screen']
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro ', abs_model_path])}
        ]
    )

    # Visualize in RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        rviz_arg,
        bridge,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz
    ])