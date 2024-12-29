import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  # Get the package directory
  package_dir = get_package_share_directory('autonomous_turret')

  # Declare launch arguments
  urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(package_dir, 'models/turret/turret.urdf'))

  # Launch description
  ld = LaunchDescription()

  # Declare launch arguments
  ld.add_action(DeclareLaunchArgument('urdf_file', default_value=urdf_file, description='Path to the URDF file'))

  map_path = (
      os.path.join(
          get_package_share_directory("hexapod_description"),
          "worlds/empty.sdf",
      ),
  )

  # map_path="visualize_lidar.sdf" ### run with empty world

  ignition_args = ["-r", " -v", " 4", " "]
  ignition_args.extend(map_path)


  # Start robot state publisher
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': Command(['xacro', ' ', urdf_file])},
                {'use_sim_time': True}]
  )

  joint_state_publisher = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen'
  )


  gz_ignition = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution(
              [
                  FindPackageShare("autonomous_turret"),
                  "launch",
                  "gazebo_launch.py",
              ]
          ),
      ),
      launch_arguments={
          "gz_version": "6",
          "gz_args": "".join(ignition_args),
      }.items(),
  )
  
  # Start rviz2
  rviz2 = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', os.path.join(package_dir, 'rviz', 'turret.rviz')]
  )

  start_gz_ign_ros_spawner_cmd = Node(
      package="ros_gz_sim",
      executable="create",
      arguments=[
          "-name",
          "turret",
          "-topic",
          "/robot_description",
          "-allow_renaming",
          "true",
          "-x",
          "0",
          "-y",
          "0",
          "-z",
          "0.6",
      ],
      output="screen",
  )

  ld.add_action(gz_ignition)
  ld.add_action(rviz2)
  ld.add_action(robot_state_publisher)
  ld.add_action(joint_state_publisher)
  ld.add_action(start_gz_ign_ros_spawner_cmd)

  return ld