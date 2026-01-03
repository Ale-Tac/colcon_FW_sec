from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

  yaml_config = LaunchConfiguration('yaml_config')
  use_rviz_launch_arg = LaunchConfiguration('use_rviz')
  rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')

  aruco_broadcaster_node = Node(
      package='aruco_broadcaster',
      executable='aruco_broadcaster_node',
      name='aruco_broadcaster_node',
      # arguments=[],
      parameters=[yaml_config],
      output={'both': 'screen'},
  )

  rviz2_node = Node(
    condition=IfCondition(use_rviz_launch_arg),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=[
      '-d', rvizconfig_launch_arg,
    ],
    parameters=[{
        # 'use_sim_time': use_sim_time_param,
    }],
    output={'both': 'log'},
    )

  static_transform_publisher = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='tf_camera_to_world',
      # the values obtained must be here. For ex.
      #arguments=["-0.021566", "-0.303249", "0.505499", "-0.362908", "0.361961", "0.608962", "0.605349", "world", "camera_link"],
      #arguments=["-0.0238978", "-0.310295", "0.542408", "0.967736", "-0.000215312", "0.00222546", "-0.0251955", "world", "camera_link"],
      arguments=["-0.021", "-0.291", "0.505499", "-0.96959", "-0.00228", "0.001333", "0.244721", "world", "camera_link"],
      output={'both': 'screen'},
  )

  return [
      aruco_broadcaster_node,
      rviz2_node,
      static_transform_publisher,
  ]


def generate_launch_description():
  declared_arguments = []
  
  declared_arguments.append(
    DeclareLaunchArgument(
      'yaml_config',
      default_value= os.path.join(
        get_package_share_directory('aruco_broadcaster'),
        'config',
        'pl2esaii.yaml'
      ),
      description=('Absolute path to yaml config file.')
    )
  )

  declared_arguments.append(
    DeclareLaunchArgument(
      'use_rviz',
      default_value='true',
      choices=('true', 'false'),
      description='launches RViz if set to `true`.',
    )
  )

  declared_arguments.append(
    DeclareLaunchArgument(
      'rvizconfig',
      default_value=PathJoinSubstitution([
        FindPackageShare('aruco_broadcaster'),
        'config',
        'aruco_mapping.rviz',
      ]),
      description='file path to the config file RViz should load.',
    )
  )

  return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])