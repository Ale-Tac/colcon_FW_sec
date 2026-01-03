from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    marker_id = LaunchConfiguration('marker_id').perform(context)
    marker_name = f'aruco_node_{marker_id}'

    aruco_single_params = {
        'image_is_rectified': False,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': LaunchConfiguration('camera_frame'),
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name=marker_name,
        parameters=[aruco_single_params],
        output='screen',
        remappings=[
            ('/camera_info', '/realsense_camera/camera_info'),
            ('/image', '/realsense_camera/image'),
        ],#the /camera/image topic name has been set in a remapping by the start_gazebo_ros_bridge_cmd Node in camera_ignition_gazebo.launch.py 
    )

    return [aruco_single]


def generate_launch_description():

    declare_use_sim_time_cmd = LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above
        ])

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='316',
        description='Marker ID.'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.026',
        description='Marker size in m. '
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_316',
        description='Frame in which the marker pose will be referred. '
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='world',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='camera_link_optical',
        description='Camera frame set at plugin in the model.sdf of the camera'
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement. ',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame_arg)
    ld.add_action(camera_frame_arg)
    ld.add_action(corner_refinement_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld