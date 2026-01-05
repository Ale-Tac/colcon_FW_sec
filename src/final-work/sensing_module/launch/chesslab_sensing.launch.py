from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter


def aruco_pipeline_setup(context, *args, **kwargs):
    marker_size = LaunchConfiguration("marker_size").perform(context)
    aruco_ids_str = LaunchConfiguration("aruco_ids").perform(context)
    reference_frame = LaunchConfiguration("reference_frame")
    camera_frame = LaunchConfiguration("camera_frame")

    ids = [int(x.strip()) for x in aruco_ids_str.split(",") if x.strip()]

    camera_image_pipeline_launch = PathJoinSubstitution(
        [FindPackageShare("sensing_module"), "launch", "camera_image_pipeline.launch.py"]
    )

    actions = []

    for m_id in ids:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_image_pipeline_launch),
                launch_arguments={
                    "marker_id": str(m_id),
                    "marker_size": marker_size,
                    "marker_frame": f"aruco_{m_id}",
                    "reference_frame": reference_frame,
                    "camera_frame": camera_frame,
                }.items(),
            )
        )

    return actions


def generate_launch_description():

    # --- argomenti per ArUco ---
    marker_size_arg = DeclareLaunchArgument(
        "marker_size", default_value="0.026", description="Marker size in meters"
    )

    aruco_ids_arg = DeclareLaunchArgument(
        "aruco_ids",
        default_value=(
            "201,202,203,204,205,206,207,208,"
            "209,210,211,212,213,214,215,216,"
            "301,302,303,304,305,306,307,308,"
            "309,310,311,312,313,314,315,316"
        ),
        description="ArUco ID List",
    )

    # *** frame della scacchiera (come nei moduli 1–2) ***
    reference_frame_arg = DeclareLaunchArgument(
        "reference_frame",
        default_value="chess_frame",
        description="Frame della scacchiera",
    )

    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_color_optical_frame",
        description="Frame ottico della camera",
    )

    # --- mondo chesslab + RViz ---
    chesslab_launch = PathJoinSubstitution(
        [FindPackageShare("chesslab_setup2"), "launch", "chesslab_gz.launch.py"]
    )

    chesslab_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(chesslab_launch),
        launch_arguments={"launch_rviz": "true"}.items(),
    )

    aruco_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('aruco_broadcaster'), '/launch', '/aruco_pl2.launch.py']),
            launch_arguments = {
                'use_rviz': 'false',
            }.items(),
    )

    # *** static TF world -> chess_frame ***
    tf_chess_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_chessboard_in_world",
        output="screen",
        arguments=[
            "0.0", "0.0", "0.0",    # x y z
            "0.0", "0.0", "0.0",    # roll pitch yaw
            "world", "chess_frame",
        ],
    )

    # Temporary static TF to ensure camera frame exists for ArUco publisher
    tf_camera_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_camera_in_world",
        output="screen",
        arguments=[
            "0.0", "0.0", "0.0",    # x y z (identity - adjust if needed)
            "0.0", "0.0", "0.0",    # roll pitch yaw
            "world", "camera_color_optical_frame",
        ],
    )

    # --- sensing_node ---
    sensing_params = PathJoinSubstitution(
        [FindPackageShare("sensing_module"), "config", "sensing_node_params.yaml"]
    )

    sensing_node = Node(
        package="sensing_module",
        executable="sensing_node",
        name="sensing_node",
        parameters=[sensing_params],
        output="screen",
    )
    use_sim_time = SetParameter(name="use_sim_time", value=True)

    ld = LaunchDescription()
    ld.add_action(use_sim_time)
    ld.add_action(marker_size_arg)
    ld.add_action(aruco_ids_arg)
    ld.add_action(aruco_broadcaster)
    ld.add_action(reference_frame_arg)
    ld.add_action(camera_frame_arg)
    ld.add_action(chesslab_scene)
    ld.add_action(tf_chess_frame)
    ld.add_action(tf_camera_frame)
    ld.add_action(OpaqueFunction(function=aruco_pipeline_setup))
    ld.add_action(sensing_node)
    return ld