from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # Uncomment if only rviz will be used
    gui_arg = DeclareLaunchArgument(name='gui', default_value='True',description='Flag to enable joint_state_publisher_gui')

    tf_prefix = LaunchConfiguration('tf_prefix', default='')

    # UR description
    # Type/series of used UR robot.
    # choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"]
    ur_type = LaunchConfiguration('ur_type', default='ur3e')
    description_package = LaunchConfiguration('description_package', default='chesslab_setup2')
    description_file = LaunchConfiguration('description_file', default='chesslab.urdf.xacro')

    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("chesslab_setup2"), "rviz", "chesslab_setup2_demo.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    #robot_description = {"robot_description": robot_description_content}
    #https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],      
    )

    joint_state_publisher = Node(
        condition = UnlessCondition(LaunchConfiguration('gui')),
            name='joint_state_publisher',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[robot_description, {'publish_default_positions': False}, {'source_list': ['ur_joint_states']}],
    )

    joint_state_publisher_gui = Node(
        condition = IfCondition(LaunchConfiguration('gui')),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )

    # Description
    ld =  LaunchDescription()

    ld.add_action(gui_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)

    return ld
