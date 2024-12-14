from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value = "handrobot.urdf.xacro",
            description = "URDF/XACRO description file with the robot."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value = "true",
            description = "Start Rviz2 and Joint State Publisher gui automatically."
        )
    )

    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("handrobot_ros2_control"), "urdf", description_file])
        ]
    )
    robot_description = {"robot_description:", robot_description_content}

    joint_state_publisher_node = Node (
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
        condition = IfCondition(gui)
    )

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [robot_description]
    )

    rviz_node = Node (
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "log",
        condition = IfCondition(gui)
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]
    
    return LaunchDescription(declared_arguments + nodes)