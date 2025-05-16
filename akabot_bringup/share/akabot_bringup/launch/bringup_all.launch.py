# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.substitutions import Command, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# import os

# def generate_launch_description():
#     # Paths
#     description_pkg = FindPackageShare("akabot_description")
#     gazebo_pkg = FindPackageShare("akabot_gazebo")
#     moveit_pkg = FindPackageShare("akabot_moveit_config")

#     urdf_file = PathJoinSubstitution([
#         description_pkg,
#         "urdf",
#         "akabot.urdf.xacro"
#     ])

#     # Robot State Publisher
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="screen",
#         parameters=[{
#             "robot_description": Command(["xacro ", urdf_file])
#         }]
#     )

#     # Joint State Publisher GUI (optional)
#     joint_state_publisher = Node(
#         package="joint_state_publisher_gui",
#         executable="joint_state_publisher_gui",
#         name="joint_state_publisher_gui",
#         output="screen"
#     )

#     # Gazebo launch
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             gazebo_pkg, "/launch/gz_launch.py"
#         ])
#     )

#     # MoveIt launch (handles RViz internally)
#     moveit_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             moveit_pkg, "/launch/demo.launch.py"
#         ])
#     )

#     return LaunchDescription([
#         robot_state_publisher,
#         joint_state_publisher,
#         gazebo_launch,
#         moveit_launch  # MoveIt already launches RViz
#     ])


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Paths
    description_pkg = FindPackageShare("akabot_description")
    gazebo_pkg = FindPackageShare("akabot_gazebo")
    moveit_pkg = FindPackageShare("akabot_moveit_config")

    urdf_file = PathJoinSubstitution([
        description_pkg,
        "urdf",
        "akabot.urdf.xacro"
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_file])
        }]
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_pkg, "/launch/gz_launch.py"
        ])
    )

    # MoveIt launch (handles RViz internally)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            moveit_pkg, "/launch/demo.launch.py"
        ])
    )

    return LaunchDescription([
        robot_state_publisher,
        # joint_state_publisher,  # Removed the duplicate Joint State Publisher GUI
        gazebo_launch,
        moveit_launch  # MoveIt already launches RViz
    ])

