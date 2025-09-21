import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    urdf_file_name = "Robot.urdf"

    urdf = os.path.join(
        get_package_share_directory("my_robot_description"),
        "urdf",
        urdf_file_name,
    )

    # URDF 읽기
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    rviz_config_file = os.path.join(
        get_package_share_directory("my_robot_description"),
        "rviz",
        "display.rviz",
    )

    # # Gazebo world (기본 world 사용)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("gazebo_ros"),
    #             "launch",
    #             "gazebo.launch.py"
    #         )
    #     ),
    #     launch_arguments={"use_sim_time": use_sim_time}.items(),
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),

            # 로봇 상태 퍼블리셔
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_desc,
                }],
            ),

            # 조인트 상태 퍼블리셔 GUI
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),

            # RViz 실행
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
            ),

            # # Gazebo 실행
            # gazebo_launch,

            # # Gazebo에 로봇 스폰
            # Node(
            #     package="gazebo_ros",
            #     executable="spawn_entity.py",
            #     arguments=["-topic", "robot_description", "-entity", "robot"],
            #     output="screen",
            # ),
        ]
    )
