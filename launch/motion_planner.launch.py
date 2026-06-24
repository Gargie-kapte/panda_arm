import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("panda_description"),
                "urdf", "panda.urdf.xacro"
            ]),
            " name:=panda",
        ]),
        value_type=str
    )

    robot_description_semantic = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("panda_moveit_config"),
                "srdf", "panda.srdf.xacro"
            ]),
            " name:=panda",
        ]),
        value_type=str
    )

    # Load kinematics.yaml
    kinematics_yaml = os.path.join(
        get_package_share_directory("panda_moveit_config"),
        "config", "kinematics.yaml"
    )
    with open(kinematics_yaml, "r") as f:
        kinematics = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package="pick_place",
            executable="motion_planner",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"robot_description_semantic": robot_description_semantic},
                {"use_sim_time": True},
                kinematics,
            ],
        ),

    ])