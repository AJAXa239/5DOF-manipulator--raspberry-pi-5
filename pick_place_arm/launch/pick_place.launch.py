from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================================================
    # EXECUTION PARAMETERS
    # =========================================================

    execute_arg = DeclareLaunchArgument(
        "execute",
        default_value="false",
        description="Execute the full pick and place sequence"
    )

    execute_initial_arg = DeclareLaunchArgument(
        "execute_initial",
        default_value="false",
        description="Execute only the initial position"
    )

    execute_pick_arg = DeclareLaunchArgument(
        "execute_pick",
        default_value="false",
        description="Execute only the pick position and close gripper"
    )

    execute_place_arg = DeclareLaunchArgument(
        "execute_place",
        default_value="false",
        description="Execute only the place position and open gripper"
    )

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    arm_pkg = "robotic_arm_description"
    moveit_pkg = "robotic_arm_moveit"

    arm_share = get_package_share_directory(arm_pkg)
    moveit_share = get_package_share_directory(moveit_pkg)

    xacro_file = os.path.join(
        arm_share,
        "urdf",
        "robotic_arm.urdf.xacro"
    )

    srdf_file = os.path.join(
        moveit_share,
        "config",
        "robotic_arm.srdf"
    )

    joint_limits_file = os.path.join(
        moveit_share,
        "config",
        "joint_limits.yaml"
    )

    kinematics_file = os.path.join(
        moveit_share,
        "config",
        "kinematics.yaml"
    )

    rviz_file = os.path.join(
        moveit_share,
        "config",
        "moveit.rviz"
    )

    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                xacro_file
            ]),
            value_type=str
        )
    }

    # =========================================================
    # SEMANTIC DESCRIPTION
    # =========================================================

    with open(srdf_file) as f:
        srdf_content = f.read()

    robot_description_semantic = {
        "robot_description_semantic": srdf_content
    }

    # =========================================================
    # JOINT LIMITS
    # =========================================================

    with open(joint_limits_file) as f:
        joint_limits = yaml.safe_load(f)

    robot_description_planning = {
        "robot_description_planning": joint_limits
    }

    # =========================================================
    # KINEMATICS
    # =========================================================

    with open(kinematics_file) as f:
        kinematics_yaml = yaml.safe_load(f)

    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    # =========================================================
    # OMPL PLANNING
    # =========================================================

    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",

        "ompl": {
            "planning_plugins": [
                "ompl_interface/OMPLPlanner"
            ],

            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
                "default_planning_request_adapters/CheckForStackedConstraints",
            ],

            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],

            "start_state_max_bounds_error": 0.1,
        }
    }

    # =========================================================
    # MOVEIT CONTROLLERS
    # =========================================================

    moveit_simple_controller_manager = {
        "moveit_simple_controller_manager": {
            "controller_names": [
                "arm_controller",
                "gripper_controller"
            ],

            "arm_controller": {
                "type": "FollowJointTrajectory",
                "joints": [
                    "joint_1",
                    "joint_2",
                    "joint_3",
                    "joint_4",
                    "joint_5"
                ],
                "action_ns": "follow_joint_trajectory",
                "default": True,
            },

            "gripper_controller": {
                "type": "FollowJointTrajectory",
                "joints": [
                    "joint_6"
                ],
                "action_ns": "follow_joint_trajectory",
                "default": True,
            },
        }
    }

    # =========================================================
    # COMMON MOVEIT PARAMETERS
    # =========================================================

    moveit_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_planning,
        robot_description_kinematics,
        planning_pipelines,
        moveit_simple_controller_manager,

        {
            "moveit_controller_manager":
                "moveit_simple_controller_manager/MoveItSimpleControllerManager",

            "use_sim_time": False
        }
    ]

    # =========================================================
    # MOVE GROUP
    # =========================================================

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_parameters
    )

    # =========================================================
    # PICK AND PLACE NODE
    # =========================================================

    pick_place_node = Node(
        package="pick_place_arm",
        executable="pick_place_node",
        output="screen",

        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            robot_description_kinematics,
            planning_pipelines,
            moveit_simple_controller_manager,

            {
                "moveit_controller_manager":
                    "moveit_simple_controller_manager/MoveItSimpleControllerManager",

                "use_sim_time": False,

                "execute":
                    LaunchConfiguration("execute"),

                "execute_initial":
                    LaunchConfiguration("execute_initial"),

                "execute_pick":
                    LaunchConfiguration("execute_pick"),

                "execute_place":
                    LaunchConfiguration("execute_place"),
            }
        ]
    )

    # =========================================================
    # RVIZ
    # =========================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",

        arguments=[
            "-d",
            rviz_file
        ],

        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning
        ]
    )

    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([

        execute_arg,
        execute_initial_arg,
        execute_pick_arg,
        execute_place_arg,

        move_group_node,
        pick_place_node,
        rviz_node,
    ])
