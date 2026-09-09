from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arm_pkg = 'robotic_arm_description'
    moveit_pkg = 'robotic_arm_moveit'

    xacro_file = PathJoinSubstitution(
        [FindPackageShare(arm_pkg), 'urdf', 'robotic_arm.urdf.xacro']
    )

    srdf_file = os.path.join(
        get_package_share_directory(moveit_pkg),
        'config',
        'robotic_arm.srdf'
    )

    joint_limits_file = os.path.join(
        get_package_share_directory(moveit_pkg),
        'config',
        'joint_limits.yaml'
    )

    rviz_file = os.path.join(
        get_package_share_directory(moveit_pkg),
        'config',
        'moveit.rviz'
    )

    # Robot description for MoveIt.
    # The real hardware controller runs on the Raspberry Pi.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    # Semantic robot description.
    with open(srdf_file) as f:
        srdf_content = f.read()

    robot_description_semantic = {
        'robot_description_semantic': srdf_content
    }

    # Joint limits.
    with open(joint_limits_file) as f:
        joint_limits = yaml.safe_load(f)

    robot_description_planning = {
        'robot_description_planning': joint_limits
    }

    # Kinematics.
    kinematics_yaml = {
        'robot_description_kinematics': {
            'arm': {
                'kinematics_solver':
                    'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.005,
                'kinematics_solver_timeout': 0.05,
            },
            'gripper': {
                'kinematics_solver':
                    'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.005,
                'kinematics_solver_timeout': 0.05,
            },
        }
    }

    # Planning pipelines.
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                'default_planning_request_adapters/CheckForStackedConstraints',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }

    # OMPL planner configurations.
    ompl_planner_configs = {
        'arm': {
            'default_planner_config': 'RRTConnect',
            'planner_configs': [
                'RRTConnect',
                'RRT',
                'RRTstar',
                'TRRT',
                'PRM',
                'PRMstar',
            ],
            'projection_evaluator': 'joints(joint_1,joint_2)',
            'longest_valid_segment_fraction': 0.005,
        },
        'planner_configs': {
            'RRTConnect': {
                'type': 'geometric::RRTConnect',
                'range': 0.0,
            },
            'RRT': {
                'type': 'geometric::RRT',
                'range': 0.0,
                'goal_bias': 0.05,
            },
            'RRTstar': {
                'type': 'geometric::RRTstar',
                'range': 0.0,
                'goal_bias': 0.05,
                'delay_collision_checking': 1,
            },
            'TRRT': {
                'type': 'geometric::TRRT',
                'range': 0.0,
                'goal_bias': 0.05,
            },
            'PRM': {
                'type': 'geometric::PRM',
                'max_nearest_neighbors': 10,
            },
            'PRMstar': {
                'type': 'geometric::PRMstar',
            },
        },
    }

    # MoveIt sends trajectories to the controllers
    # running on the Raspberry Pi.
    moveit_simple_controller_manager = {
        'moveit_simple_controller_manager': {
            'controller_names': [
                'arm_controller',
                'gripper_controller',
            ],

            'arm_controller': {
                'type': 'FollowJointTrajectory',
                'joints': [
                    'joint_1',
                    'joint_2',
                    'joint_3',
                    'joint_4',
                    'joint_5',
                ],
                'action_ns': 'follow_joint_trajectory',
                'default': True,
            },

            'gripper_controller': {
                'type': 'FollowJointTrajectory',
                'joints': ['joint_6'],
                'action_ns': 'follow_joint_trajectory',
                'default': True,
            },
        }
    }

    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_planning,
        kinematics_yaml,
        planning_pipelines,
        ompl_planner_configs,
        moveit_simple_controller_manager,

        {
            'moveit_controller_manager':
                'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        },

        {
            'use_sim_time': False
        },
    ]

    # MoveIt planning node.
    # This runs ONLY on the Pop!_OS laptop.
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    # RViz runs ONLY on the Pop!_OS laptop.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return LaunchDescription([
        move_group,
        rviz,
    ])
