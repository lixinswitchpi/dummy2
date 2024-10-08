# adapt to dummy2 by Muzhxiaowen, check out the details on bilibili.com

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("dummy2", package_name="dummy_moveit_config").to_moveit_configs()
    moveit_config = (
        MoveItConfigsBuilder("dummy2", package_name="dummy2_moveit_config")
        .robot_description(file_path="config/dummy2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )


    # Get parameters for the Servo node
    servo_params = (
        ParameterBuilder("dummy2_moveit_config")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/dummy2_simulated_config.yaml",
        )
        .to_dict()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # A node to publish world -> base_link transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["-0.45", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    servo_node = Node(
        package="dummy2_moveit_config",
        executable="dummy2_servo_cpp_interface_demo",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # tutorial_node = Node(
    #     package="dummy_moveit_config",
    #     executable="dummy_pose_sets",
    #     output="screen",
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #     ],
    # )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("dummy2_moveit_config"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit_empty.rviz")

    # rviz_config_file = (
    #     get_package_share_directory("moveit2_tutorials")
    #     + "/config/demo_rviz_config.rviz"
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("dummy2_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in ["dummy2_arm_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [rviz_node, static_tf, servo_node, run_move_group_node, ros2_control_node, robot_state_publisher]
        + load_controllers
    )
