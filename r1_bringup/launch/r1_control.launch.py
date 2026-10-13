import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------- 获取包路径 ----------
    r1_bringup_pkg = get_package_share_directory('r1_bringup')
    r1_description_pkg = get_package_share_directory('r1_description')

    # ---------- 文件路径 ----------
    controllers_yaml = os.path.join(
        r1_bringup_pkg, 'config', 'r1_controllers.yaml')
    xacro_file = os.path.join(
        r1_description_pkg, 'urdf', 'r1.urdf.xacro')

    # ---------- 用 xacro 展开 URDF ----------
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ============================================================
    # 1. robot_state_publisher
    #    负责根据 /joint_states 发布 TF
    # ============================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ============================================================
    # 2. ros2_control_node
    #    controller_manager 节点，加载硬件接口和控制器
    # ============================================================
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_yaml,
            {'robot_description': robot_description},
        ],
        output='screen'
    )

    # ============================================================
    # 3. 控制器 spawner
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen'
    )

    # ============================================================
    # 4. 启动顺序控制
    #    等 joint_state_broadcaster 完成后，再启动 diff_drive_controller
    # ============================================================
    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive,
    ])