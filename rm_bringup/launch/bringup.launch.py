import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.actions import Node, PushRosNamespace
    from launch.actions import TimerAction
    from launch import LaunchDescription
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

    # 构建传递给 URDF 的参数列表（前相机 + 后相机）
    xacro_args = []
    # 前相机外参
    if 'odom2camera0' in launch_params:
        xacro_args.extend([' camera0_xyz:=', launch_params['odom2camera0']['xyz'],
                           ' camera0_rpy:=', launch_params['odom2camera0']['rpy']])
    # 后相机外参
    if 'odom2camera1' in launch_params:
        xacro_args.extend([' camera1_xyz:=', launch_params['odom2camera1']['xyz'],
                           ' camera1_rpy:=', launch_params['odom2camera1']['rpy']])

    # 云台 TF 描述文件（包含前后相机坐标系）
    robot_gimbal_description = Command(
        ['xacro ', os.path.join(
            get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro')] + xacro_args
    )
    
    # 导航 TF 描述文件（可选）
    robot_navigation_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])

    # 云台状态发布器
    robot_gimbal_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_gimbal_description,
                    'publish_frequency': 1000.0}]
    )
    
    # 导航状态发布器（仅当 navigation 为 true 时启动）
    robot_navigation_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_navigation_description}]
    )

    def get_params(name):
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))

    # 前相机装甲板识别节点
    front_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector_front',
        namespace='front',
        output='both',
        parameters=[get_params('armor_detector_front')],
        remappings=[
            ('image_raw', '/camera_0/image_raw'),
            ('camera_info', '/camera_0/camera_info'),
            ('armor_detector/armors', '/front/armor_detector/armors')
        ]
    )

    # 后相机装甲板识别节点
    rear_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector_rear',
        namespace='rear',
        output='both',
        parameters=[get_params('armor_detector_rear')],
        remappings=[
            ('image_raw', '/camera_1/image_raw'),
            ('camera_info', '/camera_1/camera_info'),
            ('armor_detector/armors', '/rear/armor_detector/armors')
        ]
    )

    # 装甲板解算节点（多线程容器）
    if launch_params['hero_solver']:
        # 如果使用英雄解算（普通节点），保持原样
        armor_solver_container = Node(
            package='hero_armor_solver',
            executable='hero_armor_solver_node',
            name='armor_solver',
            output='both',
            parameters=[get_params('armor_solver')]
        )
    else:
        # 使用标准解算，放入多线程容器以获得并行处理能力
        armor_solver_composable = ComposableNode(
            package='armor_solver',
            plugin='fyt::auto_aim::ArmorSolverNode',
            name='armor_solver',
            parameters=[get_params('armor_solver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        armor_solver_container = ComposableNodeContainer(
            name='solver_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',   # 多线程执行器
            composable_node_descriptions=[armor_solver_composable],
            output='both',
            emulate_tty=True
        )

    # 延迟启动，确保 TF 发布器先就绪
    delay_front_detector = TimerAction(period=2.0, actions=[front_detector_node])
    delay_rear_detector = TimerAction(period=2.5, actions=[rear_detector_node])
    delay_solver = TimerAction(period=3.0, actions=[armor_solver_container])

    push_namespace = PushRosNamespace(launch_params['namespace'])
    
    # 组装启动描述列表
    launch_description_list = [
        robot_gimbal_publisher,
        push_namespace,
        delay_front_detector,
        delay_rear_detector,
        delay_solver
    ]
    
    # 如果启用导航，添加导航 TF 发布器
    if launch_params['navigation']:
        launch_description_list.append(robot_navigation_publisher)
    
    return LaunchDescription(launch_description_list)