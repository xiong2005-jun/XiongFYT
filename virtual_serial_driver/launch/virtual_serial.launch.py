from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明可配置参数
    declare_vision_mode_arg = DeclareLaunchArgument(
        'vision_mode',
        default_value='0',
        description='Vision mode (0: blue, 1: red)'
    )
    declare_roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Gimbal roll angle (degree)'
    )
    declare_pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='2.5',
        description='Gimbal pitch angle (degree)'
    )
    declare_yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='-5.0',
        description='Gimbal yaw angle (degree)'
    )
    declare_bullet_speed_arg = DeclareLaunchArgument(
        'bullet_speed',
        default_value='25.0',
        description='Bullet speed (m/s)'
    )
    declare_has_rune_arg = DeclareLaunchArgument(
        'has_rune',
        default_value='true',
        description='Whether to enable rune module'
    )
    declare_target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='odom',
        description='Parent TF frame'
    )

    # 定义组件化节点
    virtual_serial_node = ComposableNode(
        package='virtual_serial_driver',
        plugin='fyt::serial_driver::VirtualSerialNode',  # 组件类名（必须和代码中一致）
        name='serial_driver',
        parameters=[{
            'vision_mode': LaunchConfiguration('vision_mode'),
            'roll': LaunchConfiguration('roll'),
            'pitch': LaunchConfiguration('pitch'),
            'yaw': LaunchConfiguration('yaw'),
            'bullet_speed': LaunchConfiguration('bullet_speed'),
            'has_rune': LaunchConfiguration('has_rune'),
            'target_frame': LaunchConfiguration('target_frame')
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 创建组件容器（加载组件化节点）
    container = ComposableNodeContainer(
        name='serial_driver_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # 多线程容器
        composable_node_descriptions=[virtual_serial_node],
        output='screen',
        emulate_tty=True
    )

    # 组装启动描述
    return LaunchDescription([
        declare_vision_mode_arg,
        declare_roll_arg,
        declare_pitch_arg,
        declare_yaw_arg,
        declare_bullet_speed_arg,
        declare_has_rune_arg,
        declare_target_frame_arg,
        PushRosNamespace(''),  # 空命名空间（可根据需要修改）
        container
    ])
