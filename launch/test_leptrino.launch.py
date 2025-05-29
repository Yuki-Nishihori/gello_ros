from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # === Declare launch arguments ===
    declared_args = [
        DeclareLaunchArgument(
            'comport',
            default_value='/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_2407028-if00'
        ),
        DeclareLaunchArgument(
            'sampling_rate',
            default_value='1200'
        ),
    ]

    # === LaunchConfigurations ===
    comport = LaunchConfiguration('comport')
    sampling_rate = LaunchConfiguration('sampling_rate')

    # === Leptrino force torque sensor node ===
    leptrino_node = Node(
        package='leptrino_force_torque',
        executable='leptrino_force_torque',
        name='leptrino',
        output='screen',
        parameters=[{
            'com_port': comport,
            'frame_id': 'leptrino',
            'rate': sampling_rate
        }]
    )

    # === Force torque filter node ===
    ft_filter_node = Node(
        package='grinding_force_torque',
        executable='ft_filter.py',  # 注意：Pythonスクリプトの場合、entry_pointsが必要です
        name='leptrino_ft_filter',
        output='screen',
        arguments=['-z', '-t', 'leptrino/wrench', '-sf', sampling_rate]
    )

    return LaunchDescription(declared_args + [
        leptrino_node,
        ft_filter_node
    ])
