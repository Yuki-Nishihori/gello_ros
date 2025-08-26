
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    comport = LaunchConfiguration('comport')
    sampling_rate = LaunchConfiguration('sampling_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'comport',
            default_value='/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_2407028-if00',
            description='Path to the COM port'),
        DeclareLaunchArgument(
            'sampling_rate',
            default_value='1200',
            description='Sampling rate'),

        Node(
            package='leptrino_force_torque',
            executable='leptrino_force_torque',
            name='leptrino',
            output='screen',
            parameters=[
                {
                    'com_port': comport,
                    'frame_id': 'leptrino',
                    'rate': sampling_rate
                }
            ]
        ),
        Node(
            package='grinding_force_torque',
            executable='ft_filter.py',
            name='leptrino_ft_filter',
            output='screen',
            arguments=['-z', '-t', 'leptrino/wrench', '-sf', sampling_rate]
        )
    ])
