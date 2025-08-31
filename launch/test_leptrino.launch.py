
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    comport = LaunchConfiguration('comport')
    sampling_rate = LaunchConfiguration('sampling_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'comport',
            default_value='/dev/ttyACM0',
            description='Path to the COM port'
        ),
        DeclareLaunchArgument(
            'sampling_rate',
            default_value='1200.0',
            description='Sampling rate'
        ),

        ComposableNodeContainer(
            name='leptrino_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='leptrino_force_torque',
                    plugin='LeptrinoNode',
                    name='leptrino_force_torque_node',
                    namespace='leptrino',
                    parameters=[{
                        'com_port': comport,
                        'frame_id': 'leptrino_frame',
                        'rate': sampling_rate
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='grinding_force_torque',
                    plugin='grinding_force_torque::WrenchFilter',
                    name='leptrino_wrench_filter',
                    namespace='leptrino',
                    parameters=[{
                        'input_topic': '/leptrino/wrench',
                        'output_topic': '/leptrino/wrench/filtered',
                        'sampling_frequency': sampling_rate,
                        'cutoff_frequency': 2.5,
                        'filter_order': 3,
                        'data_window': 100,
                        'initial_zero': True,
                        'disable_filtering': False
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            parameters=[{'use_intra_process_comms': True}],
            output='screen',
        )
    ])
