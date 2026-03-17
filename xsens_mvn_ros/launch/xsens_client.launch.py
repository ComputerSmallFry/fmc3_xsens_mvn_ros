from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('xsens_mvn_ros')

    model_name_arg = DeclareLaunchArgument(
        'model_name', default_value='skeleton')
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='world')
    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='8001')
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_share, 'rviz', 'xsens_visualization.rviz'))

    xsens_node = Node(
        package='xsens_mvn_ros',
        executable='xsens_client',
        name='xsens',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'udp_port': LaunchConfiguration('udp_port'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='xsens_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        model_name_arg,
        reference_frame_arg,
        udp_port_arg,
        launch_rviz_arg,
        rviz_config_arg,
        xsens_node,
        rviz_node,
    ])
