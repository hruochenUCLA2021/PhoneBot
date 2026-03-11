import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('robot_visualizer')
    default_model = os.path.join(
        pkg_share, 'model', 'model_phonebot',
        'scene_joystick_flat_terrain_alternative_imu.xml',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes',
    )
    ld.add_action(namespace_arg)
    namespace = LaunchConfiguration('namespace')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='Absolute path to the MuJoCo XML model',
    )
    ld.add_action(model_path_arg)

    render_hz_arg = DeclareLaunchArgument(
        'render_hz',
        default_value='30.0',
        description='Viewer render rate in Hz',
    )
    ld.add_action(render_hz_arg)

    visualizer_node = Node(
        package='robot_visualizer',
        namespace=namespace,
        executable='phonebot_visualizer',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'render_hz': LaunchConfiguration('render_hz'),
        }],
    )
    ld.add_action(visualizer_node)

    return ld
