from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Аргументы launch-файла
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    # Путь к конфиг-файлу
    vslam_config = os.path.join(
        get_package_share_directory("orbslam3"), 'config', 'monocular', 'Arthur.yaml'
    )

    vslam_vocabulary = os.path.join(
        get_package_share_directory("orbslam3"), 'vocabulary', 'ORBvoc.txt'
    )

    # Основная нода C++
    cpp_node = Node(
        package='orbslam3',
        executable='mono',
        arguments=[
            vslam_vocabulary,
            vslam_config],
        output='screen',
    )


    return LaunchDescription([
        use_sim_time,
        cpp_node,
    ])