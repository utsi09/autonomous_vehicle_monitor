import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('monitor')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'vehicle.urdf.xml')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    sensors_yaml = os.path.join(pkg_dir, 'config', 'sensors.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # fusion = get_package_share_directory('hesai_zed_fusion')
    # fusion_launch = os.path.join(fusion,'launch','fusion.launch.py')
    # included_fusion = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(fusion_launch),
    #     # launch_arguments=().items()
    # )

    return LaunchDescription([
        use_sim_time_arg,
        # included_fusion,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time
            }],
        ),
        Node(
            package='monitor',
            executable='monitor_node',
            parameters=[sensors_yaml, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])