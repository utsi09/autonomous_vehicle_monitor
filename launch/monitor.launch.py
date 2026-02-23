import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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

    return LaunchDescription([
        use_sim_time_arg,
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