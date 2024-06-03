from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch/'), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch/'), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
    ])