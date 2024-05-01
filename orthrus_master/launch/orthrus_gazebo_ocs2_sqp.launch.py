import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

use_sim_time = LaunchConfiguration('use_sim_time', default='true')

#rviz_config_file = get_package_share_directory('ocs2_legged_robot_ros') + "/rviz/legged_robot.rviz"

ArgumentDescriptionName = launch.actions.DeclareLaunchArgument(
    name='description_name',
    default_value='legged_robot_description'
)

ArgumentMultiplot = launch.actions.DeclareLaunchArgument(
    name='multiplot',
    default_value='false'
)

ArgumentTaskFile = launch.actions.DeclareLaunchArgument(
    name='taskFile',
    default_value=get_package_share_directory(
        'orthrus_controllers') + '/config/mpc/task.info'
)

ArgumentReferenceFile = launch.actions.DeclareLaunchArgument(
    name='referenceFile',
    default_value=get_package_share_directory(
        'orthrus_controllers') + '/config/command/reference.info'
)

ArgumentUrdfFile = launch.actions.DeclareLaunchArgument(
    name='urdfFile',
    default_value=get_package_share_directory(
        'orthrus_interfaces') + '/models/orthrus/urdf/orthrus.urdf'
    #default_value=get_package_share_directory(
    #    'orthrus_interfaces') + '/models/a1/urdf/robot.urdf'
    #default_value=get_package_share_directory(
    #    'ocs2_robotic_assets') + '/resources/anymal_c/urdf/anymal.urdf'


    
)

ArgumentGaitCommandFile = launch.actions.DeclareLaunchArgument(
    name='gaitCommandFile',
    default_value=get_package_share_directory(
        'orthrus_controllers') + '/config/command/gait.info'
)

def get_orthrus_control(package, executable, name):

    return launch_ros.actions.Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[{'use_sim_time': use_sim_time},
                    {'taskFile': launch.substitutions.LaunchConfiguration('taskFile')},
                    {'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')},
                    {'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')},
                    {'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')},
                    ],
    )

def get_orthrus_gazebo(package, executable, name):
    return launch_ros.actions.Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[{'use_sim_time': use_sim_time}],
    )

def get_orthrus_gazebo_sim(package, executable):
    return launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package), 'launch/'),executable])
    )

def generate_launch_description():

    orthrus_control_node = get_orthrus_control('orthrus_controllers', 'orthrus_controllers','orthrus_controller_node')
    orthrus_gazebo_node = get_orthrus_gazebo('orthrus_gazebo', 'orthrus_gazebo','orthrus_gazebo_node')
    orthrus_gazebo_sim = get_orthrus_gazebo_sim('orthrus_gazebo', 'orthrus_sim.launch.py')

    ld = launch.LaunchDescription([
        ArgumentDescriptionName,
        ArgumentMultiplot,
        ArgumentTaskFile,
        ArgumentReferenceFile,
        ArgumentUrdfFile,
        ArgumentGaitCommandFile,
        ###
        #orthrus_gazebo_node,
        #orthrus_gazebo_sim,

        orthrus_control_node,
    ])

    return ld

if __name__ == '__main__':
    generate_launch_description()
