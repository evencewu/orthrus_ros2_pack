import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('orthrus_master'), 'launch'))

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
        'ocs2_legged_robot') + '/config/mpc/task.info'
)

ArgumentReferenceFile = launch.actions.DeclareLaunchArgument(
    name='referenceFile',
    default_value=get_package_share_directory(
        'ocs2_legged_robot') + '/config/command/reference.info'
)

ArgumentUrdfFile = launch.actions.DeclareLaunchArgument(
    name='urdfFile',
    default_value=get_package_share_directory(
        'ocs2_robotic_assets') + '/resources/anymal_c/urdf/anymal.urdf'
)

ArgumentGaitCommandFile = launch.actions.DeclareLaunchArgument(
    name='gaitCommandFile',
    default_value=get_package_share_directory(
        'ocs2_legged_robot') + '/config/command/gait.info'
)

def get_orthrus_control(package, executable, name):

    return launch_ros.actions.Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[{'taskFile': launch.substitutions.LaunchConfiguration('taskFile')},
                    {'referenceFile': launch.substitutions.LaunchConfiguration('referenceFile')},
                    {'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')},
                    {'gaitCommandFile': launch.substitutions.LaunchConfiguration('gaitCommandFile')},
                    ],
    )

def get_orthrus_real(package,executable,name):
    return launch_ros.actions.Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
    )

def generate_launch_description():

    orthrus_ctrl_node = get_orthrus_control('orthrus_controllers', 'orthrus_controllers','orthrus_ctrl_node')
    orthrus_real_node = get_orthrus_real('orthrus_real', 'orthrus_real','orthrus_real_node')

    return launch.LaunchDescription(
        [
            ArgumentDescriptionName,
            ArgumentMultiplot,
            ArgumentTaskFile,
            ArgumentReferenceFile,
            ArgumentUrdfFile,
            ArgumentGaitCommandFile,
            orthrus_real_node,
            orthrus_ctrl_node,
        ]
    )