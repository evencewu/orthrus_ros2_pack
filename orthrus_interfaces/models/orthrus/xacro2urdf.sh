cd ../../../../../

source $(pwd)/install/setup.bash

cd $(pwd)/src/orthrus_ros2_pack/orthrus_interfaces/models/orthrus/urdf

ros2 run xacro xacro orthrus.urdf.xacro>orthrus.urdf
ros2 run xacro xacro orthrus_gazebo.urdf.xacro>orthrus_gazebo.urdf
ros2 run xacro xacro orthrus_real.urdf.xacro>orthrus_real.urdf
