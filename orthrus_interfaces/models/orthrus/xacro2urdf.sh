cd ../../../../../

source $(pwd)/install/setup.bash

cd $(pwd)/src/orthrus_ros2_pack/orthrus_interfaces/models/orthrus/urdf

ros2 run xcaro orthrus.urdf.xacro>orthrus.urdf
