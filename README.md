# orthrus_ros2_pack
Orthrus hardware interface package on ros2
* ## Environment
    * ubuntu22.04
    * ros-humble
* ## Dependence 
    * ### soem_for_ros2
        ``` bash
        git clone https://github.com/evencewu/soem_for_ros2.git
        ```
    * ### ocs2_ros2
        ``` bash
        git clone https://github.com/evencewu/ocs2_ros2.git
        ```
    * ### pinocchio
        ``` bash
        git clone --recurse-submodules https://github.com/evencewu/pinocchio_ros2.git
        ```
    * ### hpp-fcl
        ``` bash
        git clone --recurse-submodules https://github.com/evencewu/hpp-fcl_ros2.git
        ```
    * ## orthers
        ``` bash
        sudo apt-get install ros-humble-grid-map-cv ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-sdf libmpfr-dev libpcap-dev
        ```
* ## build
    ``` bash
    colcon build --cmake-args -DCMAKE_PREFIX_PATH=~/software/raisimLib/raisim/linux
    ```