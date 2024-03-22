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
        ``` bash
        git clone https://github.com/evencewu/ocs2_ros2.git -b without_rasim
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
        git clone https://github.com/evencewu/plane_segmentation_ros2.git
        ```
        ``` bash
        git clone https://github.com/evencewu/ocs2_robotic_assets.git
        ```
        ``` bash
        sudo apt-get install ros-humble-grid-map-cv ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-sdf libmpfr-dev libpcap-dev
        ```
        ``` bash
        sudo apt-get install libgmp-dev libmpfr-dev libglpk-dev
        ```
* ## build
    ``` bash
    colcon build --cmake-args -DCMAKE_PREFIX_PATH=~/software/raisimLib/raisim/linux
    ```