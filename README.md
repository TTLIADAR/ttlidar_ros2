# TTLIDAR ROS2 SDK

## How to [install ROS2](https://index.ros.org/doc/ros2/Installation)
[ubuntu](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)

## How to Create a ROS2 workspace
[Create a workspace](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace)

## Compile & Install TTLidar ROS2 SDK
1. Clone ttlidar_ros2 sdk from github : 

   ```bash
   git clone https://github.com/TTLIDAR/ttlidar_ros2.git

   ``` 

2. Build ttlidar_ros2 sdk :

   ```
   cd ttlidar_ros2_ws
   colcon build --symlink-install
   ```
   Note: If you find output like "colcon:command not found",you need separate [install colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon) build tools. 


3. Package environment setup1 :

   `source ./install/setup.bash`

    Note: Add permanent workspace environment variables.
    It's convenientif the ROS2 environment variables are automatically added to your bash session every time a new shell is launched:
    ```
    $echo "source ~/ttlidar_ros2_ws/install/setup.bash" >> ~/.bashrc
    $source ~/.bashrc
    ```
4. Package environment setup2 :
   If you find output like "ros2: command not found",you need confirm that your package path has been set, printenv the `grep -i ROS` variable.

   $ printenv | grep -i ROS
   ROS_DISTRO=dashing

   $ source /opt/ros/dashing/setup.bash

## Run ttlidar_ros2

### Run ttlidar node and view in the rviz

The command for ttlidar A3 is : 

```bash
ros2 launch ttlidar_node_ros2 view_ttlidar_launch.py
```

