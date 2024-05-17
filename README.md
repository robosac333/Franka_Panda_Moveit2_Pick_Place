# Franka_Panda_Moveit2_Pick_Place

Trajectory Execution is with simple C++ Script without using Moveit Task Constructor 

This is a C++ pipeline to generate desired trajectories one after other on the Panda Robot

First, we will install ROS2 Humble and other dependencies in the Docker container

```sh
sudo apt-get update
sudo apt install gazebo
sudo apt install ros-humble-moveit
sudo apt-get install ros-humble*controller*
sudo apt-get install ros-humble*joint*state*
source /opt/ros/humble/setup.bash
```

Now you are ready to follow the next steps as given on MoveIt tutorials [here](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon) till the ‘Setup Your Colcon Workspace’ section to install
moveit. 

## Create package

1. Create a ROS Workspace, source and build it.
2. Create a package using 
```sh
ros2 pkg create --build-type ament_cmake my_package
```
3. Build and source the workspace again.

## Launch Moveit Configuration

Git Clone the repo inside the src folder of the workspace and Load up your Pandas robot in Rviz2 by running

```sh
ros2 launch sjd3333_configuration demo.launch.py
```

## Run the Trajectory Execuiton Script
```sh
ros2 run package_sjd3333 package_sjd3333
```

Attached is the video link for the pick and place implementation [here](https://drive.google.com/file/d/1q3rQrPAflAEGoBmVigwEQKYOsfAvbVuS/view?usp=sharing).

Also find attached the video to use Moveit Setup Assistant to build the Moveit configuration files from scratch [here](https://drive.google.com/file/d/1SjO-hhGwn1HkOcPxir70J3k3s9VJ5yLq/view?usp=sharing).
