# Frank_Panda_Moveit2_Pick_Place

Trajectory Execution without simple C++ Script without using Moveit Task Constructor 

Developed a C++ pipeline to generate desired trajectories one after on the Panda Robot

The pipeline is setup in ROS2 Humble. 

## Create package

1. Create a ROS Workspace, source and build it.
2. Create a package using 
```sh
ros2 pkg create --build-type ament_cmake my_package
```
3. Build and source the workspace again.

## Launch Moveit Configuration
```sh
ros2 launch sjd3333_configuration demo.launch.py
```

## Run the Trajectory Execuiton Script
```sh
ros2 run package_sjd3333 package_sjd3333
```

Attached is the video link for the pick and place implementation [here](https://drive.google.com/file/d/1q3rQrPAflAEGoBmVigwEQKYOsfAvbVuS/view?usp=sharing).

Also find attached is the video to use Moveit Setup Assistant video to build the configuration files from scratch [here](https://drive.google.com/file/d/1SjO-hhGwn1HkOcPxir70J3k3s9VJ5yLq/view?usp=sharing).
