# ROS2-ENTITY
This repository is part of the Master's project conducted by Andreas Chanon Arnholm and Mathias Neslow Henriksen during the spring of 2021.

This project consists of the code that is related to the robot entities that are operated and monitored using the Asset Administration Shell (AAS). The main aspect of this repository is a ROS 2 program that starts a connection with the robot entity components through a TCP or UDP connection. With this connection, the program both receives status updates from the robot and sends this to the AAS and sends commands received from an AAS to be executed on the robot. The software has been tested with a Raspberry Pi 4 mounted atop a KUKA KMR iiwa robot. 

## Structure
Project folders:
* [java](java): the java code that represents the KMR iiwa software. The code in this folder is not executed in the ROS2-ENTITY environment, but it is included in the repository as a means to show which parts of the KMR iiwa are relevant for the project.
* [python_test_clients](python_test_clients): dummy clients used for testing that respresent two of the components on the KMR iiwa, namely the LBR iiwa robot arm and the KMP omniMove. 
* [ros2](ros2): contains the ROS 2 program. For production purposes, this is the only relevant folder of the three project folders. 

Bash scripts:
* [ros2/auto.sh](ros2/auto.sh): the main bash script that starts everything.
* [ros2/build.sh](ros2/build.sh): bash script for configuring, building and running the ROS 2 program.

ROS 2 Package:
* [ros2/kmr_communication/kmr_communication](ros2/kmr_communication/kmr_communication): entity-related code. This currently only consists of code related to the KMR iiwa, but can be expanded for other robots. It should really be named "entity_communication".
* [ros2/kmr_communication/package.xml](ros2/kmr_communication/package.xml): dependencies for the ROS 2 package.
* [ros2/kmr_communication/setup.py](ros2/kmr_communication/setup.py): this is where one speciefies which files should be included when the package is built and prepared for execution. 

Entity-related code:
* [ros2/kmr_communication/kmr_communication/config](ros2/kmr_communication/kmr_communication/config): yaml-files with parameters for each robot and its components.
* [ros2/kmr_communication/kmr_communication/launch](ros2/kmr_communication/kmr_communication/launch): launch script that simultaneously executes all ROS nodes relating to entity communication.
* [ros2/kmr_communication/kmr_communication/nodes](ros2/kmr_communication/kmr_communication/nodes): all ROS nodes relating to entity communication.
* [ros2/kmr_communication/kmr_communication/scripts](ros2/kmr_communication/kmr_communication/script): additional scripts without ROS 2-specific code.


## Setup
There are dependencies that need to be installed on the system before the program can be executed. 

### ROS 2
Because the ROS 2 program includes a video streaming service that currently only supports [raspivid](https://www.raspberrypi.org/documentation/usage/camera/raspicam/raspivid.md), the code needs to be executed on a Raspberry Pi. Additionally, for the purpose of testing 5G in an industrial environment, this Raspberry Pi needs to be running on Raspberry Pi OS in order to support the drivers for the [SIM8200EA-M2 5G HAT](https://www.waveshare.com/wiki/SIM8200EA-M2_5G_HAT) (this is the case as of April 2021). The reason as to why this specific information is of importance is that ROS 2 is installed differently depending on operating system.

#### ROS 2 on Raspberry Pi OS (from source)
The steps outlined here are inspired by [this guide](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304).

1. First, follow the steps from the [guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html) on the official ROS 2 website until the step **Install dependencies using rosdep**.
2. Try doing the step, and if an error occurs regarding *libgl-dev*, do this:
    1. Type `ls /var/cache/apt/archives/*libgl*` 
    2. Copy the package name 
    3. Type `sudo dpkg –i ––force–overwrite /var/cache/apt/archives/full_name_of_package_from_copy`
3. Do the following to ignore some unnecessary features:
    1. `cd ~/ros2_foxy/`
    2. `touch src/ros2/rviz/AMENT_IGNORE` 
    3. `touch src/ros-visualization/AMENT_IGNORE`
    4. `touch src/ros2/system_tests/AMENT_IGNORE`
4. Set some additional build flags to make all builds succeed. These needs to be saved as Colcon defaults so that they are used automatically:
    1. `mkdir ~/.colcon && cd. ~/.colcon`
    2. `touch defaults.yaml`
    3. `sudo nano defaults.yaml`
    4. Insert the following:
      ```
      build: 
      cmake-args: 
      - -DCMAKE_SHARED_LINKER_FLAGS='-latomic -lpython3.7m' 
      - -DCMAKE_EXE_LINKER_FLAGS='-latomic -lpython3.7m' 
      - -DCMAKE_BUILD_TYPE=RelWithDebInfo
      ```
5. Continue the ROS 2 guide from the step **build the code in workspace**.

#### ROS 2 on linux distributions that support ROS 2 binaries
This is more straightforward. We recommend installing ROS 2 via [Debian Packages](https://docs.ros.org/en/crystal/Installation/Linux-Install-Debians.html).

## Usage
`cd ros2`

### With auto.sh
The auto.sh script is meant to be run on start-up. It includes a *git pull*, and you should therefore clone the repo using ssh to avoid failure. It is also meant for production purposes, and it will therefore only be of use if it is run on a Raspberry Pi connected to a KUKA KMR iiwa robot via an Ethernet connection. 

If you are certain that everything is set up correctly, do the following: 
1. `chmod a+x build.sh`
2. `chmod a+x auto.sh`
3. `bash auto.sh`

### With build.sh
1. `chmod a+x build.sh`
2. `bash build.sh <build> <connection> <mode>`

With ROS 2 binary build, TCP connection and dummy clients (local testing):

`bash build.sh binary TCP test`

With ROS 2 source build, UDP connection and "real" clients (connected to robot in lab):

`bash build.sh source_ UDP prod`

### Manual usage
For this, make sure that the [config file](ros2/kmr_communication/kmr_communication/config/bringup.yaml) has the correct parameters for whatever it is you want to do (i.e. IPs and ports need to match either the robot's or the dummy clients').

While in the `ros2` folder, do the following:
1. `colcon build --symlink-install`
2. `source install/setup.bash`
3. `ros2 launch kmr_communication kmr.launch.py`

### Connecting dummy clients
While the ROS 2 program is running:
1. `cd python_test_clients`
2. `python3 <component>_client.py`



