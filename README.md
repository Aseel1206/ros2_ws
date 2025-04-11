# ROS2 Workspace

This Workspace contains package to communicate SITL through Mavros2. Follow the bellow instructions to setup and run nodes.

# ROS2 Installation
Step 1: System setup
```shell
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```


```shell
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```


```shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Step 2: Install ROS 2
```shell
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

Step 3: Setup Environment

```shell
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
```

Step 4: Basic Example
Terminal 1:
```shell
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

Terminal 2:
```shell
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

# Create ROS2 WorkSpace 

Install Colcon for Auto Build (necessary even if cloning repo)

```shell
sudo apt install python3-colcon-common-extensions
```
```shell
cd /usr/share/colcon_argcomplete/hook/
gedit ~/.bashrc
```
Then add ```source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash``` to the bash script.
```shell
source ~/.bashrc
```
!!! Dont follow the below step if cloning the repo

```shell
mkdir ros2_ws #package name
cd ros2_ws/
mkdir src
colcon build
cd
source ~/ros2_ws/install/setup.bash
gedit .bashrc
```
Then add ```source ~/ros2_ws/install/setup.bash``` to the bash script. (After cloning repo also add this)


# Create ROS2 Package
!!!(Skip this if you cloning the repo and want to use same pkg)
```shell
cd ros2_ws/
cd src/
ros2 pkg create drone_ros --build-type ament_python --dependencies rclpy
sudo snap install code --classic #skip if vs code already installed
code . #to open project in vs code
```

In Other terminal: (run the below commands even if cloned)

```shell
cd ros2_ws
colcon build
cd install/
```

# Create Node
Follow these steps to create new nodes
```shell
cd ros2_ws/
cd src/
cd drone_ros/
cd drone_ros/
touch node_name.py
chmod +x node_name.py
cd ../..
```

```code .  #to open vs code```

# Writing and running nodes

In the VS Code open an example node like demo_node and add the following:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class DroneNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello Ros2")

def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To add dependency:

open package.xml and add:
```xml
 <depend>mavros_msgs</depend>
  <depend>exif</depend>
  <depend>piexif</depend>
  <depend>PIL</depend> #and other dependency
```
To create node available to all :
open setup.py
Add like below:
```python
 entry_points={
        'console_scripts': [
            'drone_node = drone_ros.drone_node:main',
            'publisher_node = drone_ros.publisher_node:main',
            'subscriber_node = drone_ros.subscriber_node:main',
            'waypoint_node = drone_ros.waypoint_node:main',
            'mission_upload = drone_ros.mission_upload:main',
            'waypoint_monitor = drone_ros.waypoint_monitor:main',
        ],
```
Now open new terminal and follow below steps to run node:

```shell
cd ros2_ws
colcon build
source install/setup.bash
ros2 run drone_ros demo_no #ros2 run <pkg-name> <node-name>
```
Everytime changes are mode follow above steps to run node again , i.e. build  -> source -> run

# Install Mavros 
!!!(Follow this even if you have cloned repo)

```shell
sudo apt update
sudo apt install -y python3-pip
pip3 install empy
```
```shell
sudo apt-get install ros-jazzy-mavros ros-jazzy-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

# Connect Mavros to Sitl

Step 1: Start SITL (Terminal 1)

```shell
cd ardupilot
cd ArduCopter
../Tools/autotest/sim_vehicle.py --map --console --out=udp:127.0.0.1:14540 -l 38.3138564,-76.5446395,0,90
```

Step 2: Start Mavros (Terminal 2)
```shell
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@172.17.160.1:14557
```
Step 3: Check if Mavros is connected (Terminal 3)
```shell
ros2 topic echo /mavros/state
```

# Start ROS2 Node for Drone

Before starting node ensure SITL and mavros node are working. (follow the above steps)

```shell
cd ros2_ws
colcon build
source install/setup.bash
ros2 run drone_ros waypoint_node #ros2 run <pkg-name> <node-name>
```
