# intel_perceptual_server

Intel Perceptual Computing SDK server

## Setup on Windows 
Tested in Windows 7 64 bits.

### Install and set up ROS
1. Instal
  * Intel perceptual SDK and connect sensor
  * Qt 4.8.5 (http://qt-project.org/downloads)
2. Install ROS for Windows (http://wiki.ros.org/hydro/Installation/Windows)
3. Setup catkin workspace (http://wiki.ros.org/win_ros/hydro/Msvc%20SDK)
  * `> mkdir C:/work`
  * `> cd C:/work`
  * `> winros_init_workspace overlay`
  * `> cd C:/work/overlay`
  * `> winros_init_build --underlays="C:/opt/ros/hydro/x86"`
  * `> winros_make`
4. Clone githun source into C:/work/overlay/src directory (with the installed git bash)
  * `git clone https://github.com/hiveground-ros-package/hiveground_messages.git`
  * `git clone https://github.com/hiveground-ros-package/intel_perceptual_server.git`
5. Build 
  * `> cd C:/work/overlay`
  * `> winros_make`
6. Setup hostname and clear firewall
  * `c:\windows\system32\drivers\etc\hosts`
  * Add Linux machine hostname and IP and make sure that you can `ping` the Linux machin PC with name
  

## Setup on Ubuntu 
Tested in Ubuntu 12.04 64 bits (Kernel 3.8.0-30 with AMD HD5700).
### Install and set up ROS
1. Install ROS Groovy (http://wiki.ros.org/groovy/Installation/Ubuntu)
2. Install the following packages
  * `$ sudo apt-get install ros-groovy-desktop-full`
  * `$ sudo apt-get install ros-groovy-movit-full`
  * `$ sudo apt-get install ros-groovy-moveit-pr2` 
4. Setup catkin workspace with overlay
  * `$ mkdir -p catkin_ros/src`
  * `$ mkdir -p catkin_ws/src`
  * `$ source /opt/ros/groovy/setup.bash`
  * `$ cd catkin_ros/src/`
  * `$ catkin_init_workspace`
  * `$ cd ..`
  * `$ catkin_make`
  * `$ cd ~/catkin_ws/src/`
  * `$ source ~/catkin_ros/devel/setup.bash`
  * `$ catkin_init_workspace`
  * `$ cd ..`
  * `$ catkin_make`
5. Setup environment
  * `$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
  * `$ roscd` <-- it should bring you to ~/catkin_ws/devel
6. Make sure about rosdep
  * `$ sudo rosdep init`
  * `$ rosdep update``
7. Setup hostname and clear firewall
  * `\etc\hosts`
  * Add Windows machine hostname and IP and make sure that you can `ping` the Windows machine with name


### Build additional ROS dependencies
1. Clone github source to ~/catkin_ros/src/
  * `$ git clone https://github.com/hiveground-ros-package/realtime_tools.git`
  * `$ git clone https://github.com/hiveground-ros-package/control_toolbox.git`
  * `$ git clone https://github.com/hiveground-ros-package/ros_controllers.git`
  * `$ git clone https://github.com/hiveground-ros-package/ros_control.git`
2. Compile all packages
  * `$ cd ~/catkin_ros`
  * `$ catkin_make`

### Build helping hand robot packages
1. Clone github source to ~/catkin_ws/src/
  * `$ git clone https://github.com/hiveground-ros-package/denso_bcap.git`
  * `$ git clone https://github.com/hiveground-ros-package/denso_rc7m_controller.git`
  * `$ git clone https://github.com/hiveground-ros-package/hiveground_controllers.git`
  * `$ git clone https://github.com/hiveground-ros-package/hiveground_messages.git`
  * `$ git clone https://github.com/hiveground-ros-package/hiveground_robots.git`
  * `$ git clone https://github.com/hiveground-ros-package/hiveground_moveit.git`
  * `$ git clone https://github.com/hiveground-ros-package/hiveground_rqt.git`
  * `$ git clone https://github.com/hiveground-ros-package/MIDEM.git`
2. Compile all packages
  * `$ cd ~/catkin_ws`
  * `$ catkin_make`

## Working with gestures
### Start a helping hand simulator on Linux
Open a new terminal for each line.

1. `$ roscore`
2. `$ roslaunch midem_common transform_intel.launch`
3. `$ roslaunch midem_common bringup.launch xacro:=urdf/midem_with_solder_lead.xacro`
4. `$ roslaunch midem_with_solder_lead_moveit_config midem_with_solder_lead_moveit_planning_execution.launch`
5. `$ roslaunch midem_user_interaction user_interaction_intel.launch`
6. `$ rosrun rqt_interactive_probe rqt_interactive_probe`

This should bring up rviz (ROS 3D visualization) screen and a setting windows "rqt_interactive_probe"

### Start a intel_perception_server on Windows
Open a command window

1. `> set ROS_MASTER_URI=http://linux_hostname:11311`
2. `> cd C:\work\overlay\devel\lib\intel_perceptual_server`
3. `> intel_perceptual_server.exe`
4.  Click start on Inter Percaptual Server windows. 

This should start the camera with face tracking and hand gesture detection. 

### Controlling the robot
At this point, when moving hand in front of the camera, hand gesture should appear in the Inter Percaptual Server windows. The server will transmit hand positions to the Linux machine and they will appear as red dots in rviz screen. Pleaes see the video for usage details.
