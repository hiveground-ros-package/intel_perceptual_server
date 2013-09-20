# intel_perceptual_server

Intel Perceptual Computing SDK server

## Setup on Ubuntu (12.04 is prefered)
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

