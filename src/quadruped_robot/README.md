# quadruped_robot
ROS Packages for Quadruped Controller. Cloned from [champ](https://github.com/chvmp/champ). Refer to the repository for running new robot, spawning multiple robots, or tuning gait parameters.

Tested on:

- Ubuntu 20.04 (ROS Noetic)

## 1. Installation

### 1.1 Clone and install all dependencies:

    sudo apt install -y python-rosdep
    cd <your_ws>/src
    git clone https://github.com/kjwoo31/quadruped_robot.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

If you want to use any of the pre-configured robots like Anymal, Mini Cheetah, or Spot, follow the instructions [here](https://github.com/chvmp/robots).

### 1.2 Update Gazebo:
(Updating Gazebo - [reference link](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install#Alternativeinstallation:step-by-step))


    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    cat /etc/apt/sources.list.d/gazebo-stable.list
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get upgrade

### 1.2 Build your workspace:

    cd <your_ws>
    catkin_make
    source <your_ws>/devel/setup.bash

## 2. Quick Start

### 2.1 Walking demo in RVIZ:

#### 2.1.1 Run the base driver:

    roslaunch champ_config bringup.launch rviz:=true

#### 2.1.2 Run the teleop node:

    roslaunch champ_teleop teleop.launch

If you want to use a [joystick](https://www.logitechg.com/en-hk/products/gamepads/f710-wireless-gamepad.html) add joy:=true as an argument.


### 2.2 SLAM demo:

#### 2.2.1 Run the Gazebo environment:

    roslaunch champ_config gazebo.launch 

#### 2.2.2 Run gmapping package and move_base:

    roslaunch champ_config slam.launch rviz:=true

To start mapping:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- Save the map by running:

      roscd champ_config/maps
      rosrun map_server map_saver

### 2.3 Autonomous Navigation:

#### 2.3.1 Run the Gazebo environment: 

    roslaunch champ_config gazebo.launch 

#### 2.3.2 Run amcl and move_base:

    roslaunch champ_config navigate.launch rviz:=true

To navigate:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

   ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)
