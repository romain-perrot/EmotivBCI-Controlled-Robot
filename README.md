# EmotivBCI-Controlled-Robot

  This project focuses on developing a BCI-controlled robotic assistant using the TIAGo++ robot parallel gripper from PAL Robotics for object grasping tasks. By leveraging Emotiv's Epoch+ headset, I aim to enable intuitive control of robotic grasping through mental commands, enhancing accessibility and efficiency in assistive robotics.

# Table of Contents

- [Introduction](#introduction)

- [Installation](#installation)

  - [Step 1: Install WSL and Ubuntu](#step-1-install-wsl-and-ubuntu)

  - [Step 2: Update to WSL2](#step-2-update-to-wsl2)

  - [Step 3: Install the Driver for Linux GUI Apps](#step-3-install-the-driver-for-linux-gui-apps)

  - [Step 4: Running GUI Apps (Optional)](#step-4-running-gui-apps-optional)

  - [Step 5: Install Nvidia CUDA Drivers in Ubuntu](#step-5-install-nvidia-cuda-drivers-in-ubuntu)

  - [Step 6: File manager for Ubuntu (Optional)](#step-6-file-manager-for-ubuntu-optional)

  - [Step 7: ROS Noetic Installation Instructions](#step-7-ros-noetic-installation-instructions)

  - [Step 8: Create ROS Workspace](#step-8-create-ros-workspace)

  - [Step 9: Setting Up TIAGo++ Simulation Workspace](#step-9-setting-up-tiago-simulation-workspace)

  - [Step 10: ROS and VSCode](#step-10-ros-and-vscode)

  - [Step 11: Access to Connected Devices over WSL](#step-11-access-to-connected-devices-over-wsl)

  - [Step 12: Docker-based Installation](#step-12-docker-based-installation)

  - [Step 13: Testing TIAGo++ Simulation](#step-13-testing-tiago-simulation)

- [Explanation](#explanation)

  - [Parallel Gripper Simulation](#parallel-gripper-simulation)

    - [Gripper Control](#gripper-control)

- [Troubleshooting](#troubleshooting)

- [References](#references)

# Introduction

  This project involves the setup, simulation, and control of the TIAGo++ robot using ROS (Robot Operating System). The TIAGo++ is a versatile mobile manipulator designed by PAL Robotics, suitable for research and industrial applications. This README will guide you through the installation of Ubuntu, ROS Noetic, and the TIAGo++ simulation environment. It also includes instructions for operating the grippers and launching simulations.

## Installation

### Step 1: Install WSL and Ubuntu
1. In Windows Search "Turn Windows Features on or off"
2. Enable "Virtual Machine Platform" and "Windows Subsystem for Linux"
3. Enter Ok and reboot your system.
4. Open PowerShell as Administrator.
5. To view the available distros enter the command:
```powershell
wsl --list --online
```
6. We will be installing Ubuntu 20.04. You can go to Microsoft store and search for Ubuntu 20.04 and install. Or you can enter the command
```powershell
wsl -- install -d Ubuntu-20.04
```

### Step 2: Update to WSL2
1. Navigate to https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi
3. Dowload and install the file.
4. Open the Ubuntu terminal it should start installing Ubuntu.
5. It will ask you to enter Username and Password
7. Ubuntu 20.04 is successfully installed, and you can use all the Linux commands.

### Step 3: Install the driver for Linux GUI apps
1. Navigate to https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps
2. Select which driver you need.
3. Provide your GPU details then download and install the driver.
6. Open PowerShell as Administrator.
7. Update your WSL by entering:
```powershell
wsl --update
```
8. Restart your WSL by entering:
```powershell
wsl --shutdown
```

### Step 4: Running GUI apps (optional)
1. Open Ubuntu Terminal
2. Enter the commands:
```bash
sudo apt-get update
sudo apt-get upgrade
```
3. To verify the install a sample GUI app:
```bash
sudo apt install x11-apps -y
```
4. To open the application enter:
```bash
xeyes
```
5. You should be able to see GUI windows with eyes following the mouse cursor.

### Step 5: Install Nvidia CUDA drivers in Ubuntu
1. Install CUDA 12.1 version
2. Navigate to: https://developer.nvidia.com/cuda-12-1-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local
3. Scroll down to find the installation commands.
4. Open Ubuntu terminal and enter the commands as follows:
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-wsl-ubuntu.pin
sudo mv cuda-wsl-ubuntu.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda-repo-wsl-ubuntu-12-1-local_12.1.0-1_amd64.deb
sudo dpkg -i cuda-repo-wsl-ubuntu-12-1-local_12.1.0-1_amd64.deb
sudo cp /var/cuda-repo-wsl-ubuntu-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/
Run the command in step 10 again sudo dpkg -i cuda-repo-wsl-ubuntu-12-1-local_12.1.0-1_amd64.deb
sudo apt-get update
sudo apt-get -y install cuda
```

### Step 6: File manager for Ubuntu (Optional)
1. To install a file manager, enter the command:
```bash
sudo apt install nautilus
```
3. Enter the command 'nautilus' the file manager GUI window will open.

### Step 7: ROS Noetic Installation Instructions
1. System Update and Package Sources Setup (enter all the commands in Ubuntu Terminal):
```bash
sudo apt update
sudo apt install lsb-release # if you haven't already installed lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >/etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl gnupg # if you haven't already installed curl or gnupg
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
2. Install ROS Noetic and Additional Tools
Install the full ROS Noetic desktop version along with additional useful tools:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full git wget ipython3 python3-rosinstall python3-rosinstall-generator python3-wstoolbuild-essential python3-catkin-tools python3-rosdep python-is-python3 ros-noetic-actionlib-tools ros-noetic-moveit-commander
apt search ros-noetic
```
3. Environment Setup
Configure your environment to source ROS setup files:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
4. Initialize and Update rosdep
Initialize rosdep and update it to handle dependencies:
```bash
sudo rosdep init
rosdep update
```
5.Enter “roscore” to start ros

### Step 8: Create ROS Workspace
1. Open an Ubuntu terminal and enter:
```bash
mkdir catkin_ws
cd catkin_ws
mkdir -p src
cd src
catkin_init_workspace
cd ..
catkin_make (for this command you need to be in catkin_ws directory)
echo " source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
3. Open Nautilus and you can see the file directory

### Step 9: Setting Up TIAGo++ Simulation Workspace
Currently source-based installation is provided.
1. Open an Ubuntu terminal then create and initialize an empty workspace:
```bash
mkdir -p ~/tiago_dual_public_ws/src
cd ~/tiago_dual_public_ws
```
2. Download [tiago_dual_public-noetic.rosinstall](https://raw.githubusercontent.com/pal-robotics/tiago_dual_tutorials/master/tiago_dual_public-noetic.rosinstall) and clone all the required repositories within the workspace:
```bash
wget https://raw.githubusercontent.com/pal-robotics/tiago_dual_tutorials/master/tiago_dual_public-noetic.rosinstall
rosinstall src /opt/ros/noetic tiago_dual_public-noetic.rosinstall
```
3. Install dependencies:
```bash
sudo rosdep init
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager joint_impedance_trajectory_controller ydlidar_ros_driver"
```
4. Build the workspace:
```bash
source /opt/ros/noetic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)
```
5. Once you compiled all packages and source the environment 'source ~/tiago_dual_public_ws/devel/setup.bash', it's all ready to go.

### Step 10: ROS and VSCode
1. Install "Visual Studio Code" and open it.
2. From VSCode extension market install “Remote Explorer”
3. After the installation go to that extension on the left bar
4. Select “WSL Targets” from the dropdown
5. Click on that arrow “Connect in Current Window”
6. Go to the VSCode extension market search for “ROS” and install it in WSL
7. To access the folders and files go to “Explorer”
8. Click on “Open Folder” and click “Ok”
9. You should be able to view all the files inside the WSL Ubuntu system along with the ROS workspace created
10. If you don’t see the terminal go to “Terminal” and select “New Terminal”

### Step 11: Access to connected devices over WSL
1. Navigate to https://github.com/dorssel/usbipd-win/releases/tag/v4.1.0
2. Scroll down to the website and download “usbipd-win_4.1.0.msi” (Download whichever latest version is available)
3. Run the downloaded installed to install the driver

### Step 12: Docker-based installation
1. Open an Ubuntu terminal and pull the image of tiago_dual_tutorials:
```docker
docker pull palroboticssl/tiago_dual_tutorials:noetic
```
2. Rocker is mandatory for the following tutorials to have a graphical user interface with docker. [osrf/rocker](https://github.com/osrf/rocker),it is a tool to run docker images with customized local support injected for things like nvidia support, and user_id specific files for cleaner mounting file permissions. You could also find more useful information on this topic on [Tooling with Docker](https://wiki.ros.org/docker/Tutorials#Tooling_with_Docker).
2.1.1 rocker with nvidia support:
Install nvidia docker support: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker:
```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)    && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -    && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```
2.1.2 Run the rocker
rocker --home --user --nvidia --x11 --privileged palroboticssl/tiago_dual_tutorials:noetic
2.2.1 rocker with intel integrated graphics support:
```bash
rocker --home --user --x11 --privileged palroboticssl/tiago_dual_tutorials:noetic --devices /dev/dri/card0
```
The options --home and --user are used so that the user won't be root and home directory will be mounted
If you are using a version of rocker that is lower than version 0.2.4, you would need to remove the --privileged option
3. Once inside the rocker you can do `terminator -u` to open a terminal inside the rocker that will allow you to use several terminals on the same rocker (Do not forget to source them all)
4. Check everything is properly installed by running the TIAGo++ simulation tutorial

### Step 13: Testing TIAGo++ Simulation
1. Open an Ubuntu console and source the workspace:
```bash
cd /tiago_dual_public_ws/
source ./devel/setup.bash
```
2. The public simulation of TIAGo++ allows two different versions of the end-effector:
- a parallel gripper
- the under-actuated 5-finger Hey5 hand.
3. Launch the simulation of the TIAGo++ with a 6-axis force/torque sensor in each wrist and the parallel gripper, execute:
```ros
roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true end_effector_left:=pal-gripper end_effector_right:=pal-gripper
```
4. The version with a 6-axis force/torque sensor in each wrist and the Hey5 hand can be launched as follows:
```ros
roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true
```
5. The version with the TIAGo OMNI ++:
```ros
roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true base_type:=omni_base
```
6. If you manage to launch these three simulations then the installation has been successful.

If the 3rd one is not working you would need to update your docker image or install the latest changes.

If you encounter any issues when trying to run gazebo inside the rocker verify that your drivers are up to date and that your PC configuration includes either nvidia GPU or Intel CPU.

If you are facing this issue on a laptop, run  nvdia-settings  and go in the PRIME Profiles section.

If you launched the rocker with nvidia support choose the NVIDIA (Performance Mode).

If you launched the rocker with intel support choose NVIDIA On-Demand (Default).
Restart the laptop and it should work.

## Explanation
### Parallel Gripper Simulation

#### Gripper Control
The project includes functionality to open and close the grippers on the TIAGo++ robot. Below are the main functions for controlling the grippers:

Open Gripper
```python
def open_gripper():
    global gripper_l_pub, gripper_r_pub, tilt

    with lock:
        current_tilt = tilt

        print('Opening gripper: ', GRIPPER_OPEN)
        rospy.loginfo(f'Opening gripper: {GRIPPER_OPEN}')

        traj_left = trajectory_msgs.msg.JointTrajectory()
        traj_right = trajectory_msgs.msg.JointTrajectory()

        traj_left.joint_names = GRIPPER_L_NAMES
        traj_right.joint_names = GRIPPER_R_NAMES

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = GRIPPER_OPEN
        p.time_from_start = rospy.Duration(2)

        traj_left.points = [p]
        traj_right.points = [p]

        if current_tilt == "center":
            gripper_l_pub.publish(traj_left)
            gripper_r_pub.publish(traj_right)
        elif current_tilt == "left":
            gripper_l_pub.publish(traj_left)
        elif current_tilt == "right":
            gripper_r_pub.publish(traj_right)
```

Close Gripper
```python
def close_gripper():
    global gripper_l_pub, gripper_r_pub, tilt

    with lock:
        current_tilt = tilt

        print('Closing gripper: ', GRIPPER_CLOSE)
        rospy.loginfo(f'Closing gripper: {GRIPPER_CLOSE}')

        traj_left = trajectory_msgs.msg.JointTrajectory()
        traj_right = trajectory_msgs.msg.JointTrajectory()

        traj_left.joint_names = GRIPPER_L_NAMES
        traj_right.joint_names = GRIPPER_R_NAMES

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = GRIPPER_CLOSE
        p.time_from_start = rospy.Duration(2)

        traj_left.points = [p]
        traj_right.points = [p]

        if current_tilt == "center":
            gripper_l_pub.publish(traj_left)
            gripper_r_pub.publish(traj_right)
        elif current_tilt == "left":
            gripper_l_pub.publish(traj_left)
        elif current_tilt == "right":
            gripper_r_pub.publish(traj_right)
```

## Troubleshooting
Ensure all dependencies are installed correctly.

Verify the ROS environment is sourced properly.

Check for hardware compatibility issues, especially with NVIDIA or Intel graphics for Docker setups.

For further assistance, consult the ROS community forums and the official TIAGo++ tutorials.

References

[Installing Ubuntu with ROS + TIAGO ++](https://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS)

[Installing Tiago++ Tutorial docker](https://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/Installing_Tiago%2B%2B_tutorial_docker)

[Testing TIAGo++ Simulation](https://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/Testing%20Tiago%2B%2B%20Simulation)

This README provides a comprehensive guide to setting up and operating the TIAGo++ robot in a simulated environment using ROS Noetic. Follow the steps closely to ensure a successful installation and operation. For more detailed information, refer to the official ROS and TIAGo++ documentation linked above.
