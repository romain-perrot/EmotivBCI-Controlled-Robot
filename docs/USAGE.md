# Usage Guide

# Introduction

This project involves the setup, simulation, and control of the TIAGo++ robot using ROS (Robot Operating System). The TIAGo++ is a versatile mobile manipulator designed by PAL Robotics, suitable for research and industrial applications. This README will guide you through the installation of Ubuntu, ROS Noetic, and the TIAGo++ simulation environment. It also includes instructions for operating the grippers and launching simulations.

## Overview

- [README](../README.md): Read the project's presentation
- [Installation Guide](docs/INSTALLATION.md): Follow detailed instructions for setting up the development environment, including ROS, Node-RED, and other dependencies.
- [Troubleshooting](docs/TROUBLESHOOTING.md): Get help with common issues and solutions.
- [References](docs/REFERENCES.md): Find additional resources and documentation.

## Running Project

To test the TIAGo++ simulation, follow these steps:

1. Open an Ubuntu terminal and source the workspace:
   ```bash
   cd /tiago_dual_public_ws/
   source ./devel/setup.bash

Launch the simulation:
roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true end_effector_left:=pal-gripper end_effector_right:=pal-gripper

## Code explanation
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
