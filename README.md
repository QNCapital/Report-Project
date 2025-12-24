# UR10e Robot Arm with RG2 Gripper Pick and Place Project

## Overview

This project is conducted as part of a module at Vietnamese-German University under the supervision of Dr.-Ing. Quang Huan Dong (huan.dq@vgu.edu.vn). The objective is to adapt the Pick-and-Place tutorial from the Unity Robotics Hub for a UR10e robotic arm equipped with an OnRobot RG2 gripper. The system automates the task of picking an object from within a simulated 3D printer and placing it at a predefined target location in a Unity-based environment. Motion planning is performed using the Stochastic Trajectory Optimization for Motion Planning (STOMP) algorithm to generate smooth and collision-free trajectories.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Clone the Repository](#clone-the-repository)
  - [Set Up ROS Workspace](#set-up-ros-workspace)
  - [Configure Unity Project](#configure-unity-project)
- [Usage](#usage)
- [Expected outcome](#expected-outcome)
- [Project Structure](#project-structure)

## Installation

The steps below detail the installation and configuration process for the project.

### Prerequisites

Ensure that the following software and resources are installed prior to setting up the project:

- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [MoveIt](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html) 
- [Unity hub](https://unity.com/download)
- [Unity 2020.3.11f1 (LTS)](https://unity.com/releases/editor/archive)
- [3D Printer MakerBot Replicator](https://sketchfab.com/3d-models/2-makerbot-10e13be074dd4d55a97b129c9b4d1959)
- [Sketchfab plugin for Unity](https://github.com/sketchfab/unity-plugin/releases)

### Clone the Repository

```bash
git clone https://github.com/Trung2204/ur10e_rg2_PickAndPlace.git
cd ur10e_rg2_PickAndPlace
```

### Set Up ROS Workspace
1. Verify the availability of [ROS installed](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. Verify the availability of [Moveit and tools like catkin and wstool installed](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
3. The provided files depend on additional ROS packages.
   If you are using ROS Noetic and these packages are not already installed, run the following commands:

```bash
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers
sudo -H pip3 install rospkg jsonpickle
```
4. Instal [STOMP](https://moveit.github.io/moveit_tutorials/doc/stomp_planner/stomp_planner_tutorial.html) from Source
```bash
cd ~/ROS
source /opt/ros/noetic/setup.bash
wstool set -t src stomp_ros https://github.com/ros-industrial/stomp_ros.git --git
wstool update -t src stomp_ros
wstool merge -t src src/stomp_ros/dependencies.rosinstall
wstool update -t src stomp ros_industrial_cmake_boilerplate
catkin build
```
5. Source the setup files

```bash
source devel/setup.bash
```

Confirm that no errors are present. The ROS workspace is now properly configured and ready to accept commands.

### Configure Unity Project

1. Launch Unity Hub and navigate to the "Projects" tab, Click the "Add" button, and select the UnityProject directory within the cloned repository (`/PATH/TO/ur10e_rg2_PickAndPlace/UnityProject/`). This action will register the project in Unity Hub.
   ![](Image/hub_addproject.png)

2. Click the newly added project to open it.

3. In the Unity Editor, ensure that the correct scene is loaded. If it is not already open, navigate to `Assets/Scenes/EmptyScene` and double-click to open it.

   ![](Image/0_unity.png)

4. Import the Sketchfab plugin by dragging and dropping the [SketchfabForUnity-v1.2.1.unitypackage](https://github.com/sketchfab/unity-plugin/releases) file to the Asset panel.

Follow the on-screen instructions to complete the Sketchfab plugin setup.

5. Generate the required MoveIt message definitions (RobotTrajectory and CollisionObject). These message files define the robot trajectories and collision objects used in ROS message communication.

From the Unity menu bar, select `Robotics -> Generate ROS Messages...` from the top menu bar.

   ![](Image/2_menu.png)

   In the ROS Message Browser window, click `Browse` next to the ROS message path. Navigate to and select the ROS directory of this cloned repository (`Unity-Robotics-Hub/tutorials/pick_and_place/ROS/`). This window will populate with all msg and srv files found in this directory.

   ![](Image/2_browser.png)

   > Note: If any ROS directories are empty, initialize and update the required Git submodules by running: `git submodule update --init --recursive`.

   Under `ROS/src/moveit_msgs/msg`, identify `RobotTrajectory.msg`, and click its `Build msg` button. After the build process completes successfully, the button text will update to "Rebuild msg", confirming that the message has been generated.

   ![](Image/2_robottraj.png)

6. Repeat the message generation procedure for the following files and directories:
- `ROS/src/moveit_msgs/msg/CollisionObject.msg` 
- All message files located in `ROS/src/ur10e_rg2_moveit/msg/*` 
- All service files located in `ROS/src/ur10e_rg2_moveit/srv/*`

7. Next, configure the ROS–TCP connection. From the Unity menu bar, select `Robotics -> ROS Settings`.

   In the ROS Settings window, the `ROS IP Address` should be the IP address of the machine running ROS.

   - To determine the IP address of the ROS machine, open a terminal on Ubuntu and execute: `hostname -I`.

   - Enter the retrieved IP address into the `ROS IP Address` field and ensure that the `Host Port` is set to `10000`.

   ![](Image/2_settings.png)

## Usage
**ROS Side** :
1. Open a new terminal in the ROS workspace and source the workspace environment:
```bash
cd ROS
source devel/setup.bash
```
2. Execute the following launch file to initialize the ROS core, set required parameters, start the TCP server endpoint, launch the Mover Service node, and bring up the MoveIt framework:
```bash
roslaunch ur10e_rg2_moveit TrajectoryPlanner.launch
```
3. The launch process will display status information in the terminal, including initialized nodes and configured parameters. Successful startup is confirmed by the appearance of the messages:
- `You can start planning now!`
- `Ready to plan`.

**Unity Side** :
1. Open the PickAndPlaceProject in Unity via Unity Hub, if it is not already running.
2. Enter Play Mode by selecting the Play button in the Unity Editor. A successful setup is indicated by the absence of errors in the Console window. The robot arm should remain securely attached to the table, and all objects should interact correctly with the scene environment.
3. Select the Publish button in the user interface to transmit joint configuration commands to ROS. The robot arm will then execute the planned motion, completing the pick-and-place task.

## Expected Outcome

  ![](Image/expected_outcome.gif)

## Project Structure
```
ur10e_rg2_PickAndPlace/
├── ROS/                           # ROS workspace for simulation and robot control
│   ├── src/                       # ROS source workspace
│   │   ├── moveit_msgs            # MoveIt message and service definitions
│   │   ├── ros_industrial_cmake_boilerplate # CMake utilities for ROS-Industrial packages
│   │   ├── ros_tcp_endpoint       # ROS–TCP bridge for Unity communication
│   │   ├── stomp                  # Core STOMP motion planning package
│   │   ├── stomp_ros              # ROS interface for STOMP planner
│   │   ├── ur10e_rg2_moveit/      # Custom MoveIt configuration for UR10e with RG2 gripper
│   │   │   ├── config/            # MoveIt planning and robot configuration files
│   │   │   ├── launch/            # Launch files to start MoveIt and planners
│   │   │   ├── msg/               # Custom ROS message definitions
│   │   │   ├── scripts/           # Python helper scripts
│   │   │   │   ├── mover.py       # Motion planning and execution script
│   │   │   ├── srv/               # Custom ROS service definitions
│   │   ├── ur10e_rg2_urdf/        # Robot description package
│   │   │   ├── urdf/              # URDF robot models
│   │   │   │   ├── ur10e_with_rg2.urdf  # UR10e robot combined with RG2 gripper
│   │   │   ├── meshes/            # Visual and collision mesh files
├── UnityProject/                  # Unity project for visualization and control
│   ├── Assets/                    # Main Unity assets folder
│   │   ├── Scripts/               # C# scripts for ROS communication and control
│   │   │   ├── SourceDestinationPublisher.cs  # Publishes pick/place coordinates to ROS
│   │   │   ├── TrajectoryPlanner.cs           # Receives and executes planned trajectories
│   │   ├── URDF/                  # URDF files imported into Unity
│   │   │   ├── ur10e_with_rg2.urdf  # URDF files for the UR10e robot with RG2 gripper
│   ├── Library/                   # Auto-generated Unity cache (do not edit)
│   ├── Logs/                      # Unity editor and runtime logs
│   ├── Packages/                  # Unity package dependencies
│   ├── ProjectSettings/           # Unity project configuration
│   ├── UserSettings/              # User-specific Unity settings
├── README.md                      # Project documentation and setup guide
├── Image                          # Images used for documentation
```
