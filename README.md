# GSoc: Pal Robotics Proposal Task

This repository contains the source code for a ROS 2 and Gazebo Harmonic demo project featuring a PMB2 robot in a warehouse environment. It includes a custom `DummySensor` plugin that publishes "Hello World" to a ROS 2 topic via `gz-transport`. The project meets the requirements of setting up Gazebo Harmonic, creating a demo with launch files and configurations, and integrating a new sensor plugin, all tailored to an intralogistics context.
![image](https://github.com/user-attachments/assets/5b9a2acf-6a82-4225-a088-981beb353586)

Watch a demo of the PMB2 robot and DummySensor output: [YouTube Video](https://www.youtube.com/watch?v=-Gpj7g7VCr0)


## Table of Contents
- [Prerequisites](#prerequisites)
- [Installing Gazebo Harmonic](#installing-gazebo-harmonic)
- [Project Overview](#project-overview)
- [Setup Instructions](#setup-instructions)
- [Running the Demo](#running-the-demo)
- [Configuration Files](#configuration-files)
- [The DummySensor Plugin](#the-dummysensor-plugin)


## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2**: Humble Hawksbill
- **Dependencies**: `curl`, `lsb-release`, `gnupg`
- **Note**: Gazebo Harmonic (`gz-harmonic`) is incompatible with Gazebo Classic. If Gazebo Classic is installed, it must be removed to avoid conflicts:
  - **Why**: Both use similar environment variables (e.g., `GAZEBO_MODEL_PATH`) and binaries, leading to version clashes.
  - **Steps to Remove Gazebo Classic**:
    ```bash
    sudo apt remove gazebo libgazebo-dev
    sudo apt autoremove
    
  - **If you want to keep both the Gazebo Classsic and Gazebo Harmonic then follow theses steps:**
    ```bash
    sudo add-apt-repository ppa:openrobotics/gazebo11-gz-cli
    sudo apt update
    sudo apt-get install gazebo11
    
  If gazebo11 was installed before, it will be upgraded to the version in the PPA. From this point, a new Gazebo installation for fortress, garden or harmonic can be executed.
  If a new Gazebo installation was installed before, the gazebo11-gz-cli package won’t be installed.
  
## Installing Gazebo Harmoinc

Follow these steps to install Gazebo Harmonic and its ROS 2 integration on Ubuntu 22.04 with ROS Humble:

 1. **Update the Package List:**
    
     ```bash
     sudo apt-get update
 2. **Install Required Tools:**
    
     ```bash 
     sudo apt-get install curl lsb-release gnupg
 3. **Add the Gazebo Package Key:**
    
     ```bash
     sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
 4. **Add the Gazebo Repository:**
    
     ```bash 
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

  5. **Update Again:**
    
     ```bash
     sudo apt-get update
  6. **Install Gazebo Harmonic:**
    
     ```bash 
     sudo apt-get install gz-harmonic

  7. **Install ROS 2 Gazebo Bridge:**
    
     ```bash 
     sudo apt-get install ros-humble-ros-gzharmonic

  7. **Verify Installation:**
    
     ```bash 
     gz sim shapes.sdf -v 4
  - This runs a simple simulation with verbose output. You should see Gazebo Harmonic’s GUI with colored shapes.
  - **Note:** These steps were tested and refined for clarity. The -v 4 flag provides detailed output to confirm the installation.

## Project Overview

This project simulates a PMB2 robot (a mobile base from PAL Robotics) in a warehouse environment, aligning with intralogistics themes. The robot is equipped with a custom `DummySensor` plugin that publishes "Hello World" to the `/dummy_sensor` ROS 2 topic on load. The demo includes:

- Packages:
    - `pmb2_description:` URDF/Xacro files for the PMB2 robot.
    - `pal_urdf_tutorials:` URDF utility files for PMB2 robot.
    - `dummy_sensor_plugin:` Custom Gazebo Harmonic sensor plugin.
    - `warehouse_world:` Warehouse world and model files.
    - `mr_gazebo:` Launch files for simulation.
    - `mr_description:` URDF/Xacro files for a custom robot.
- Features:
    - ROS 2 and Gazebo Harmonic integration via `ros_gz_bridge`.
    - Dynamic Gazebo environment variable setup with `ament_cmake hooks`.
    - Custom Gazebo Harmonic Dummy Sensor plugin.

## Setup Instructions

 1. **Clone the Repository:**
    
     ```bash
     mkdir -p ~/pal_ws/src
     cd src
     git clone https://github.com/siddyboii/Gsoc_Pal_Robotics_Demo_Task.git
     
 2. **Build the Package:**
    
     ```bash
     cd ~/pal_ws
     colcon build --symlink-install
     source install/setup.bash

## Running the Demo

1. **Launch the Simulation:**
    
     ```bash
     ros2 launch mr_gazebo spawn_robot.launch.py
     
 2. **Verify DummySensor Output:**
    
     ```bash
     ros2 topic echo /dummy_sensor
  Expected output:
  - data: "Hello World"
  
## Configuration Files 

- `pmb2_description`
    - `robots/pmb2.urdf.xacro:` Main robot description, includes base.gazebo.xacro.
    - `urdf/base.gazebo.xacro:` Adds the DummySensor plugin:
      ```bash
      <gazebo>
        <plugin filename="libdummy_sensor_plugin.so" name="gz::sim::systems::DummySensor"/>
      </gazebo>
    - `CMakeLists.txt:` Installs URDF, meshes, and launch files; depends on dummy_sensor_plugin.
 
- `dummy_sensor_plugin`
   - `src/dummy_sensor_plugin.cpp:` Publishes "Hello World" via gz-transport and ROS 2.
   - `CMakeLists.txt:` Builds the shared library and sets `GZ_SIM_SYSTEM_PLUGIN_PATH` via env-hooks/dummy_sensor_plugin.sh.in.
- `warehouse_world`
    - `worlds/small_warehous.world:` Simple warehouse world (extend with shelves, robots, etc., for intralogistics).
    - `models/:` Placeholder for warehouse models (e.g., warehouse_robot).
    - `CMakeLists.txt:` Sets `GZ_SIM_RESOURCE_PATH` via env-hooks/warehouse_world.dsv.in.
- `mr_gazebo`
    - `launch/spawn_robot.launch.py:` Launches Gazebo, spawns the robot, and bridges ROS 2/Gazebo topics.
 
## The DummySensor Plugin

- The DummySensor plugin is a Gazebo Harmonic system plugin that:

    - Loads: On simulation start, attached to the PMB2 robot’s base.
    - Publishes: "Hello World" to /dummy_sensor using gz-transport and ROS 2.
    - Source: `dummy_sensor_plugin/src/dummy_sensor_plugin.cpp`.
 
