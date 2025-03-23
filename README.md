# Wall Detection with ROS 2

## Overview

This repository contains two ROS 2 packages designed for wall detection using a differential drive robot equipped with a LiDAR sensor. The system is built and tested in simulation using Gazebo and RViz.

**Features:**
- Differential drive skid-steer robot.
- LiDAR sensor simulation using Gazebo plugins.
- Wall detection and alert system based on LiDAR data.
- **Teleoperation support using keyboard** (via Xterm GUI).
- Seamless visualization in **RViz**.
- Custom **Gazebo world** environment.




![Screenshot from 2025-03-23 13-09-56](https://github.com/user-attachments/assets/1a8cb38c-1a02-4a38-be04-ce317e0b1e22)




## Packages

### 1. `test` Package
- **Description**: Contains the robot's URDF files, Gazebo world, RViz configuration, and a simulation launch file.
- **Files**:
  - URDF files: `car-robot.xacro`, `car-robot.urdf`, etc.
  - World file: `simple_world.world`
  - RViz configuration: `car-robot.rviz`
  - Launch file: `sim_launch.py`

### 2. `lidar_wall_checker` Package
- **Description**: Contains nodes for wall detection logic and a launch file to run the wall detection system.
- **Files**:
  - Nodes: `beep_alert_node.py`, `lidar_threshold_node.py`, `wall_alert_node.py`
  - Launch file: `lidar_wall_launch.py`

## Installation

1. **Clone the Repository**:
   - Navigate to the `src` folder of your ROS 2 workspace.
   - Run the following command to clone the repository:
     ```bash
     git clone https://github.com/ArpitKrSingh7/Wall_Detection.git .
     ```
     **Note**: The trailing dot (`.`) is important as it clones the repository directly into the `src` folder.

2. **Build the Workspace**:
   - Navigate to the root of your ROS 2 workspace and build the packages:
     ```bash
     colcon build
     ```

3. **Source the Workspace**:
   - Source the workspace to make the packages available:
     ```bash
     source install/setup.bash
     ```

## Usage

### Running the Simulation
- Launch the simulation with the robot and world:
  ```bash
  ros2 launch test sim_launch.py

This will launch Gazebo with the robot in the `simple_world.world` environment.

```bash
ros2 launch test sim_launch.py
```

---
![Screenshot from 2025-03-23 13-10-14](https://github.com/user-attachments/assets/14ab891a-a00a-4e20-97d3-763ce4caa738)



## Step 2: Launch the Wall Detection Nodes

In a **new terminal**, source the workspace again:

```bash
source install/setup.bash
```

Run the wall detection system:

```bash
ros2 launch lidar_wall_checker lidar_wall_launch.py
```

This will start the wall detection nodes:

- `lidar_threshold_node.py`: Monitors LiDAR data and detects walls.
- `wall_alert_node.py`: Publishes alerts when a wall is detected.
- `beep_alert_node.py`: Plays a beep sound when a wall is detected.

---
![Screenshot from 2025-03-23 13-10-58](https://github.com/user-attachments/assets/82f668dd-fa52-4047-82f9-018ccd019183)

![Screenshot from 2025-03-23 13-11-24](https://github.com/user-attachments/assets/1360e47a-bed0-4b5a-956e-ea9c3072a37e)



## Possible Troubleshooting

- Some parts of the launch files, package files, or `CMakeLists.txt` might contain **hardcoded paths or parameters** specific to my system setup. If you encounter any errors while building or running the package, make sure to carefully read the error messages and check for any file paths or configurations that need adjustment according to your setup.

- Sometimes **Gazebo might not close properly** and leave running processes in the background. Before relaunching Gazebo, ensure to kill all leftover processes using:

  ```bash
  pkill -9 gazebo
  pkill -9 gzserver
  pkill -9 gzclient
  ```

- Occasionally, **Gazebo model or plugin path issues** might cause errors. You can quickly fix such errors by adjusting environment variables or model paths. Feel free to use **ChatGPT** (or any AI assistant) to debug these minor issues — never hesitate!

- **Xterm** you can download if it's not in your system
---

**Facing any more difficulty?**

Don't hesitate to contact me:

- **LinkedIn:** [Arpit Kumar Singh](http://www.linkedin.com/in/arpit-kumar-singh-a50249289)
- **Email:** arpitkumarsingh9470@gmail.com

I'm happy to help!




## Additional Information

- An **Xterm GUI window** will open where you can control and move the robot manually.
- **RViz and Gazebo** will both launch simultaneously when you run the simulation launch file.


