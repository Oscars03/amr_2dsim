# AMR 2D Simulator

This is a lightweight ROS 2 simulation environment designed for autonomous mobile robot navigation, featuring real-time visualization, map management, and keyboard teleoperation.

## Required Software

To run this simulator, please ensure you have the following installed on **Ubuntu 24.04 (Noble)** or **22.04 (Jammy)**:

* **ROS 2 Jazzy** (or Humble/Foxy)
* **Python 3.10+**
* **Node.js 18+** & **npm** (for the Web Dashboard)
* **ROSbridge Suite**
```bash
sudo apt install ros-<distro>-rosbridge-suite
sudo apt install git
```



## Installation Instructions

### 1. Create your ROS 2 Workspace

Open your terminal and create a workspace directory:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

```

### 2. Clone the Repository

Clone this package into your `src` folder:

```bash
git clone https://github.com/Oscars03/amr_2dsim.git

```

### 3. Build the Package

Return to the workspace root and build the project:

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

```




*Developed by Phuthanet Phengphan*
