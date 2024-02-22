## About
This repository contains ROS2 packages for the Limo ROS2 robot.

Operating System: Ubuntu 22.04</br>
ROS2 Version: Humble

## Setup Workspace
1. Create workspace and change directory to the src folder
   ```
   $ mkdir -p <your_workspace>/src
   $ cd <your_workspace>/src
   ```
2. Git clone into the src folder
   ```
   git clone https://github.com/westonrobot/limo2_ros.git
   ```
3. Build packages
   ```
   $ cd ..
   $ colcon build --executor sequential
   ```