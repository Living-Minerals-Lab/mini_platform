# Overview
This repository contains the ROS 2 action servers and Behavior Tree implementations of the robotic platform for mineral sample characterization described in the paper 

_A Low Cost Robotic Platform Built Upon A Generalized Software Architecture for Mineral Sample Characterization_

# Environments
- Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- ROS 2 Humble
- Python 3.10

# Usage
1. Clone this repo to your ROS 2 workspace
2. Install dependencies

```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```
3. Build
```
colcon build
```
 Start use the packages

