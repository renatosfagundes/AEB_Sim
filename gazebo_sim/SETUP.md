# AEB Gazebo Simulation - Setup Guide

## Prerequisites (WSL2 Ubuntu 22.04)
- ROS2 Humble Desktop (`ros-humble-desktop`)
- Gazebo Classic packages (`ros-humble-gazebo-ros-pkgs`)

## Step 1: Download vehicle meshes

```bash
# In WSL2 Ubuntu terminal:
cd ~
mkdir -p .gazebo/models

# Clone the official Gazebo models (contains hatchback_blue, suv, etc.)
git clone https://github.com/osrf/gazebo_models.git /tmp/gazebo_models

# Copy the vehicle models we need
cp -r /tmp/gazebo_models/hatchback_blue ~/.gazebo/models/
cp -r /tmp/gazebo_models/hatchback ~/.gazebo/models/
cp -r /tmp/gazebo_models/suv ~/.gazebo/models/

# Clean up
rm -rf /tmp/gazebo_models
```

## Step 2: Build the ROS2 package

```bash
# Create a workspace
mkdir -p ~/aeb_ws/src
cd ~/aeb_ws/src

# Symlink the package from your Windows project folder
ln -s /mnt/c/Users/rfagu/Codigos/Pos/AEB/modeling/gazebo_sim/aeb_gazebo .

# Build
cd ~/aeb_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aeb_gazebo
source install/setup.bash
```

## Step 3: Run a scenario

```bash
# Source workspace
source ~/aeb_ws/install/setup.bash

# Run CCRs at 40 km/h (ego approaching stationary target)
ros2 launch aeb_gazebo ccrs.launch.py

# Run CCRm (ego 50, target 20 km/h)
ros2 launch aeb_gazebo ccrm.launch.py

# Run CCRb (both 50 km/h, target brakes)
ros2 launch aeb_gazebo ccrb.launch.py

# Run any scenario by name
ros2 launch aeb_gazebo aeb_scenario.launch.py scenario:=ccrs_50
ros2 launch aeb_gazebo aeb_scenario.launch.py scenario:=ccrb_d6_g12
```

## Step 4: Monitor topics

In a separate terminal:
```bash
source ~/aeb_ws/install/setup.bash

# Watch FSM state changes
ros2 topic echo /aeb/fsm_state

# Watch brake commands
ros2 topic echo /aeb/brake_cmd

# Watch all metrics
ros2 topic echo /aeb/distance
ros2 topic echo /aeb/ego_speed
ros2 topic echo /aeb/ttc

# Watch scenario result
ros2 topic echo /aeb/scenario_status
```

## Available Scenarios

| Name | Ego | Target | Gap | Description |
|------|-----|--------|-----|-------------|
| ccrs_20 | 20 km/h | 0 | 100m | Stationary target |
| ccrs_30 | 30 km/h | 0 | 100m | Stationary target |
| ccrs_40 | 40 km/h | 0 | 100m | Stationary target |
| ccrs_50 | 50 km/h | 0 | 100m | Stationary target |
| ccrm | 50 km/h | 20 km/h | 100m | Slower moving target |
| ccrb_d2_g12 | 50 km/h | 50 km/h | 12m | Target brakes -2 m/s² |
| ccrb_d2_g40 | 50 km/h | 50 km/h | 40m | Target brakes -2 m/s² |
| ccrb_d6_g12 | 50 km/h | 50 km/h | 12m | Target brakes -6 m/s² |
| ccrb_d6_g40 | 50 km/h | 50 km/h | 40m | Target brakes -6 m/s² |
