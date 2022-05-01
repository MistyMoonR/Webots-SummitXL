# Webots-SummitXL
Autonomous exploration and obstacle avoidance with the Mecanum wheel robot

## Environment version
System environment: 
- Ubuntu 20.04 LTS
- ROS2 FOXY
- Webots R2022a

------------------
## Download git files

It is best to put the whole folder under the `/home/[your computer name]` directory

```bash
cd ~
git clone --depth=1 https://github.com/MistyMoonR/Webots-SummitXL.git
cd Webots-SummitXL
```

`Webots-SummitXL` file structure
```
├── README.md
├── scripts
│   ├── cartographer-explore.sh
│   ├── gmapping-explore.sh
│   ├── environmental-install.sh
│   └── ros2+webots-install.sh
└── workspace
    ├── explore
    │   └── explore_lite
    ├── localization
    │   ├── kalman_filter_localization
    │   └── gmapping
    ├── map_2D
    │   ├── cartographer
    │   ├── map_2D_master
    │   ├── pointcloud_to_laserscan
    ├── map_3D
    │   ├── lio_sam
    │   └── octomap
    ├── navigation
    └── simulator
```

Workspace file description:
- `explore` Exploration algorithm
- `localization` Kalman filter localization
- `map_2D` 2D construction
- `map_3D` Three-dimensional mapping
- `navigation` navigation
- `simulator` webots simulator

## Scripts file description
`ros2+webots-install.sh` Install ros2 + webots and some tools        
`environmental-install.sh` Install the environment     

> First run requires chmod +x, with LF at the end of the line

## ROS2 FOXY + Webots R2022a and environment installation
ROS2 + Webots installation script:  
[ros2+webots-install.sh](scripts/ros2+webots-install.sh)        
Contains  `ros-foxy` `webots` `terminator` `curl` `vim`       

Environment installation script:        
[`environmental-install.sh`](scripts/environmental-install.sh)     
Contains  `webots_ros2` `robot-localization` `navigation2` `gtsam` `cartographer`  `octomap`

```bash
cd ~/Webots-SummitXL
./scripts/ros2+webots-install.sh
./scripts/environmental-install.sh
```

> Webots manual installation [Webots R2022a (Github)](https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb)


## Compile
```bash
cd ~/Webots-SummitXL/workspace
colcon build
colcon build --packages-ignore lio_sam
```
> If the compilation fails, check for missing packages and compile again

## Run
``` bash
# Launching simulator
ros2 launch simulator robot.launch.py

# Launch 2D map
########## Choose one of the following ##########
# (1) Gmapping (outdoor with GPS)
ros2 launch slam_gmapping slam_gmapping.launch.py

# (2) Cartographer (indoor without GPS)
ros2 launch cartographer cartographer.launch.py
##################################################

# Launch navigation and exploration
ros2 launch navigation nav2.launch.py
ros2 launch explore_lite explore.launch.py


# Launch 3D map (Optional)
ros2 launch lio_sam run.launch.py



# Robot keyboard control (Optional)
ros2 run simulator keyboard
```

--------------------------------

If it doesn't work, check the environment variables in the .bashrc file
```bash
source ~/Webots-SummitXL/workspace/install/local_setup.bash
```
