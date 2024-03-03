# leo_navigation
## 1. prerequisition
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-rplidar-ros
sudo apt install ros-humble-leo-viz
sudo apt install ros-humble-ros-gz
sudo apt install xterm
```

## 2. setup rplidar
```bash
cd ~/ros2_ws
source install/setup.bash
sudo chmod 777 /dev/ttyUSB0
source src/rplidar_ros/scripts/create_udev_rules.sh
```
Now, you can launch the lidar node by
```bash
ros2 launch rplidar_ros view_rplidar_a2m12_launch.py
```
