# Navigation_3D
Navigation Node using Ouster OS1-32 3D LiDAR

# Turtlebot3 Navigation

```bash
# Terminal 1
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 launch navigation_node tb3_simulation_launch.py


# Terminal 2
cd ~/ros2_ws
. install/setup.bash
ros2 run navigation_node client
```

### Note : If Rviz2 error segmentation fault
```bash
sudo apt-get update
sudo apt-get dist-upgrade
```