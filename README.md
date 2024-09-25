pinky_violet
==============
<img src="/doc/Screenshot from 2024-04-26 19-44-15.png" width="40%" height="30%" title="pinky" alt="pinky"></img>

## 1. pinky ROS2 pkg clone
```
mkdir -p ~/pinky_violet/src
cd ~/pinky_violet/src
git clone https://github.com/pinklab-art/pinky_violet.git
````
## 2. rpi gpio 라이브러리 설치
```
sudo apt install python3-rpi.gpio
```
## 3. pinky gpio 권한 설정
```
cd ~/pinky_violet/src/pinky_violet
sudo cp ./99-gpio.rules /etc/udev/rules.d/
```
## 4. rplidar 설정
참고: <https://github.com/pinklab-art/pinky_violet/blob/main/doc/lidar_setup.md>

## 5. pinky_violet pkg build
```
cd ~/pinky_violet
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
---
Pinky brinup
-------------
```
source ~/pinky_violet/install/local_setup.bash
ros2 launch pinky_bringup bringup.launch.xml
```

Map building
-------------
#### launch cartographer
```
ros2 launch pinky_cartographer cartographer.launch.xml
```
#### map view 
```
ros2 launch pinky_cartographer map_view.launch.xml
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

Navigation2 
-------------
#### launch cartographer
```
ros2 launch pinky_navigation bringup_launch.xml map:=<map name>
```
#### nav2 view
```
ros2 launch pinky_navigation nav2_view.launch.xml
```



