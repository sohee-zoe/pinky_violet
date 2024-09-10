pinky_violet
==============
<img src="/doc/Screenshot from 2024-04-26 19-44-15.png" width="40%" height="30%" title="pinky" alt="pinky"></img>

pinky 설정
---
```
mkidr -p ~/pinky_violet/src
cd ~/pinky_violet/src
git clone https://github.com/pinklab-art/pinky_violet.git
````

pinky gpio 권한 설정
-------------
```
cd ~/pinky_violet/src/pinky_violet
sudo cp ./99-gpio.rules /etc/udev/rules.d/
```

pinky_violet pkg build
---
```
cd ~/pinky_violet
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

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



