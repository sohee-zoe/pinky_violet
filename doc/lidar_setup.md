sudo raspi-config
```
```
Interface Options->Serial Port로 이동하여, 
"Would you like a login shell to be accessible over serial?"에 No, 
"Would you like the serial port hardware to be enabled?" "Yes로 설정합니다.
```
```
sudo reboot
```
## 3. lidar 설치- https://github.com/Slamtec/sllidar_ros2 참고
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws/
source /opt/ros/<rosdistro>/setup.bash
colcon build --symlink-install
echo "source <ros2_ws>/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. sh 파일 수정 
```
sudo nano ~/ros2_ws/src/sllidar_ros2/scripts/create_udev_rules.sh
```

```
#!/bin/bash

echo "remap the devices serial port(ttySX) to  RPLIDAR"
echo "devices connection as /dev/PRLIDAR, check it using the command : ls -l /dev|grep -e -e ttyS0"
sudo cp $HOME/ros2_ws/src/sllidar_ros2/scripts/rplidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish "
```

## 5. rules 수정
### 
```
sudo nano ~/ros2_ws/src/sllidar_ros2/scripts/rplidar.rules
```
```
# set the udev rule , make the device_port be fixed by rplidar
#
#KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"

KERNEL=="ttyS0", MODE:="0777", SYMLINK+="RPLIDAR"
```

## 6. sh 적용
```
sh ~/ros2_ws/src/sllidar_ros2/scripts/create_udev_rules.sh
```
## 7. 설정 터미널 확인 
```
ls -al /dev/RPLIDAR
```
```
lrwxrwxrwx 1 root root 5 May 24 11:30 /dev/RPLIDAR -> ttyS0
```

## 8.launch 파일 수정(15 line)
```
 sudo nano ~/colcon_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py
 ```

### -수정 전 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/USB0')
```

### -수정 후 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/RPLIDAR')
```
## 9. Lidar실행 및 터미널 확인
```
ros2 launch sllidar_ros2 sllidar_c1_launch.py
