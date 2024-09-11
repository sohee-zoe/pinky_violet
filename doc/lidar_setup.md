# RPLIDAR C1 SETUP

##  1.라즈베리파이 설정
```
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
## 2. lidar 설치- https://github.com/Slamtec/sllidar_ros2 참고
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

## 3. sh 파일 수정 
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

## 4. rules 수정
#### rplidar.rules 파일을 만들고
```
sudo nano ~/ros2_ws/src/sllidar_ros2/scripts/rplidar.rules
```
#### 다음내용 추가
```
# set the udev rule , make the device_port be fixed by rplidar
#
#KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"

KERNEL=="ttyS0", MODE:="0777", SYMLINK+="RPLIDAR"
```

## 5. sh 적용
```
sh ~/ros2_ws/src/sllidar_ros2/scripts/create_udev_rules.sh
```
## 6. 설정 터미널 확인 
```
ls -al /dev/RPLIDAR
```
```
lrwxrwxrwx 1 root root 5 May 24 11:30 /dev/RPLIDAR -> ttyS0
```

## 7.launch 파일 수정(15 line)
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
## 8. Lidar실행 및 터미널 확인
```
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```
```
[INFO] [launch]: All log files can be found below /home/pi/.ros/log/2024-05-24-11-33-27-987100-raspberrypi-1131
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [sllidar_node-1]: process started with pid [1134]
[sllidar_node-1] [INFO] [1716518008.416250815] [sllidar_node]: SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:1.0.1, SLLIDAR SDK Version:2.1.0
[sllidar_node-1] [INFO] [1716518008.919035964] [sllidar_node]: SLLidar S/N: EBE2E1F4C2E398C0BCEA9AF365394806
[sllidar_node-1] [INFO] [1716518008.919179575] [sllidar_node]: Firmware Ver: 1.01
[sllidar_node-1] [INFO] [1716518008.919226575] [sllidar_node]: Hardware Rev: 18
[sllidar_node-1] [INFO] [1716518008.919647703] [sllidar_node]: SLLidar health status : 0
[sllidar_node-1] [INFO] [1716518008.919734814] [sllidar_node]: SLLidar health status : OK.
[sllidar_node-1] [INFO] [1716518009.150550095] [sllidar_node]: current scan mode: Standard, sample rate: 5 Khz, max_distance: 16.0 m, scan frequency:10.0 Hz,

```
