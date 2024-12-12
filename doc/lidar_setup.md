# RPLIDAR C1 SETUP

##  1.UART 설정
#### /boot/firmware/config.txt 열기
```
sudo nano /boot/firmware/config.txt
```
#### 맨아래 추가
```
dtparam=uart0=on
```
#### 재부팅
```
sudo reboot
```
#### AMA0가 생겼는지 확인
```
ls /dev/ttyAMA*
```
## 2. lidar 설치- https://github.com/Slamtec/sllidar_ros2 참고
```
cd ~/pinky_violet/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/pinky_violet/
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
```

## 3. lidar udev 설정
#### rplidar.rules 파일을 만들고
```
sudo nano /etc/udev/rules.d/rplidar.rules
```
#### 다음내용 추가
```
# set the udev rule , make the device_port be fixed by rplidar
#
KERNEL=="ttyAMA0", MODE:="0777", SYMLINK+="RPLIDAR"
```

## 4. rulse 적용
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
## 5. 설정 터미널 확인 
```
ls -al /dev/RPLIDAR
```
```
lrwxrwxrwx 1 root root 7 Dec 12 15:50 /dev/RPLIDAR -> ttyAMA0
```
위와 같이 출력이 안되면 재부팅후 다시확인
```
sudo reboot
```

## 6.launch 파일 수정(15, 17 line)
```
 sudo nano ~/pinky_violet/src/sllidar_ros2/launch/sllidar_c1_launch.py
 ```

### -수정 전 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/USB0')

frame_id = LaunchConfiguration('frame_id', default='laser')
```

### -수정 후 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/RPLIDAR')

frame_id = LaunchConfiguration('frame_id', default='laser_link')
```
## 7. Lidar실행 및 터미널 확인
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
