# ROS2_rada_driver
Uart Rada driver for ROS2 Foxy/Galactic on Jetson (JP4.6)

# Get the repo
```
git clone https://github.com/zhj-buffer/ROS2_rada_driver.git
```
# Build from source
```
colcon build
```

# launch the rada node
```
ros2 run   ros2_rada_driver_pub  rada_pub 
```

# Get the msg interface
```
nvidia@nvidia-desktop:~/workspace/alan/test$ ros2 interface show ros2_rada_msg/msg/Rada  
uint32 r1  
uint32 r2  
uint32 r3  
uint32 r4  
uint32 crc 
```

# Echo the Rada data from rada1 and rada2
```
ros2 topic echo /rada1
nvidia@nvidia-desktop:~/workspace/alan/test$ ros2 topic echo /rada1
r1: 1892 
r2: 0
r3: 93
r4: 61166
crc: 163
--- 
r1: 1897
r2: 0
r3: 93
r4: 61166
crc: 168
---
r1: 1893 
r2: 0
r3: 133
r4: 61166
crc: 204
```

```
ros2 topic echo /rada1
nvidia@nvidia-desktop:~/workspace/alan/test$ ros2 topic echo /rada2
r1: 1892 
r2: 0
r3: 93
r4: 61166
crc: 163
--- 
r1: 1897
r2: 0
r3: 93
r4: 61166
crc: 168
---
r1: 1893 
r2: 0
r3: 133
r4: 61166
crc: 204
```
