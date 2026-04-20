本项目基于中南FYT二次修改
双相机自瞄，增加了后相机，减小了视野盲区。
启动方式
1. 启动双相机包
ros2 launch hik_camera hik_camera.launch.py
2. 启动串口包
ros2 launch serial_def_sdk serial.launch.py
（或虚拟串口）
ros2 launch virtual_serial_driver virtual_serial.launch.py
3. 启动自瞄
ros2 launch rm_bringup bringup.launch.py
