# colcon build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# run Node
colcon build --symlink-install
source install/setup.bash
ros2 run delivery_controller esp32_bridge
ros2 run delivery_controller esp32_bridge --ros-args -p port:=/dev/ttyUSB0

# ปิด tf Odom ตอนใช้จริง กันชนกับ efk
ros2 run delivery_controller esp32_bridge --ros-args -p publish_tf:=false


# lidar
ros2 launch delivery_bringup ydlidar_x3.launch.py 

# Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# URDF
ros2 launch delivery_description description.launch.py

# เปิด GUI (ตอนพัฒนา)
ros2 launch delivery_description description.launch.py use_gui:=true

# ปิด GUI (ตอนใช้งานจริง / ไม่มีจอ)
ros2 launch delivery_description description.launch.py use_gui:=false

# efk -> Odom
ros2 launch delivery_localization ekf.launch.py

# slam
ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/home/jirayu/ros2_ws/src/delivery_localization/config/slam_params.yaml




# ต่อไปเวลาแก้โค้ดแล้ว push branch นี้
git branch
git add .
git commit -m "your message"
git push