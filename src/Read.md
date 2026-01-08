# run Node
colcon build --symlink-install
source install/setup.bash
ros2 run delivery_controller esp32_bridge --ros-args -p port:=/dev/ttyUSB0

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


# colcon build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
