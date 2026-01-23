# colcon build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# run Node
colcon build --symlink-install
source install/setup.bash
ros2 run delivery_controller esp32_bridge
ros2 run delivery_controller esp32_bridge --ros-args -p port:=/dev/ttyUSB0

ros2 run delivery_controller esp32_bridge --ros-args -p port:=/dev/ttyAMA0 -p baud:=115200

ros2 run delivery_controller esp32_bridge --ros-args -p port:=/dev/serial0 -p baud:=115200

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

cd ros2_ws/
source install/setup.bash 

# ต่อไปเวลาแก้โค้ดแล้ว push branch นี้
git branch
git add .
git commit -m "your message"
git push

# URDF
ros2 launch delivery_description description.launch.py
# ปิด GUI (ตอนใช้งานจริง / ไม่มีจอ)
ros2 launch delivery_description description.launch.py use_gui:=false


# esp32
ros2 run delivery_controller esp32_bridge --ros-args -p port:=/dev/ttyAMA0 -p baud:=115200

# imu
ros2 launch bno055 bno055.launch.py

# efk -> Odom
ros2 launch delivery_localization ekf.launch.py

# lidar
ros2 launch delivery_bringup ydlidar_x3.launch.py 

# Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# slam
ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/home/jirayu/ros2_ws/src/delivery_localization/config/slam_params.yaml