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

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash


# URDF
ros2 launch delivery_description description.launch.py
# ปิด GUI (ตอนใช้งานจริง / ไม่มีจอ)
ros2 launch delivery_description description.launch.py use_gui:=false

ros2 launch delivery_description description.launch.py use_gui:=false use_rviz:=false


cd ros2_ws/
source install/setup.bash 

# esp32
ros2 run delivery_controller esp32_bridge --ros-args -p port:=/dev/ttyAMA0 -p baud:=115200

# ปิด tf Odom ตอนใช้จริง กันชนกับ efk
ros2 run delivery_controller esp32_bridge --ros-args -p publish_tf:=false

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


# save map
ros2 run nav2_map_server map_saver_cli \ -f ~/ros2_ws/src/delivery_navigation/maps/office_map \ --ros-args -p map_topic:=/map -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0

# run map
ros2 launch delivery_navigation navigation.launch.py

ros2 launch delivery_navigation navigation.launch.py \
  map:=/home/delivery/ros2_ws/src/delivery_navigation/maps/office_map.yaml \
  params_file:=/home/delivery/ros2_ws/src/delivery_navigation/config/nav2_params.yaml

# เปิดภาพแมพได้
ros2 launch nav2_bringup localization_launch.py   map:=/home/delivery/ros2_ws/src/delivery_navigation/maps/office_map.yaml   params_file:=/home/delivery/ros2_ws/src/delivery_navigation/config/nav2_params.yaml   use_sim_time:=false

# เหมือนคำสั่งที่คุณทำแล้วแมพขึ้น
ros2 launch delivery_navigation navigation.launch.py \
  mode:=localization \
  map:=/home/delivery/ros2_ws/src/delivery_navigation/maps/office_map.yaml \
  params_file:=/home/delivery/ros2_ws/src/delivery_navigation/config/nav2_params.yaml \
  use_sim_time:=false

# ใช้ Nav2Goal (ต้องเป็น full navigation)
ros2 launch delivery_navigation navigation.launch.py \
  mode:=navigation \
  map:=/home/delivery/ros2_ws/src/delivery_navigation/maps/office_map.yaml \
  params_file:=/home/delivery/ros2_ws/src/delivery_navigation/config/nav2_params.yaml \
  use_sim_time:=false

# ..........
ros2 launch delivery_navigation navigation_launch.py \
  mode:=navigation \
  map:=/home/delivery/ros2_ws/src/delivery_navigation/maps/office_map.yaml \
  params_file:=/home/delivery/ros2_ws/src/delivery_navigation/config/nav2_params.yaml
