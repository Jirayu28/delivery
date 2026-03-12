ssh delivery@192.168.1.110

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
ros2 run delivery_controller esp32_bridge

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


# ..........Open map
ros2 launch delivery_navigation navigation_launch.py

ros2 launch delivery_navigation navigation_launch.py \
  mode:=navigation \
  map:=/home/delivery/ros2_ws/src/delivery_navigation/maps/office_map.yaml \
  params_file:=/home/delivery/ros2_ws/src/delivery_navigation/config/nav2_params.yaml
