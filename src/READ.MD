# ลำดับ “ของจริง” ที่ถูกต้อง (สำคัญมาก)
# 1️⃣ Description (ไม่ต้อง GUI)
ros2 launch delivery_description description.launch.py \
  use_gui:=false use_rviz:=false

# 2️⃣ ESP32 Bridge (ปิด TF ❗)
ros2 run delivery_controller esp32_bridge \
  --ros-args -p publish_tf:=false -p port:=/dev/ttyAMA0
เช็ก:
ros2 topic echo /odom --once

# 3️⃣ IMU (BNO055)w
รัน node IMU ของคุณ (เช่น):
ros2 launch bno055_driver imu.launch.py
เช็ก:
ros2 topic echo /imu/data --once

# 4️⃣ LiDAR
ros2 launch delivery_bringup ydlidar_x3.launch.py
เช็ก:
ros2 topic echo /scan --once

# 5️⃣ EKF (หัวใจของระบบ)
ros2 launch delivery_localization ekf.launch.py
เช็ก:
ros2 topic echo /odometry/filtered --once
และ:
ros2 run tf2_tools view_frames

# ต้องเห็น:
odom -> base_link   (จาก EKF)
base_link -> lidar_link
base_link -> wheels

# 6️⃣ เปิด RViz (ดูรวมทั้งหมด)
rviz2

ตั้ง:
Fixed Frame = odom
Add:
TF
RobotModel
LaserScan
Odometry