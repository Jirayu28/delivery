#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class FakeSensors(Node):
    def __init__(self):
        super().__init__('fake_sensors')
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_imu  = self.create_publisher(Imu, '/imu/data', 10)

        self.create_timer(0.1, self.pub_odom_cb)   # 10 Hz
        self.create_timer(0.02, self.pub_imu_cb)   # 50 Hz

    def pub_odom_cb(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(msg)

    def pub_imu_cb(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w = 1.0
        msg.angular_velocity.z = 0.0
        self.pub_imu.publish(msg)

def main():
    rclpy.init()
    node = FakeSensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
