#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import serial


class ESP32Bridge(Node):
    """
    Bridge ROS2 /cmd_vel <-> ESP32 serial

    ESP32 protocol (ตามโค้ด velMode ของคุณ):
      - v <linear_mps> <angular_radps>
      - stopv
      - ESP32 อาจพิมพ์ E <ms> <countL> <countR> เพื่อทำ odom
    """

    def __init__(self):
        super().__init__('esp32_bridge')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_timeout', 0.5)     # วินาที
        self.declare_parameter('tx_rate', 20.0)        # Hz ส่งลง ESP32
        self.declare_parameter('max_v', 0.6)           # m/s clamp
        self.declare_parameter('max_w', 3.0)           # rad/s clamp

        self.declare_parameter('wheel_r', 0.033)       # เมตร
        self.declare_parameter('wheel_base', 0.314)     # เมตร
        self.declare_parameter('cprL', 989.2)          # counts per rev
        self.declare_parameter('cprR', 989.2)
        self.declare_parameter('ticks_per_m_L', 4860.0)
        self.declare_parameter('ticks_per_m_R', 4860.0)

        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_footprint')
        self.declare_parameter('publish_tf', True)  # ปิด tf ของ odom ตอนใช้จริง

        self.declare_parameter('log_tx', True)         # log TX
        self.declare_parameter('log_esp', True)        # log non-encoder lines from ESP32

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.tx_rate = float(self.get_parameter('tx_rate').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)

        self.wheel_r = float(self.get_parameter('wheel_r').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.cprL = float(self.get_parameter('cprL').value)
        self.cprR = float(self.get_parameter('cprR').value)
        self.ticks_per_m_L = float(self.get_parameter('ticks_per_m_L').value)
        self.ticks_per_m_R = float(self.get_parameter('ticks_per_m_R').value)
        self._warned_ticks = False
        self._warned_base = False

        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.log_tx = bool(self.get_parameter('log_tx').value)
        self.log_esp = bool(self.get_parameter('log_esp').value)

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        # ----------------------------
        # State (cmd_vel)
        # ----------------------------
        self._lock = threading.Lock()
        self._last_cmd_time = 0.0
        self._cmd_v = 0.0
        self._cmd_w = 0.0

        # ✅ สำคัญ: กัน stopv spam + กันส่งซ้ำเดิม ๆ
        self._stopped = True          # เริ่มต้นถือว่า "หยุด"
        self._last_tx = ""            # เก็บบรรทัดล่าสุดที่ส่ง
        self._last_stop_sent = 0.0    # เวลา stopv ล่าสุด (กันถี่ ๆ)

        # ----------------------------
        # Odom integration (from encoder)
        # ----------------------------
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._last_enc_ms = None
        self._last_countL = None
        self._last_countR = None

        # ----------------------------
        # Serial
        # ----------------------------
        self.ser = None
        self._rx_thread = None
        self._rx_stop = False

        # Open serial
        self._open_serial()

        # Timer ส่งคำสั่ง
        period = 1.0 / max(1.0, self.tx_rate)
        self.tx_timer = self.create_timer(period, self._tx_timer_cb)

        self.get_logger().info(
            f"ESP32Bridge ready. port={self.port} baud={self.baud} "
            f"wheel_base={self.wheel_base} ticks_per_m_L={self.ticks_per_m_L} ticks_per_m_R={self.ticks_per_m_R}"
        )

    # ----------------------------
    # ROS callbacks
    # ----------------------------
    def cb_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # clamp
        v = max(-self.max_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, w))

        with self._lock:
            self._cmd_v = v
            self._cmd_w = w
            self._last_cmd_time = time.time()

    # ----------------------------
    # Serial open / RX loop
    # ----------------------------
    def _open_serial(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.05,
                write_timeout=0.2
            )
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            # บางบอร์ดเปิดพอร์ตแล้วรีเซ็ต -> รอหน่อย
            time.sleep(0.2)

            self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            raise

        # RX thread
        self._rx_stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # กันมอเตอร์กระชากตอนเริ่ม
        self._send_line("stopv")
        self._stopped = True
        self._last_tx = "stopv"
        self._last_stop_sent = time.time()

    def _rx_loop(self):
        buf = b""
        while not self._rx_stop and rclpy.ok():
            try:
                b = self.ser.read(256)
                if not b:
                    continue
                buf += b

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors='ignore').strip()
                    if not s:
                        continue
                    self._handle_esp_line(s)

            except Exception as e:
                self.get_logger().error(f"Serial RX error: {e}")
                time.sleep(0.2)

    def _handle_esp_line(self, s: str):
        # Encoder: "E <ms> <countL> <countR>"
        if s.startswith("E "):
            parts = s.split()
            if len(parts) >= 4:
                try:
                    t_ms = int(parts[1])
                    cL = int(parts[2])
                    cR = int(parts[3])
                    self._on_encoder(t_ms, cL, cR)
                    return
                except Exception:
                    pass

        if self.log_esp:
            self.get_logger().info(f"ESP32: {s}")

    def _on_encoder(self, t_ms: int, cL: int, cR: int):
        now = self.get_clock().now()

        if self._last_enc_ms is None:
            self._last_enc_ms = t_ms
            self._last_countL = cL
            self._last_countR = cR
            return

        dt_ms = t_ms - self._last_enc_ms
        if dt_ms <= 0:
            # รีเซ็ตให้ sync ใหม่
            self._last_enc_ms = t_ms
            self._last_countL = cL
            self._last_countR = cR
            return  # ข้ามถ้าเวลาไม่เพิ่ม
        dt = dt_ms / 1000.0
        if dt > 0.5:
            self._last_enc_ms = t_ms
            self._last_countL = cL
            self._last_countR = cR
            return  # ข้ามถ้านานเกินไป

        
        if self.ticks_per_m_L <= 0.0 or self.ticks_per_m_R <= 0.0:
            if not self._warned_ticks:
                self.get_logger().warn("ticks_per_m is <= 0, check parameters!")
                self._warned_ticks = True
            return
        else:
            self._warned_ticks = False
        
        if self.wheel_base <= 0.0:
            if not self._warned_base:
                self.get_logger().warn("wheel_base is <= 0, check parameters!")
                self._warned_base = True
            return
        else:
            self._warned_base = False

        dL = cL - self._last_countL
        dR = cR - self._last_countR

        self._last_enc_ms = t_ms
        self._last_countL = cL
        self._last_countR = cR

        # counts -> meters
        # ticks -> meters (ใช้ค่าที่คาลิเบรตมา)
        distL = float(dL) / self.ticks_per_m_L
        distR = float(dR) / self.ticks_per_m_R

        v = (distR + distL) * 0.5 / dt
        w = (distR - distL) / self.wheel_base / dt

        # ✅ deadband กันสั่น/กัน drift ตอนหยุด (ปรับค่าได้)
        if abs(v) < 1e-3:
            v = 0.0
        if abs(w) < 1e-3:
            w = 0.0

        # integrate pose
        dtheta = w * dt
        yaw_mid = self._yaw + 0.5 * dtheta
        self._x += v * math.cos(yaw_mid) * dt
        self._y += v * math.sin(yaw_mid) * dt
        self._yaw += dtheta
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self._yaw * 0.5)
        qw = math.cos(self._yaw * 0.5)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

        if self.tf_br is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.frame_odom
            t.child_frame_id = self.frame_base
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_br.sendTransform(t)

    # ----------------------------
    # TX timer (send cmd to ESP32)
    # ----------------------------
    def _tx_timer_cb(self):
        if self.ser is None:
            return

        with self._lock:
            v = self._cmd_v
            w = self._cmd_w
            t_last = self._last_cmd_time

        now = time.time()
        timed_out = (t_last == 0.0) or ((now - t_last) > self.cmd_timeout)

        # ถ้า timeout หรือสั่ง (0,0) -> stopv (ส่ง "ครั้งเดียว" และกันถี่)
        if timed_out or (abs(v) < 1e-3 and abs(w) < 1e-3):
            if (not self._stopped) and ((now - self._last_stop_sent) > 0.15):
                self._send_line("stopv")
                self._stopped = True
                self._last_tx = "stopv"
                self._last_stop_sent = now
            return

        # มีคำสั่ง -> ส่ง v w (หน่วย m/s, rad/s)
        cmd = f"v {v:.3f} {w:.3f}"

        # กันส่งซ้ำเดิม ๆ (ช่วยลด spam log)
        if cmd != self._last_tx:
            self._send_line(cmd)
            self._last_tx = cmd

        self._stopped = False

    def _send_line(self, s: str):
        try:
            msg = (s + "\n").encode("utf-8")
            self.ser.write(msg)
            self.ser.flush()
            if self.log_tx:
                self.get_logger().info(f"TX -> ESP32: {s}")
        except Exception as e:
            self.get_logger().error(f"Serial TX error: {e}")

    # ----------------------------
    # Shutdown
    # ----------------------------
    def destroy_node(self):
        try:
            self._rx_stop = True
            if self._rx_thread is not None:
                self._rx_thread.join(timeout=0.5)
        except Exception:
            pass

        try:
            if self.ser is not None:
                try:
                    self._send_line("stopv")
                except Exception:
                    pass
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
