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
from std_msgs.msg import Empty


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
        self.declare_parameter('port', '/dev/ttyAMA0')
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
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('publish_tf', True)

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
        self.sub_reset = self.create_subscription(Empty, '/reset_odom', self.cb_reset_odom, 10)
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
        self._need_sync = True

        # Open serial
        self._open_serial()

        # Timer ส่งคำสั่ง
        period = 1.0 / max(1.0, self.tx_rate)
        self.tx_timer = self.create_timer(period, self._tx_timer_cb)

        self.get_logger().info(
            f"ESP32Bridge ready. port={self.port} baud={self.baud} "
            f"wheel_r={self.wheel_r} wheel_base={self.wheel_base} cprL={self.cprL} cprR={self.cprR}"
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

            time.sleep(0.2)
            self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            raise

        # RX thread
        self._rx_stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ✅ กันมอเตอร์กระชากตอนเริ่ม
        self._send_line("stopv")
        time.sleep(0.05)

        # ✅ รีเซ็ต encoder/odom ที่ ESP32 ทุกครั้งที่ bridge เริ่ม
        self._send_line("z")

        with self._lock:
            # รีเซ็ต state ฝั่ง ROS + encoder sync พร้อมกันใน lock เดียว
            self._x = 0.0
            self._y = 0.0
            self._yaw = 0.0

            self._need_sync = True
            self._last_enc_ms = None
            self._last_countL = None
            self._last_countR = None

        time.sleep(0.10)

        # ✅ reset TX state
        with self._lock:
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

        # ---------- state update (lock) ----------
        with self._lock:
            if self._need_sync:
                self._last_enc_ms = t_ms
                self._last_countL = cL
                self._last_countR = cR
                self._need_sync = False
                return

            if self._last_enc_ms is None:
                self._last_enc_ms = t_ms
                self._last_countL = cL
                self._last_countR = cR
                return

            dt_ms = t_ms - self._last_enc_ms
            if dt_ms <= 0:
                return

            dt = dt_ms / 1000.0
            dL = cL - self._last_countL
            dR = cR - self._last_countR

            self._last_enc_ms = t_ms
            self._last_countL = cL
            self._last_countR = cR

        # ---------- compute (no lock) ----------
        distL = float(dL) / self.ticks_per_m_L
        distR = float(dR) / self.ticks_per_m_R
        v = (distR + distL) * 0.5 / dt
        w = (distR - distL) / self.wheel_base / dt

        # ---------- integrate pose (lock) ----------
        with self._lock:
            yaw_mid = self._yaw + (w * dt * 0.5)
            self._x += v * math.cos(yaw_mid) * dt
            self._y += v * math.sin(yaw_mid) * dt
            self._yaw += w * dt
            self._yaw = self._wrap_pi(self._yaw)   # <- wrap ตรงนี้

            x, y, yaw = self._x, self._y, self._yaw  # <- ต้องมี!


        # ---------- publish (no lock) ----------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)

        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0

        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

        if self.tf_br is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.frame_odom
            t.child_frame_id = self.frame_base
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0

            # ✅ เพิ่มให้ครบ
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw * 0.5)
            t.transform.rotation.w = math.cos(yaw * 0.5)

            self.tf_br.sendTransform(t)

    # ----------------------------
    # TX timer (send cmd to ESP32)
    # ----------------------------
    def _tx_timer_cb(self):
        if self.ser is None:
            return

        now = time.time()

        with self._lock:
            v = self._cmd_v
            w = self._cmd_w
            t_last = self._last_cmd_time
            last_tx = self._last_tx
            stopped = self._stopped
            last_stop_sent = self._last_stop_sent

        timed_out = (t_last == 0.0) or ((now - t_last) > self.cmd_timeout)

        if timed_out or (abs(v) < 1e-3 and abs(w) < 1e-3):
            if (not stopped) and ((now - last_stop_sent) > 0.15):
                self._send_line("stopv")
                with self._lock:
                    self._stopped = True
                    self._last_tx = "stopv"
                    self._last_stop_sent = now
            return

        cmd = f"v {v:.3f} {w:.3f}"

        if cmd != last_tx:
            self._send_line(cmd)
            with self._lock:
                self._last_tx = cmd

        with self._lock:
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

    def cb_reset_odom(self, msg: Empty):
        with self._lock:
            self._cmd_v = 0.0
            self._cmd_w = 0.0
            self._last_cmd_time = 0.0

            self._x = 0.0
            self._y = 0.0
            self._yaw = 0.0
            self._last_enc_ms = None
            self._last_countL = None
            self._last_countR = None
            self._need_sync = True

            self._stopped = True
            self._last_tx = "stopv"
            self._last_stop_sent = time.time()

        self._send_line("stopv")
        time.sleep(0.02)
        self._send_line("z")
        with self._lock:
            self._need_sync = True
            self._last_enc_ms = None
            self._last_countL = None
            self._last_countR = None
        self.get_logger().warn("RESET_ODOM: sent stopv + z, cleared ROS odom state.")

    def _wrap_pi(self, a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

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