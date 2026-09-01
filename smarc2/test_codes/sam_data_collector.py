import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

import numpy as np
import math
import time
import random
import atexit
import signal

def quaternion_to_euler(x, y, z, w):
    """Converts quaternion orientation to Euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class SAMDataCollector(Node):
    def __init__(self):
        super().__init__('sam_data_collector')

        # Configuration
        self.target_sam = 'sam_1'  # Change to 'sam_2' if needed
        self.dt = 0.1              # 10 Hz publishing/sampling loop
        self.file_output = 'sam_koopman_dataset.npz'
        
        # --- Publishers for All 6 Control Inputs ---
        self.pub_t1 = self.create_publisher(Float32, f'/{self.target_sam}/core/standard/thruster1_cmd', 10)
        self.pub_t2 = self.create_publisher(Float32, f'/{self.target_sam}/core/standard/thruster2_cmd', 10)
        self.pub_tv_pitch = self.create_publisher(Float32, f'/{self.target_sam}/core/standard/thrust_vector_pitch_cmd', 10)
        self.pub_tv_yaw = self.create_publisher(Float32, f'/{self.target_sam}/core/standard/thrust_vector_yaw_cmd', 10)
        self.pub_vbs = self.create_publisher(Float32, f'/{self.target_sam}/core/standard/vbs_cmd', 10)
        self.pub_lcg = self.create_publisher(Float32, f'/{self.target_sam}/core/standard/lcg_cmd', 10)

        # --- Odometry Subscriber ---
        self.sub_odom = self.create_subscription(
            Odometry, 
            f'/{self.target_sam}/smarc/odom', 
            self.odom_callback, 
            10
        )

        # Active Commands: [rpm1, rpm2, pitch, yaw, vbs, lcg]
        self.current_cmd = np.array([0.0, 0.0, 0.0, 0.0, 50.0, 50.0])
        
        # Data Buffers
        self.time_buffer = []
        self.state_buffer = []  # 12-DOF: [x, y, z, roll, pitch, yaw, u, v, w, p, q, r]
        self.input_buffer = []  # 6 inputs: [rpm1, rpm2, pitch, yaw, vbs, lcg]

        self.latest_odom_state = None
        self.start_time = time.time()
        self.data_saved = False

        # --- Timers ---
        # 1. 10 Hz Control Publish & Log Loop
        self.create_timer(self.dt, self.control_and_record_loop)
        # 2. Excitation Randomizer (Generates new profiles every 6 to 12 seconds)
        self.create_timer(20.0, self.randomize_control_inputs)
        
        # Initial command excitation
        self.randomize_control_inputs()

        # Handle clean saving on shutdown
        atexit.register(self.save_dataset)
        self.get_logger().info(f"Collector started for {self.target_sam}. Publishing excitation signals...")

    def randomize_control_inputs(self):
        """Generates diverse control input combinations to excite 6-DOF dynamics."""
        # 1. Propellers: Forward/reverse thrust and differential RPM for roll/yaw excitation
        base_rpm = random.uniform(100.0, 600.0)
        differential_rpm = random.uniform(-100.0, 100.0)
        self.current_cmd[0] = np.clip(base_rpm + differential_rpm, -300.0, 800.0)  # Thruster 1
        self.current_cmd[1] = self.current_cmd[0]  # Thruster 2

        # 2. Thrust Vectoring: Pitch and Yaw angles (radians, within +/- 0.2 rad)
        self.current_cmd[2] = random.uniform(-0.18, 0.18)  # Pitch vector
        self.current_cmd[3] = random.uniform(-0.18, 0.18)  # Yaw vector

        # 3. Parameter-Varying Actuators: Buoyancy and Mass Balance (%)
        self.current_cmd[4] = random.uniform(10.0, 90.0)   # VBS
        self.current_cmd[5] = random.uniform(10.0, 90.0)   # LCG

        self.get_logger().info(
            f"[Input Step] RPM: ({self.current_cmd[0]:.0f}, {self.current_cmd[1]:.0f}) | "
            f"TV (P/Y): ({self.current_cmd[2]:.2f}, {self.current_cmd[3]:.2f}) | "
            f"VBS/LCG: ({self.current_cmd[4]:.1f}%, {self.current_cmd[5]:.1f}%)"
        )

    def odom_callback(self, msg):
        """Processes and converts Odometry messages into standard NED vehicle states."""
        # Frame transformation (Unity Odom -> NED coordinate conventions)
        x = msg.pose.pose.position.x
        y = -msg.pose.pose.position.y
        z = -msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        pitch = -pitch
        yaw = -yaw

        u = msg.twist.twist.linear.x
        v = -msg.twist.twist.linear.y
        w = -msg.twist.twist.linear.z

        p = msg.twist.twist.angular.x
        q_rate = -msg.twist.twist.angular.y
        r = -msg.twist.twist.angular.z

        self.latest_odom_state = np.array([x, y, z, roll, pitch, yaw, u, v, w, p, q_rate, r])

    def control_and_record_loop(self):
        """Publishes control commands at 10 Hz and logs the synchronized (X, U) pair."""
        # 1. Publish commands to vehicle topics
        self.pub_t1.publish(Float32(data=float(self.current_cmd[0])))
        self.pub_t2.publish(Float32(data=float(self.current_cmd[1])))
        self.pub_tv_pitch.publish(Float32(data=float(self.current_cmd[2])))
        self.pub_tv_yaw.publish(Float32(data=float(self.current_cmd[3])))
        self.pub_vbs.publish(Float32(data=float(self.current_cmd[4])))
        self.pub_lcg.publish(Float32(data=float(self.current_cmd[5])))

        # 2. Record data if odometry has arrived
        if self.latest_odom_state is not None:
            t = time.time() - self.start_time
            self.time_buffer.append(t)
            self.state_buffer.append(self.latest_odom_state.copy())
            self.input_buffer.append(self.current_cmd.copy())

    def save_dataset(self):
        """Flushes recorded buffers into a .npz file."""
        if self.data_saved or len(self.state_buffer) == 0:
            return

        self.data_saved = True
        X = np.array(self.state_buffer)  # Shape: (N, 12)
        U = np.array(self.input_buffer)  # Shape: (N, 6)
        t = np.array(self.time_buffer)   # Shape: (N,)

        np.savez_compressed(self.file_output, X=X, U=U, t=t)
        print(f"\n[SUCCESS] Saved {len(X)} synchronized samples to '{self.file_output}'.")

def main(args=None):
    rclpy.init(args=args)
    node = SAMDataCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        node.save_dataset()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()