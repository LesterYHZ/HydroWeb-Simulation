import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32  

# import matplotlib
# matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec
from collections import deque
import threading
import math
import time

def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class FullStatePlotterNode(Node):
    def __init__(self):
        super().__init__('full_state_plotter_node')
        
        max_pts = 1000 
        self.start_time = time.time()
        
        self.sam1_data = self._init_data_dict(max_pts)
        self.sam2_data = self._init_data_dict(max_pts)

        # Odometry Subscribers
        self.create_subscription(Odometry, '/sam_1/smarc/odom', self.sam1_odom_cb, 10)

        # Feedback Subscribers (SAM 1)
        self.create_subscription(Float32, '/sam_1/core/standard/lcg_fb', self.make_cb(self.sam1_data, 't_lcg', 'lcg'), 10)
        self.create_subscription(Float32, '/sam_1/core/standard/vbs_fb', self.make_cb(self.sam1_data, 't_vbs', 'vbs'), 10)
        self.create_subscription(Float32, '/sam_1/core/standard/thruster1_fb', self.make_cb(self.sam1_data, 't_t1', 't1'), 10)
        self.create_subscription(Float32, '/sam_1/core/standard/thruster2_fb', self.make_cb(self.sam1_data, 't_t2', 't2'), 10)
    
        self.get_logger().info("Subscribed to Odometry and Control Feedbacks for SAM 1")

    def _init_data_dict(self, maxlen):
        return {
            't_odom': deque(maxlen=maxlen),
            'x': deque(maxlen=maxlen), 'y': deque(maxlen=maxlen), 'z': deque(maxlen=maxlen),
            'roll': deque(maxlen=maxlen), 'pitch': deque(maxlen=maxlen), 'yaw': deque(maxlen=maxlen),
            'vx': deque(maxlen=maxlen), 'vy': deque(maxlen=maxlen), 'vz': deque(maxlen=maxlen),
            'wx': deque(maxlen=maxlen), 'wy': deque(maxlen=maxlen), 'wz': deque(maxlen=maxlen),
            # Independent time buffers for async feedback topics
            't_lcg': deque(maxlen=maxlen), 'lcg': deque(maxlen=maxlen),
            't_vbs': deque(maxlen=maxlen), 'vbs': deque(maxlen=maxlen),
            't_t1': deque(maxlen=maxlen),  't1': deque(maxlen=maxlen),
            't_t2': deque(maxlen=maxlen),  't2': deque(maxlen=maxlen)
        }

    def make_cb(self, data_dict, t_key, val_key):
        """Helper to generate callbacks for simple float topics"""
        def callback(msg):
            current_time = time.time() - self.start_time
            data_dict[t_key].append(current_time)
            data_dict[val_key].append(msg.data)
        return callback

    def extract_odom_data(self, msg, data_dict):
        current_time = time.time() - self.start_time
        
        x = msg.pose.pose.position.x
        y = - msg.pose.pose.position.y
        z = - msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        pitch = - pitch
        yaw = - yaw
        
        vx = msg.twist.twist.linear.x
        vy = - msg.twist.twist.linear.y
        vz = - msg.twist.twist.linear.z
        
        wx = msg.twist.twist.angular.x
        wy = - msg.twist.twist.angular.y
        wz = - msg.twist.twist.angular.z
        
        data_dict['t_odom'].append(current_time)
        data_dict['x'].append(x);  data_dict['y'].append(y);  data_dict['z'].append(z)
        data_dict['roll'].append(roll); data_dict['pitch'].append(pitch); data_dict['yaw'].append(yaw)
        data_dict['vx'].append(vx); data_dict['vy'].append(vy); data_dict['vz'].append(vz)
        data_dict['wx'].append(wx); data_dict['wy'].append(wy); data_dict['wz'].append(wz)

    def sam1_odom_cb(self, msg): self.extract_odom_data(msg, self.sam1_data)
    def sam2_odom_cb(self, msg): self.extract_odom_data(msg, self.sam2_data)


# --- Matplotlib Dashboard Setup ---
fig = plt.figure(figsize=(20, 14))
fig.canvas.manager.set_window_title('SAM 1: Full State & Controls')

# 5x4 Grid Layout
gs = gridspec.GridSpec(5, 4, figure=fig)

ax_x     = fig.add_subplot(gs[0, 0])
ax_y     = fig.add_subplot(gs[0, 1])
ax_z     = fig.add_subplot(gs[0, 2])

ax_roll  = fig.add_subplot(gs[1, 0])
ax_pitch = fig.add_subplot(gs[1, 1])
ax_yaw   = fig.add_subplot(gs[1, 2])

ax_vx    = fig.add_subplot(gs[2, 0])
ax_vy    = fig.add_subplot(gs[2, 1])
ax_vz    = fig.add_subplot(gs[2, 2])

ax_wx    = fig.add_subplot(gs[3, 0])
ax_wy    = fig.add_subplot(gs[3, 1])
ax_wz    = fig.add_subplot(gs[3, 2])

# Control Inputs (Bottom Row)
ax_lcg   = fig.add_subplot(gs[4, 0])
ax_vbs   = fig.add_subplot(gs[4, 1])
ax_t1    = fig.add_subplot(gs[4, 2])
ax_t2    = fig.add_subplot(gs[4, 3])

# Trajectories (Right-most Column)
ax_2d    = fig.add_subplot(gs[0:2, 3])
ax_3d    = fig.add_subplot(gs[2:4, 3], projection='3d')

def update_plot(frame, node):
    all_axes = [ax_x, ax_y, ax_z, ax_roll, ax_pitch, ax_yaw, 
                ax_vx, ax_vy, ax_vz, ax_wx, ax_wy, ax_wz, 
                ax_lcg, ax_vbs, ax_t1, ax_t2, ax_2d, ax_3d]
    
    for ax in all_axes: ax.cla()

    d1 = node.sam1_data

    def plot_pair(ax, t_key, y_key, title, ylabel):
        ax.plot(d1[t_key], d1[y_key], label='SAM 1', color='blue')
        ax.set_title(title, fontsize=9)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True)

    # 1. Position States
    plot_pair(ax_x, 't_odom', 'x', 'Position X', 'X (m)')
    plot_pair(ax_y, 't_odom', 'y', 'Position Y', 'Y (m)')
    plot_pair(ax_z, 't_odom', 'z', 'Position Z', 'Z (m)')

    # 2. Orientation States
    plot_pair(ax_roll, 't_odom', 'roll', 'Orientation Roll', 'Roll (rad)')
    plot_pair(ax_pitch, 't_odom', 'pitch', 'Orientation Pitch', 'Pitch (rad)')
    plot_pair(ax_yaw, 't_odom', 'yaw', 'Orientation Yaw', 'Yaw (rad)')

    # 3. Linear Body Velocities
    plot_pair(ax_vx, 't_odom', 'vx', 'Linear Vel Vx', 'Vx (m/s)')
    plot_pair(ax_vy, 't_odom', 'vy', 'Linear Vel Vy', 'Vy (m/s)')
    plot_pair(ax_vz, 't_odom', 'vz', 'Linear Vel Vz', 'Vz (m/s)')

    # 4. Angular Body Velocities
    plot_pair(ax_wx, 't_odom', 'wx', 'Angular Vel Wx', 'Wx (rad/s)')
    plot_pair(ax_wy, 't_odom', 'wy', 'Angular Vel Wy', 'Wy (rad/s)')
    plot_pair(ax_wz, 't_odom', 'wz', 'Angular Vel Wz', 'Wz (rad/s)')

    # 5. Control Feedback
    plot_pair(ax_lcg, 't_lcg', 'lcg', 'LCG Position', 'Pos (%)')
    plot_pair(ax_vbs, 't_vbs', 'vbs', 'VBS Volume', 'Vol (%)')
    plot_pair(ax_t1, 't_t1', 't1', 'Thruster 1', 'RPM / %')
    plot_pair(ax_t2, 't_t2', 't2', 'Thruster 2', 'RPM / %')

    for ax in [ax_lcg, ax_vbs, ax_t1, ax_t2]:
        ax.set_xlabel('Time (s)', fontsize=8)

    # --- 2D Trajectory Plot (X vs Y) ---
    ax_2d.plot(d1['x'], d1['y'], label='SAM 1', color='blue')
    ax_2d.set_title('2D Trajectory (X-Y)', fontsize=10, fontweight='bold')
    ax_2d.set_xlabel('X Position (m)', fontsize=8)
    ax_2d.set_ylabel('Y Position (m)', fontsize=8)
    ax_2d.legend(loc='upper right')
    ax_2d.grid(True)
    ax_2d.axis('equal')

    # --- 3D Trajectory Plot (X vs Y vs Z) ---
    ax_3d.plot(d1['x'], d1['y'], d1['z'], label='SAM 1', color='blue')
    ax_3d.set_title('3D Trajectory (X-Y-Z)', fontsize=10, fontweight='bold')
    ax_3d.set_xlabel('X (m)', fontsize=8)
    ax_3d.set_ylabel('Y (m)', fontsize=8)
    ax_3d.set_zlabel('Z (m)', fontsize=8)

    plt.tight_layout()

def main(args=None):
    rclpy.init(args=args)
    node = FullStatePlotterNode()

    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    ani = animation.FuncAnimation(fig, update_plot, fargs=(node,), interval=150, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()