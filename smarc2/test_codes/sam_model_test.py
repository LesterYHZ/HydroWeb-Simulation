import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec

from sam_model import SAM

def run_open_loop_simulation():
    # 1. Simulation Parameters
    dt = 0.1                # Time step (seconds)
    t_end = 60.0            # Total simulation time (seconds)
    time = np.arange(0, t_end, dt)
    n_steps = len(time)

    # 2. Initialize Model and Storage
    sam = SAM(dt=dt)
    
    eta_history = np.zeros((n_steps, 6)) # [x, y, z, phi, theta, psi]
    nu_history = np.zeros((n_steps, 6))  # [u, v, w, p, q, r]
    u_history = np.zeros((n_steps, 6))  # [rpm1, rpm2, d_aileron, d_rudder, vbs, lcg]

    # 3. Simulation Loop
    for i, t in enumerate(time):
        # Define Open-Loop Control Inputs: [rpm1, rpm2, d_aileron, d_rudder, vbs, lcg]
        # Base commands: Forward thrust, neutral buoyancy (50%), neutral LCG (50%)
        u_cmd = [800.0, 800.0, 0.0, 0.0, 50.0, 50.0]
        
        # Introduce a step response (e.g., apply rudder and pitch after 10 seconds)
        if t >= 30.0:
            u_cmd[3] = 0.1   # Positive rudder deflection (yaw)
            u_cmd[2] = -0.05 # Slight aileron/elevator deflection (pitch)
            
        # Step the model
        nu, eta = sam.step(u_cmd)
        
        # Record states
        nu_history[i, :] = nu
        eta_history[i, :] = eta
        u_history[i, :] = u_cmd

    # 4. Plot Results
    plot_simulation_results(time, eta_history, nu_history, u_history)

def plot_simulation_results(time, eta, nu, u):
    """Generates a single comprehensive dashboard for the AUV simulation."""
    # Use a safe fallback style in case older seaborn styles are deprecated
    try:
        plt.style.use('seaborn-v0_8-paper')
    except:
        plt.style.use('bmh')
    
    # Create a single large figure
    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title("SAM AUV Open-Loop Simulation")
    
    # Define a 3x3 grid
    gs = GridSpec(3, 3, figure=fig, wspace=0.3, hspace=0.4)
    
    # --- 1. 3D Trajectory (Spans Rows 0 & 1, Col 0) ---
    ax1 = fig.add_subplot(gs[0:2, 0], projection='3d')
    ax1.plot(eta[:, 0], eta[:, 1], eta[:, 2], label='AUV Trajectory', color='b', linewidth=2)
    ax1.scatter(eta[0, 0], eta[0, 1], eta[0, 2], color='g', marker='o', s=50, label='Start')
    ax1.scatter(eta[-1, 0], eta[-1, 1], eta[-1, 2], color='r', marker='x', s=50, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (depth m)')
    ax1.invert_zaxis() # Z is positive downwards
    ax1.set_title('3D Open-Loop Trajectory', fontweight='bold')
    ax1.legend()

    # --- 2. Inertial Pose (Row 0, Cols 1 & 2) ---
    ax_pos = fig.add_subplot(gs[0, 1])
    ax_pos.plot(time, eta[:, 0], label='x')
    ax_pos.plot(time, eta[:, 1], label='y')
    ax_pos.plot(time, eta[:, 2], label='z')
    ax_pos.set_ylabel('Position (m)')
    ax_pos.set_title('Inertial Position')
    ax_pos.legend()

    ax_ang = fig.add_subplot(gs[0, 2], sharex=ax_pos)
    ax_ang.plot(time, np.degrees(eta[:, 3]), label='phi (roll)')
    ax_ang.plot(time, np.degrees(eta[:, 4]), label='theta (pitch)')
    ax_ang.plot(time, np.degrees(eta[:, 5]), label='psi (yaw)')
    ax_ang.set_ylabel('Angle (deg)')
    ax_ang.set_title('Inertial Orientation')
    ax_ang.legend()

    # --- 3. Body Velocities (Row 1, Cols 1 & 2) ---
    ax_lin = fig.add_subplot(gs[1, 1], sharex=ax_pos)
    ax_lin.plot(time, nu[:, 0], label='u (surge)')
    ax_lin.plot(time, nu[:, 1], label='v (sway)')
    ax_lin.plot(time, nu[:, 2], label='w (heave)')
    ax_lin.set_ylabel('Velocity (m/s)')
    ax_lin.set_title('Linear Velocities')
    ax_lin.legend()

    ax_rate = fig.add_subplot(gs[1, 2], sharex=ax_pos)
    ax_rate.plot(time, np.degrees(nu[:, 3]), label='p (roll rate)')
    ax_rate.plot(time, np.degrees(nu[:, 4]), label='q (pitch rate)')
    ax_rate.plot(time, np.degrees(nu[:, 5]), label='r (yaw rate)')
    ax_rate.set_ylabel('Rate (deg/s)')
    ax_rate.set_title('Angular Velocities')
    ax_rate.legend()

    # --- 4. Input Commands (Row 2, Cols 0, 1, & 2) ---
    ax_rpm = fig.add_subplot(gs[2, 0], sharex=ax_pos)
    ax_rpm.plot(time, u[:, 0], label='rpm1', drawstyle='steps-post')
    ax_rpm.plot(time, u[:, 1], label='rpm2', drawstyle='steps-post', linestyle='--')
    ax_rpm.set_ylabel('RPM')
    ax_rpm.set_xlabel('Time (s)')
    ax_rpm.set_title('Thruster Commands')
    ax_rpm.legend()

    ax_def = fig.add_subplot(gs[2, 1], sharex=ax_pos)
    ax_def.plot(time, u[:, 2], label='d_aileron', drawstyle='steps-post')
    ax_def.plot(time, u[:, 3], label='d_rudder', drawstyle='steps-post')
    ax_def.set_ylabel('Deflection (rad)')
    ax_def.set_xlabel('Time (s)')
    ax_def.set_title('Control Surfaces')
    ax_def.legend()

    ax_params = fig.add_subplot(gs[2, 2], sharex=ax_pos)
    ax_params.plot(time, u[:, 4], label='vbs', drawstyle='steps-post')
    ax_params.plot(time, u[:, 5], label='lcg', drawstyle='steps-post')
    ax_params.set_ylabel('VBS/LCG (%)')
    ax_params.set_xlabel('Time (s)')
    ax_params.set_title('Mass/CG Parameters')
    ax_params.legend()

    fig.suptitle('SAM AUV Open-Loop Simulation', fontsize=16, fontweight='bold', y=0.98)
    
    # Optional tight layout override to prevent title clipping with GridSpec
    # plt.tight_layout() is handled partially by GridSpec parameters above, 
    # but we can call a constrained layout if needed.
    plt.show()

if __name__ == "__main__":
    run_open_loop_simulation()