import numpy as np

class SAMAuvDynamicModel:
    def __init__(self, dt=0.1):
        """
        Initialize the SAM AUV 6-DOF Fossen Model.
        :param dt: Integration time step (seconds)
        """
        self.dt = dt
        self.g = 9.81  # Acceleration due to gravity (m/s^2)
        
        # Static Vehicle Parameters
        self.m_base = 12.012 + 0.3  # Base mass + Piston (kg)
        self.max_vbs_mass = 0.249   # Max water intake (kg)
        self.xg_min = -0.057         # Min LCG offset (m)
        self.xg_max = 0          # Max LCG offset (m)
        self.B = (self.m_base + (self.max_vbs_mass / 2.0)) * self.g # Assumed constant Buoyancy force (N)
        self.L_t = 0.73             # Thruster distance from CG (m)
        
        # Rigid Body Inertias (kg*m^2)
        self.Ix = 0.0293
        self.Iy = 1.6202
        self.Iz = 1.6202
        
        # State Vectors
        self.nu = np.zeros(6)  # Body velocities [u, v, w, p, q, r]
        self.eta = np.zeros(6) # Inertial pose [x, y, z, phi, theta, psi]
        
    def _get_varying_params(self, vbs, lcg):
        """Calculates mass and longitudinal center of gravity from inputs."""
        m = self.m_base + (self.max_vbs_mass * (vbs / 100.0))
        xg = np.interp(lcg, [0, 100], [self.xg_min, self.xg_max])
        return m, xg

    def get_matrices(self, nu, eta, u_cmd):
        """Builds the Fossen system matrices."""
        rpm1, rpm2, d_pitch, d_yaw, vbs, lcg = u_cmd
        u, v, w, p, q, r = nu
        phi, theta, psi = eta[3], eta[4], eta[5]
        
        m, xg = self._get_varying_params(vbs, lcg)
        
        # 1. Mass Matrix M(u)
        M = np.diag([m, m, m, self.Ix, self.Iz, self.Iy])
        M[1, 5] = m * xg   # Y-r coupling
        M[2, 4] = -m * xg  # Z-q coupling
        M[4, 2] = -m * xg  # M-w coupling
        M[5, 1] = m * xg   # N-v coupling

        # 2. Coriolis Matrix C_RB(nu, u)
        C = np.zeros((6, 6))
        C[0, 4] = m * (w - xg * q)
        C[0, 5] = -m * (v + xg * r)
        C[1, 3] = -m * w
        C[1, 4] = m * xg * p
        C[1, 5] = m * u
        C[2, 3] = m * v
        C[2, 4] = -m * u
        C[2, 5] = m * xg * p
        C[3, 1] = m * w
        C[3, 2] = -m * v
        C[3, 4] = -self.Iy * r
        C[3, 5] = -self.Iz * q
        C[4, 0] = -m * (w - xg * q)
        C[4, 1] = -m * xg * p
        C[4, 2] = m * u
        C[4, 3] = self.Iy * r
        C[4, 5] = self.Ix * p
        C[5, 0] = m * (v + xg * r)
        C[5, 1] = -m * u
        C[5, 2] = -m * xg * p
        C[5, 3] = self.Iz * q
        C[5, 4] = -self.Ix * p

        # 3. Damping Matrix D(nu)
        D = np.diag([5*abs(u), 20*abs(v), 50*abs(w), 1*abs(p), 20*abs(q), 20*abs(r)])
        D[4, 2] = 5 * abs(w)  # Pitch-Heave coupling from cp_x = 0.1
        D[5, 1] = -2 * abs(v) # Yaw-Sway coupling from cp_x = 0.1
        
        # 4. Restoring Forces g(eta, u)
        W = m * 9.81
        g = np.array([
            (W - self.B) * np.sin(theta),
            -(W - self.B) * np.cos(theta) * np.sin(phi),
            -(W - self.B) * np.cos(theta) * np.cos(phi),
            0,
            xg * W * np.cos(theta) * np.cos(phi),
            -xg * W * np.cos(theta) * np.sin(phi)
        ])
        
        # 5. Actuation tau(u)
        def thrust(rpm):
            return (rpm * 0.005) if rpm >= 0 else (rpm * 0.005 * 0.6)
            
        F_T1, F_T2 = thrust(rpm1), thrust(rpm2)
        F_T = F_T1 + F_T2
        
        tau = np.array([
            F_T * np.cos(d_pitch) * np.cos(d_yaw),
            F_T * np.cos(d_pitch) * np.sin(d_yaw),
            -F_T * np.sin(d_pitch),
            (F_T1 - F_T2) * 8.004e-4,
            F_T * np.sin(d_pitch) * self.L_t,
            F_T * np.cos(d_pitch) * np.sin(d_yaw) * self.L_t
        ])

        return M, C, D, g, tau

    def kinematic_transform(self, eta):
        """Maps Body frame velocities to Inertial frame (J matrix)."""
        phi, theta, psi = eta[3], eta[4], eta[5]
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)
        
        J = np.zeros((6, 6))
        # Linear velocity mapping
        J[0:3, 0:3] = [
            [cth*cpsi, -cphi*spsi + sphi*sth*cpsi,  sphi*spsi + cphi*sth*cpsi],
            [cth*spsi,  cphi*cpsi + sphi*sth*spsi, -sphi*cpsi + cphi*sth*spsi],
            [-sth,      sphi*cth,                   cphi*cth]
        ]
        # Angular velocity mapping
        if np.abs(cth) > 1e-6: # Avoid singularity at pitch = +/- 90 degrees
            J[3:6, 3:6] = [
                [1, sphi * np.tan(sth), cphi * np.tan(sth)],
                [0, cphi,              -sphi],
                [0, sphi / cth,         cphi / cth]
            ]
        return J

    def step(self, u_cmd):
        """
        Integrates the model forward by one time step (Euler method).
        :param u_cmd: [rpm1, rpm2, d_pitch, d_yaw, vbs_pct, lcg_pct]
        :return: updated (nu, eta)
        """
        # Clamp inputs
        u_cmd[2] = np.clip(u_cmd[2], -0.2, 0.2)
        u_cmd[3] = np.clip(u_cmd[3], -0.2, 0.2)
        u_cmd[4] = np.clip(u_cmd[4], 0, 100)
        u_cmd[5] = np.clip(u_cmd[5], 0, 100)

        M, C, D, g, tau = self.get_matrices(self.nu, self.eta, u_cmd)
        
        # nu_dot = M^-1 * (tau - C*nu - D*nu - g)
        M_inv = np.linalg.inv(M)
        nu_dot = M_inv.dot(tau - C.dot(self.nu) - D.dot(self.nu) - g)
        
        # Kinematics
        J = self.kinematic_transform(self.eta)
        eta_dot = J.dot(self.nu)
        
        # Integration
        self.nu += nu_dot * self.dt
        self.eta += eta_dot * self.dt
        
        return self.nu, self.eta