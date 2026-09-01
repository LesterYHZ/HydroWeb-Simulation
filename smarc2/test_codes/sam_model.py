import numpy as np
from gnc import m2c

class SAM:
    def __init__(self, dt=0.1, nu_init = np.zeros(6), eta_init = np.zeros(6)):
        """
        Initialize the SAM AUV 6-DOF Fossen Model.
        :param dt: Integration time step (seconds)
        """
        self.dt = dt
        self.g = 9.81  # Acceleration due to gravity (m/s^2)
        
        # Static Vehicle Parameters
        self.m_base = 12.012 + 0.3  # Base mass + Piston (kg)
        self.max_vbs_mass = 0.249   # Max water intake (kg)
        self.xg_min = -0.057/2.0         # Min LCG offset (m)
        self.xg_max = 0.057/2.0          # Max LCG offset (m)
        self.B = (self.m_base + (self.max_vbs_mass * 0.5)) * self.g # Assumed constant Buoyancy force (N)
        
        # Rigid Body Inertias (kg*m^2)
        self.Ix = 0.0293
        self.Iy = 1.6202
        self.Iz = 1.6202 
        # State Vectors
        self.nu = nu_init  # Body velocities [u, v, w, p, q, r]
        self.eta = eta_init # Inertial pose [x, y, z, phi, theta, psi]
        
    def _get_varying_params(self, vbs, lcg):
        """Calculates mass and longitudinal center of gravity from inputs."""
        m = self.m_base + (self.max_vbs_mass * (vbs / 100.0))
        xg = np.interp(lcg, [0, 100], [self.xg_min, self.xg_max])
        xg = np.round(xg, decimals=2)
        return m, xg

    def get_matrices(self, nu, eta, u_cmd):
        """Builds the Fossen system matrices."""
        rpm1, rpm2, d_aileron, d_rudder, vbs, lcg = u_cmd
        u, v, w, p, q, r = nu
        phi, theta, psi = eta[3], eta[4], eta[5]
        
        m, xg = self._get_varying_params(vbs, lcg)
        
        # 1. Mass Matrix M(u)
        M = np.diag([m, m, m, self.Ix, self.Iy, self.Iz])
        M[1, 5] = m * xg   # Y-r coupling
        M[2, 4] = -m * xg  # Z-q coupling
        M[4, 2] = -m * xg  # M-w coupling
        M[5, 1] = m * xg   # N-v coupling

        # 2. Coriolis Matrix C_RB(nu, u)
        C = m2c(M, nu)

        # 3. Damping Matrix D(nu)
        D = np.diag([1*abs(u), 1*abs(v), 1*abs(w), 1*abs(p), 1*abs(q), 1*abs(r)])
        #D[4, 2] = 5 * abs(w)  # Pitch-Heave coupling from cp_x = 0.1
        #D[5, 1] = -2 * abs(v) # Yaw-Sway coupling from cp_x = 0.1
        
        # 4. Restoring Forces g(eta, u)
        W = m * self.g
        G = np.array([
            (W - self.B) * np.sin(theta),
            -(W - self.B) * np.cos(theta) * np.sin(phi),
            -(W - self.B) * np.cos(theta) * np.cos(phi),
            0,
            xg * W * np.cos(theta) * np.cos(phi),
            -xg * W * np.cos(theta) * np.sin(phi)
        ])
        
        # 5. Actuation tau(u)
        F_T = 0.0175 * (rpm1 + rpm2)
        M_T = 0.01 * (rpm1 + rpm2)
        M_Tx = 0.01 * (rpm1 - rpm2)

        d_e = d_aileron * 0.1
        d_r = d_rudder * 0.1
        
        tau = np.array([
            F_T * np.cos(d_e) * np.cos(d_r),
            F_T * np.cos(d_r) * np.sin(d_e),
            F_T * np.sin(d_r),
            M_Tx * np.cos(d_e) * np.cos(d_r),
            M_T * np.sin(d_r),
            M_T * np.cos(d_r) * np.sin(d_e)
        ])

        return M, C, D, G, tau

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
        else:
            J[3:6, 3:6] = np.eye(3)
        return J

    def step(self, u_cmd):
        """
        Integrates the model forward by one time step (Euler method).
        :param u_cmd: [rpm1, rpm2, d_pitch, d_yaw, vbs_pct, lcg_pct]
        :return: updated (nu, eta)
        """
        # Clamp inputs
        u_cmd[0] = np.clip(u_cmd[0], -1000, 1000)
        u_cmd[1] = np.clip(u_cmd[1], -1000, 1000)
        u_cmd[2] = np.clip(u_cmd[2], -0.2, 0.2)
        u_cmd[3] = np.clip(u_cmd[3], -0.2, 0.2)
        u_cmd[4] = np.clip(u_cmd[4], 0, 100)
        u_cmd[5] = np.clip(u_cmd[5], 0, 100)

        M, C, D, G, tau = self.get_matrices(self.nu, self.eta, u_cmd)
        
        # nu_dot = M^-1 * (tau - C*nu - D*nu - g)
        M_inv = np.linalg.inv(M)
        nu_dot = M_inv.dot(tau - C.dot(self.nu) - D.dot(self.nu) - G)
        
        # Kinematics
        J = self.kinematic_transform(self.eta)
        eta_dot = J.dot(self.nu)
        
        # Integration
        self.nu += nu_dot * self.dt
        self.eta += eta_dot * self.dt
        
        return self.nu, self.eta

