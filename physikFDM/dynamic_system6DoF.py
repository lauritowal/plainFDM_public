import numpy as np
from scipy.integrate import solve_ivp, ode

#Euler Flat Earth
class DynamicSystem6DoF(object):

    def __init__(self):
        pass

    # integrationsschritt
    def integrate(self, state, forces, moments, mass, inertia, stepweite):
        return (solve_ivp(fun=lambda t, y: self._system_equations(t, y, state, mass, inertia, forces, moments), t_span=[0, stepweite], y0=state,
                          t_eval=[stepweite]))

    def _system_equations(self, t, y, state, mass, inertia, forces, moments):
        mass = mass
        Ix = inertia[0, 0]
        Iy = inertia[1, 1]
        Iz = inertia[2, 2]
        Jxz = - inertia[0, 2]

        Fx, Fy, Fz = forces[0], forces[1], forces[2]
        L, M, N = moments[0], moments[1], moments[2]

        u, v, w = state[0], state[1], state[2]
        p, q, r = state[6], state[7], state[8]
        phi, theta, psi = state[9], state[10], state[11]

        # translation forces equations
        # Allerton S. 145
        # ACHTUNG: Brockhaus hat andere Vorzeichen S. 72
        du_dt = Fx / mass + r * v - q * w
        dv_dt = Fy / mass - r * u + p * w
        dw_dt = Fz / mass + q * u - p * v

        # Angular momentum equations
        # ACHTUNG: Vielleicht falsch siehe Allerton S. 147
        dp_dt = (L * Iz + N * Jxz - q * r * (Iz ** 2 - Iz * Iy + Jxz ** 2) +
                 p * q * Jxz * (Ix + Iz - Iy)) / (Ix * Iz - Jxz ** 2)
        dq_dt = (M + (Iz - Ix) * p * r - Jxz * (p ** 2 - r ** 2)) / Iy
        dr_dt = (L * Jxz + N * Ix + p * q * (Ix ** 2 - Ix * Iy + Jxz ** 2) -
                 q * r * Jxz * (Iz + Ix - Iy)) / (Ix * Iz - Jxz ** 2)

        # translation kinematic equations
        # siehe: S. 16 - https://www.nasa.gov/centers/dryden/pdf/88104main_H-1391.pdf

        dx_dt = (np.cos(theta) * np.cos(psi) * u +
                 (np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi)) * v +
                 (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi)) * w)
        dy_dt = (np.cos(theta) * np.sin(psi) * u +
                 (np.sin(phi) * np.sin(theta) * np.sin(psi) + np.cos(phi) * np.cos(psi)) * v +
                 (np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi)) * w)
        dz_dt = -u * np.sin(theta) + v * np.sin(phi) * np.cos(theta) + w * np.cos(
            phi) * np.cos(theta)

        # Angular Kinematic equations
        # Siehe Brockhaus S. 75
        dtheta_dt = q * np.cos(phi) - r * np.sin(phi)
        dphi_dt = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        dpsi_dt = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)

        # ACHTUNG: Vielleicht falsch PHI, THETA, PSI
        return np.array([du_dt, dv_dt, dw_dt, dx_dt, dy_dt, dz_dt, dp_dt, dq_dt, dr_dt, dphi_dt, dtheta_dt,
                         dpsi_dt])


    




