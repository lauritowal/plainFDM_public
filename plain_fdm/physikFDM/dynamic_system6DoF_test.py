import unittest
import numpy as np
from plain_fdm.physikFDM.aircraft_beaver import AircraftBaever
from plain_fdm.physikFDM.dynamic_system6DoF import DynamicSystem6DoF

class DynamicSystem6DoFTest(unittest.TestCase):
    def test_something(self):
        aircraft = AircraftBaever()
        dynamic_system = DynamicSystem6DoF()
        step_length = 0.02

        solver = dynamic_system.integrate(
            state=aircraft.getState(),
            forces=aircraft.getForces(),
            moments=aircraft.getMoments(),
            mass=aircraft.mass,
            inertia=aircraft.inertia,
            stepweite=step_length
        )

        ## TODO: Check if integrated values are correct
        self.u, self.v, self.w, self.x, self.y, self.z, self.p, self.q, self.r, self.phi, self.theta, self.psi = solver.y.flatten()

        self.assertEqual(True, False)


if __name__ == '__main__':
    unittest.main()
