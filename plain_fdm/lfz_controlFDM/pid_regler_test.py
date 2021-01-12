import unittest
import numpy as np

from plain_fdm.lfz_controlFDM.pid_regler import PidRegler
from plain_fdm.lfz_controlFDM.track_angle_controller import TrackAngleController
from plain_fdm.physikFDM.aircraft_beaver import AircraftBaever
from plain_fdm.physikFDM.dynamic_system6DoF import DynamicSystem6DoF

class PidReglerTest(unittest.TestCase):
    def test_get_aileron_command(self):
        aircraft = AircraftBaever()
        pid = PidRegler()
        dynamicSystem = DynamicSystem6DoF()
        stepweite = 0.01

        aircraft.setState(40., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., psi=np.deg2rad(20))

        for step in range(1000):
            aircraft.delta_aileron = pid.get_aileron_command(heading_reference=np.deg2rad(80),
                                                             heading=aircraft.psi,
                                                             roll_angle=aircraft.phi,
                                                             roll_angle_rate=aircraft.p,
                                                             delta_aileron=aircraft.delta_aileron)
            for _ in range(10):
                solver = dynamicSystem.integrate(
                    aircraft.getState(),
                    aircraft.getForces(),
                    aircraft.getMoments(),
                    aircraft.mass,
                    aircraft.inertia,
                    stepweite
                )
                aircraft.setState(*solver.y.flatten())

            print(step, "aileron:", np.rad2deg(aircraft.delta_aileron))

        self.assertEqual(True, False)

    '''def test_heading_hold(self):
        aircraft = AircraftBaever()
        track_angle_controller = TrackAngleController()
        dynamicSystem = DynamicSystem6DoF()
        stepweite = 0.01

        aircraft.setState(40., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., psi=0)

        for step in range(1200):
            aircraft.delta_aileron = track_angle_controller.heading_hold(
                heading_reference=0,
                aircraft=aircraft
            )
            for _ in range(10):
                solver = dynamicSystem.integrate(
                    aircraft.getState(),
                    aircraft.getForces(),
                    aircraft.getMoments(),
                    aircraft.mass,
                    aircraft.inertia,
                    stepweite
                )
                aircraft.setState(*solver.y.flatten())

            print(step, "aileron:", np.rad2deg(aircraft.delta_aileron))

        self.assertEqual(True, False)'''


if __name__ == '__main__':
    unittest.main()
