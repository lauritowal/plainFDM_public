import unittest
import numpy as np
from plain_fdm.lfz_controlFDM.track_angle_controller import TrackAngleController

class TrackAngleControllerTest(unittest.TestCase):
        def test_normalize_angle(self):
            track_angle_controller = TrackAngleController()
            normalized_angle = track_angle_controller.normalize_angle(-500)
            self.assertEqual(normalized_angle, -140)

        def test_normalize_angle_2(self):
            track_angle_controller = TrackAngleController()
            normalized_angle = track_angle_controller.normalize_angle(500)
            self.assertEqual(normalized_angle, 140)

        def test_normalize_angle_3(self):
            track_angle_controller = TrackAngleController()
            normalized_angle = track_angle_controller.normalize_angle(360)
            self.assertEqual(normalized_angle, 0)

        def test_normalize_angle_4(self):
            track_angle_controller = TrackAngleController()
            normalized_angle = track_angle_controller.normalize_angle(0)
            self.assertEqual(normalized_angle, 0)

        def test_normalize_angle_5(self):
            track_angle_controller = TrackAngleController()
            normalized_angle = track_angle_controller.normalize_angle(-10)
            self.assertEqual(normalized_angle, -10)


if __name__ == '__main__':
    unittest.main()

