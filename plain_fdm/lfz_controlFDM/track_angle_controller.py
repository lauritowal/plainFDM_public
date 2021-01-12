import numpy as np
from dataclasses import dataclass
from typing import Optional
import logging
logging.basicConfig(filename='track_angle_controller.log',  filemode='w', level=logging.DEBUG)

from plain_fdm.physikFDM.aircraft_beaver import AircraftBaever



@dataclass
class TrackAngleController(object):
    MAX_TURN_RATE = 3.0
    GRAVITATIONAL_FORCE = 9.81

    # elevator
    KPA_ELEVATOR: float = 110
    KDA_ELEVATOR: float = 4.5

    # aileron
    KPA_AIERON: float = 100.0
    KDA_AILERON: float = 10

    # rudder
    KPA_RUDDER: float = np.deg2rad(20.0)

    def turn_coordinator(self, beta):
        return -beta * self.KPA_RUDDER

    def bank_angle_hold(self, roll_angle_reference, roll_angle, roll_angle_rate, delta_aileron) -> float:
        diff_rollAngle = roll_angle_reference - roll_angle

        diff_rollAngle = np.clip(np.deg2rad(-20), np.deg2rad(20), diff_rollAngle)

        AileronCommand = (diff_rollAngle * self.KPA_AIERON - roll_angle_rate * self.KDA_AILERON)
        AileronCommand = AileronCommand + delta_aileron
        AileronCommand = np.deg2rad(np.clip(AileronCommand, -1, 1) * (-15))

        logging.debug("aileron_command: %s", np.rad2deg(AileronCommand))

        return AileronCommand

    def heading_hold(self, heading_reference_deg: float, heading_current_deg: float, roll_angle_rate: float, airspeed:float, delta_aileron: float) -> float:
        difference_heading: float = heading_reference_deg - heading_current_deg

        normalized = self.normalize_angle(difference_heading)

        logging.debug("difference_heading: %f", difference_heading)

        turn_rate: float = normalized * 0.8 # 20 degree --> turn rate of 3 degree / sec
        if turn_rate > self.MAX_TURN_RATE:
            turn_rate = self.MAX_TURN_RATE
        if turn_rate < -self.MAX_TURN_RATE:
            turn_rate = -self.MAX_TURN_RATE

        logging.debug("turn_rate: %f", turn_rate)

        roll_angle_command: float = turn_rate * airspeed / self.GRAVITATIONAL_FORCE

        logging.debug("u: %f", airspeed)
        logging.debug("roll_angle_command: %f", roll_angle_command)

        return self.bank_angle_hold(np.deg2rad(roll_angle_command), np.deg2rad(heading_current_deg), roll_angle_rate, delta_aileron)

    '''
    keep values between -180 and 180 degrees
    input and output parameters are in degrees
    '''
    def normalize_angle(self, degrees: float) -> float:
        degrees = degrees % 360
        if degrees <= 180:
            return degrees
        else:
            return degrees - 360


if __name__ == '__main__':
    track_angle_controller = TrackAngleController()
