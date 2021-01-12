import numpy as np
import logging

from plain_fdm.servicesFDM.utils import Utils

logging.basicConfig(filename='pid_regler.log',  filemode='w', level=logging.DEBUG)


# siehe auch Drohne-Code Unity
class PidRegler(object):

    def __init__(self):
        self.utils = Utils()
        self.kpaElevator = 110
        self.kdaElevator = 4.5
        self.kpaAileron = 100.0
        self.kdaAileron = 10

        
    # Headline: Aileron -->
    def get_aileron_command(self, heading_reference, heading, roll_angle, roll_angle_rate, delta_aileron):
        roll_angle_reference = self._outerLoopAileron(heading_reference, heading)
        return self._innerLoopAileron(roll_angle_reference, roll_angle, roll_angle_rate, delta_aileron)

    def _outerLoopAileron(self, heading_reference, heading):

        logging.debug("heading_reference: %s", heading_reference)
        logging.debug("heading: %s", heading)

        heading_difference = np.deg2rad(self.utils.normalize_angle(heading_reference - heading))
        logging.debug("heading_difference: %s (rad)", heading_difference)

        logging.debug("heading_reference: %s (degree)", np.rad2deg(heading_difference))

        heading_roll_angle_reference = heading_difference * 1.0  #Vorsteuerung als P-Regler
        return heading_roll_angle_reference

    # innerLoop: heading_roll->Aileron
    def _innerLoopAileron(self, roll_angle_reference, roll_angle, roll_angle_rate, delta_aileron):

        logging.debug("roll_angle_reference: %s", roll_angle_reference)
        logging.debug("roll_angle: %s", roll_angle)

        diff_rollAngle = roll_angle_reference - roll_angle

        logging.debug("diff_rollAngle: %s", diff_rollAngle)

        #if np.rad2deg(rollAngle_Current) < -2:
        #    print("jetzt")
        AileronCommand = (diff_rollAngle * self.kpaAileron - roll_angle_rate * self.kdaAileron)
        AileronCommand = AileronCommand + delta_aileron
        AileronCommand = np.deg2rad(np.clip(AileronCommand, -1, 1) * (-15))

        logging.debug("AileronCommand: %s (in degrees)", AileronCommand)

        return AileronCommand
    
    # Headline: Elevator
    def getElevatorCommand(self, TASReference, TASCurrent, pitchAngleCurrent, pitchAngleRateCurrent, elevatorCurrent):
        pitchAngleReference = self._outerLoopElevator(TASReference, TASCurrent)
        elevatorCommand = self._innerLoopElevator(pitchAngleReference, pitchAngleCurrent, pitchAngleRateCurrent, elevatorCurrent)
        return elevatorCommand

    def _outerLoopElevator(self, TASReference, TASCurrent):
        pitchAngleReference = (TASCurrent - TASReference) * 1.0  #Vorsteuerung als P-Regler
        return pitchAngleReference

    # innerLoop: Pitch->Elevator
    def _innerLoopElevator(self, pitchAngleReference, pitchAngleCurrent, pitchAngleRateCurrent, elevatorCurrent):
        diffPitchAngle = pitchAngleReference - pitchAngleCurrent
        elevatorCommand = np.clip(diffPitchAngle * self.kpaElevator - pitchAngleRateCurrent * self.kdaElevator, -1, 1)
        elevatorCommand = elevatorCommand + elevatorCurrent
        elevatorCommand = np.deg2rad(np.clip(elevatorCommand, -1, 1) * (-20))
        return elevatorCommand

    def __difference_yaw_angle(self, heading_reference, heading_current):
        # keep values between 0 and 360 degrees
        heading_reference = heading_reference % 360
        heading_current = heading_current % 360

        logging.debug("heading_reference mod 360: %s", heading_reference)
        logging.debug("heading_current mod 360: %s", heading_current)

        heading_difference = heading_reference - heading_current

        logging.debug("heading_difference: %s", heading_difference)

        normalized = self.utils.normalize_angle(heading_difference)

        logging.debug("normalized: %s", normalized)

        return normalized


def main():
    pid = PidRegler()
    print(pid.getElevatorCommand(70,60,0,0,0))

if __name__ == '__main__':
    main()