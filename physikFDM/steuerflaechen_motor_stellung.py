import numpy as np

class SteuerflaechenUndMotorStellung(object):

    def __init__(self):
        # Stellungen in radient
        self.deltaElevator = 0.0
        self.deltaAileron = 0
        self.deltaRudder = 0
        self.deltaThrust = 0.0

        self.controlLimits = {'deltaElevatorBorder': (np.deg2rad(-26), np.deg2rad(28)),  # rad
                              'deltaAileronBorder': (np.deg2rad(-15), np.deg2rad(20)),  # rad
                              'deltaRudderBorder': (np.deg2rad(-16), np.deg2rad(16)),  # rad
                              'deltaThrustBorder': (0, 1)}  # non-dimensional

    def checkMaximumMinimum(self):
        pass

    def setSteuerflaechenUndMotorStellung(self):
        pass

    def getSteuerflaechenUndMotorStellung(self):
        return [self.deltaElevator, self.deltaAileron, self.deltaRudder, self.deltaThrust]


    '''trimmed Control level-0
        {
        'delta_elevator': -0.048951124635247797,
        'delta_aileron': 0.0,
        'delta_rudder': 0.0,
        'delta_t': 0.57799667845248515}'''