
class Utils(object):
    '''
    keep values between -180 and 180 degrees
    input and output parameters are in degrees
    '''
    def normalize_angle(self, degrees: float) -> float:
        degrees = degrees % 360
        if degrees > 180:
            return degrees - 360
        return degrees
