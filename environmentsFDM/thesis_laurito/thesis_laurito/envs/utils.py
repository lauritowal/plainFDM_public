import numpy as np
import numpy.linalg as la


def angle(vector1, vector2):
    """ Returns the angle in radians between vectors 'vector1' and 'vector2', works also good for very small values  """
    cos_angle = np.dot(vector1, vector2)
    sin_angle = la.norm(np.cross(vector1, vector2))

    angle_in_radians = np.arctan2(sin_angle, cos_angle)

    return angle_in_radians
