import numpy as np

from scipy.spatial.transform import Rotation as R

p = R.from_euler('z', 90, degrees=True)


r = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])

gravity = np.array([0, 0, 9.81])

print(gravity * p.as_euler('xyz', degrees=True))