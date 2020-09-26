import numpy as np


class UmrechnungenKoordinaten(object):

    def __init__(self):
        pass


    def aero2flug(self, wind_coords, alpha, beta):
        #ACHTUNG: quaternions benutzen sonst gibt es limits
        Lbw = np.array([[np.cos(alpha) * np.cos(beta), - np.cos(alpha) * np.sin(beta), - np.sin(alpha)],
            [np.sin(beta), np.cos(beta), 0],
            [np.sin(alpha) * np.cos(beta), - np.sin(alpha) * np.sin(beta), np.cos(alpha)]
        ])
        body_coords = Lbw.dot(wind_coords)
        return body_coords


    def geo2flug(self, geo_coords, phi=0, theta=0, psi=0):
        Lbh = np.array([
            [np.cos(theta) * np.cos(psi), np.cos(theta) * np.sin(psi), - np.sin(theta)],
            [np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi), np.sin(phi) * np.sin(theta) * np.sin(psi) + np.cos(phi) * np.cos(psi), np.sin(phi) * np.cos(theta)],
            [np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi), np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi), np.cos(phi) * np.cos(theta)]
        ])
        flug_coords = Lbh.dot(geo_coords)
        return flug_coords



    def flug2geo(self, flug_coords, phi=0, theta=0, psi=0):
        # Transformation matrix from body to local horizon
        Lhb = np.array([
            [np.cos(theta) * np.cos(psi),
             np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi),
             np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi)],
            [np.cos(theta) * np.sin(psi),
             np.sin(phi) * np.sin(theta) * np.sin(psi) + np.cos(phi) * np.cos(psi),
             np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi)],
            [- np.sin(theta),
             np.sin(phi) * np.cos(theta),
             np.cos(phi) * np.cos(theta)]
        ])

        geo_coords = Lhb.dot(flug_coords)

        return geo_coords

    #siehe auch pyfme: https://github.com/AeroPython/PyFME/blob/master/src/pyfme/utils/change_euler_quaternion.py
    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Convert degrees to quaternions
        """
        t0 = np.cos(np.radians(yaw * 0.5))
        t1 = np.sin(np.radians(yaw * 0.5))
        t2 = np.cos(np.radians(roll * 0.5))
        t3 = np.sin(np.radians(roll * 0.5))
        t4 = np.cos(np.radians(pitch * 0.5))
        t5 = np.sin(np.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]


def main():
    service = Services()
    gewichtsvektor_g_ks = [0, 0, 9.81]
    gewichtsvektor_f_ks = service.geo2flug(gewichtsvektor_g_ks, np.pi/2.0, 0, 0)
    print(gewichtsvektor_f_ks)

if __name__ == '__main__':
    main()