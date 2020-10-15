import numpy as np
from FDM.config import Config
from FDM.geometryFDM.datComGeometry_beaver import DatComGeometry_beaver
from FDM.servicesFDM.umrechnungen_koordinaten import UmrechnungenKoordinaten

class Aircraft_baever(object):

    def __init__(self):
        self.geometry = DatComGeometry_beaver()
        self.umrechnungenKoordinaten = UmrechnungenKoordinaten()

        # Ziel: ist die Flugbahngleichung
        # todo: prüfen ob der state in ein Dictonary soll
        # state/Zustand des Objects
        # np.array([u,v,w,x,y,z,p,q,r,theta,phi,psi])

        # 2. Gechwindigkeit
        # im flugzeugfesten System | f_ks
        self.u = 0  # m/s
        self.v = 0  # m/s
        self.w = 0  # m/s

        # im flugzeugfesten System | f_ks
        self.x = 0  # m
        self.y = 0  # m
        self.z = 0  # m

        # 4. Winkelgeschwindigkeiten p,q,r
        self.p = -0.00197 # rad/s
        self.q = 0.00331  # rad/s
        self.r = 0.0339 # rad/s

        self.phi = 0.0974 # rad
        self.theta = 0.0873  # rad
        self.psi = 0.142  # rad

        self.alpha = 0.0  # Deg
        self.alphaDot = 0.0
        self.beta = 0.0
        self.TAS = 42.7
        self.mass = self.geometry.mass
        self.inertia = self.geometry.inertia

        # Forces & moments
        self.coeff_aerodynamics = np.zeros(6)
        self.total_forces_f_ks = np.zeros(3)
        self.total_moments_f_ks = np.zeros(3)

        # todo: muss ins env
        self.rho = 0.989
        self.staudruck = 0.5 * self.rho * self.TAS ** 2
        self.g = 9.81
        self.Sw = 23.23

        # abgeleitete Koordinaten geodaetisch mit
        self.x_geo = 0
        self.y_geo = 0
        self.z_geo = 0
        self.x_dot_g_ks = 0
        self.y_dot_g_ks = 0
        self.z_dot_g_ks = 0

        self.delta_elevator = 0
        self.delta_aileron = 0
        self.delta_rudder = 0
        self.delta_thrust = 0

    def getSteuerflaechenUndMotorStellung(self):
        return [self.delta_elevator, self.delta_aileron, self.delta_rudder, self.delta_thrust]

    def getState(self):
        # ([u,v,w,x,y,z,p,q,r,theta,phi,psi])
        self.state = np.array(
            [self.u, self.v, self.w, self.x, self.y, self.z, self.p, self.q, self.r, self.phi, self.theta, self.psi])
        return self.state

    def setState(self, stateInput):
        # ACHTUNG: Wahrscheinlich falsch... Rückgabe ist phi, theta, psi
        self.u, self.v, self.w, self.x, self.y, self.z, self.p, self.q, self.r, self.phi, self.theta, self.psi = \
        stateInput[0], stateInput[1], stateInput[2], stateInput[3], \
        stateInput[4], stateInput[5], stateInput[6], stateInput[7], stateInput[8], stateInput[9], stateInput[10], \
        stateInput[11]
        # at new State: aktualisieren der Werte
        # todo: delta x,y,z in earthX, earthY, earthZ und lat, long, höhe umwandeln

        self.alpha = np.arctan2(self.w, self.u)
        self.beta = np.arctan2(self.v, (np.sqrt(np.power(self.u, 2) + np.power(self.w, 2))))
        self.TAS = np.sqrt(np.power(self.u, 2) + np.power(self.v, 2) + np.power(self.w, 2))  # Va = Vk - Vw
        # reynolds = self.rho * self.TAS * self.geometry.radius / self.mu  # Reynolds number
        self.staudruck = 0.5 * self.rho * np.power(self.TAS, 2)
        self.state = stateInput

    def getForces(self):
        # todo: splitten der Coeff bzgl des KS. aero liegen im a_ks, engine im f_ks
        coeff_aerodynamics = np.array([self._calculate_aero_alpha_beta_coeffs(), self._calculate_steuerfleachen_coeffs(), self._calculate_body_dumping_coeffs()]).sum(axis=0)
        coeff_engine = self._calculate_engine_coeffs()
        self.coeff_body = np.array([coeff_aerodynamics, coeff_engine]).sum(axis=0)  # xyz-body; lmn-body
        forces_body = self.coeff_body[0:3] * self.staudruck * self.Sw

        weight_force_g_ks = np.array([ 0, 0, self.g * self.geometry.mass])
        weight_force_f_ks = self.umrechnungenKoordinaten.geo2flug(weight_force_g_ks, self.phi, self.theta, self.psi)

        self.total_forces_f_ks = np.array([forces_body, weight_force_f_ks]).sum(axis=0)

        return self.total_forces_f_ks

    def getMoments(self):
        L_RollMoment = self.coeff_body[3] * self.staudruck * self.Sw * self.geometry.span
        M_PitchMoment = self.coeff_body[4] * self.staudruck * self.Sw * self.geometry.chord
        N_YawMoment = self.coeff_body[5] * self.staudruck * self.Sw * self.geometry.span

        self.total_moments_f_ks = np.array([L_RollMoment, M_PitchMoment, N_YawMoment])

        return self.total_moments_f_ks


    def _calculate_aero_alpha_beta_coeffs(self):
        # 1. alpha-, beta-coeff
        alpha = self.alpha
        alpha2 = alpha**2
        alpha3 = alpha**3
        beta = self.beta
        beta2 = beta**2
        beta3 = beta**3

        cx = alpha * self.geometry.cx_alpha + alpha2 * self.geometry.cx_alpha2 + alpha3 * self.geometry.cx_alpha3
        cy = beta * self.geometry.cy_beta
        cz = alpha * self.geometry.cz_alpha + alpha3 * self.geometry.cz_alpha3

        cl = beta * self.geometry.cl_beta
        cm = alpha * self.geometry.cm_alpha + alpha2 * self.geometry.cm_alpha2 + beta2 * self.geometry.cm_beta2
        cn = beta * self.geometry.cn_beta + beta3 * self.geometry.cn_beta3

        coeff_datum0 = [-0.0355400000000000, -0.00222600000000000, -0.0550400000000000, 0.000591000000000000, 0.0944800000000000, -0.00311700000000000]
        coeff_datum = np.add(coeff_datum0, [cx, cy, cz, cl, cm, cn])

        return coeff_datum

    def _calculate_steuerfleachen_coeffs(self):
        alpha = self.alpha
        beta = self.beta
        beta2 = beta ** 2
        deltaElevator = self.delta_elevator  # rad
        deltaAileron = self.delta_aileron  # rad
        deltaRudder = self.delta_rudder  # rad

        # Aileron:
        cx = deltaAileron * 0.0
        cy = deltaAileron * self.geometry.cy_da
        cz = deltaAileron * 0.0

        cl = deltaAileron * self.geometry.cl_da + (alpha * deltaAileron) * self.geometry.cl_da_alpha
        cm = deltaAileron * 0.0
        cn = deltaAileron * self.geometry.cn_da

        coeff_steuerflaechen_aileron = np.array([cx, cy, cz, cl, cm, cn])
        
        # Elevator
        cx = deltaElevator * 0.0
        cy = deltaElevator * 0.0
        cz = beta2 * deltaElevator * self.geometry.cz_de_beta + deltaElevator * self.geometry.cz_de

        cl = deltaElevator * 0.0
        cm = deltaElevator * self.geometry.cm_de
        cn = deltaElevator * 0.0

        coeff_steuerflaechen_elevator = np.array([cx, cy, cz, cl, cm, cn])

        # Rudder
        cx = deltaRudder * self.geometry.cx_dr
        cy = deltaRudder * self.geometry.cy_dr + alpha * deltaRudder * self.geometry.cy_dr_alpha
        cz = deltaRudder * 0

        cl = deltaRudder * self.geometry.cl_dr
        cm = deltaRudder * 0
        cn = deltaRudder * self.geometry.cn_dr

        coeff_steuerflaechen_rudder = np.array([cx, cy, cz, cl, cm, cn])

        # todo: Flaps tbd

        coeff_steuerflaechen = np.array([coeff_steuerflaechen_aileron, coeff_steuerflaechen_elevator, coeff_steuerflaechen_rudder]).sum(axis=0)

        return coeff_steuerflaechen

    def _calculate_body_dumping_coeffs(self):

        # p
        cx = 0
        cy = self.geometry.cy_p
        cz = 0

        cl = self.geometry.cl_p
        cm = 0
        cn = self.geometry.cn_p

        coeff_body_dumping_p = np.array([cx, cy, cz, cl, cm, cn]) * self.p * self.geometry.span/2

        # q
        cx = self.geometry.cx_q
        cy = 0
        cz = self.geometry.cz_q

        cl = 0
        cm = self.geometry.cm_q
        cn = self.geometry.cn_q

        coeff_body_dumping_q = np.array([cx, cy, cz, cl, cm, cn]) * self.q * self.geometry.chord

        # r
        cx = 0
        cy = self.geometry.cy_r
        cz = 0

        cl = self.geometry.cl_r
        cm = self.geometry.cm_r
        cn = self.geometry.cn_r

        coeff_body_dumping_r = np.array([cx, cy, cz, cl, cm, cn]) * self.r * self.geometry.span / 2

        coeff_body_dumping = (np.array([coeff_body_dumping_p, coeff_body_dumping_q, coeff_body_dumping_r]).sum(axis=0)) / self.TAS

        return coeff_body_dumping


    def _calculate_engine_coeffs(self):
        deltaThrottle = self.delta_thrust
        # 1 und 2 entspricht uprop in matlab DeHavilland
        # 1. deltaThrottle2RPM
        motordrehzahl = np.interp(deltaThrottle, self.geometry.stellbereichThrust, self.geometry.rpmMotor)  # rpm
        # 2. RPM to Manifold
        manifold = np.interp(motordrehzahl, self.geometry.rpmMotor, self.geometry.manifold)  # rpm
        dpt = ((2 / (self.TAS**3)) / self.rho) * (((((manifold + 7.4) * (motordrehzahl + 2010)) * 0.00412) + ((408.0 - 0.0965 * motordrehzahl) * (1.0 - self.rho/1.225) - 326.5)) * 0.7355) * 191.18 + 0.08696

        cx = dpt * self.geometry.cx_dpt + dpt*dpt * self.alpha * self.geometry.cx_dpt2_alpha * 1  # Headline: Faktor 3 durch RT
        cy = 0
        cz = dpt * self.geometry.cz_dpt

        cl = 0 #dpt * self.alpha**2 * self.geometry.cl_dpt_alpha2
        cm = 0 #dpt * self.geometry.cm_dpt
        cn = 0 #dpt**3 * self.geometry.cn_dpt3

        coeff_engine = np.array([cx, cy, cz, cl, cm, cn])

        return coeff_engine


def main():
    config = Config()
    aircraft = Aircraft_baever()
    aircraft.getForces()
    print(aircraft._calculate_engine_coeffs())

if __name__ == '__main__':
    main()




