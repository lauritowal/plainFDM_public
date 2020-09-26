import numpy as np
import sys
sys.path.append('../..')
sys.path.append('.')

from scipy.interpolate import RectBivariateSpline

from FDM.config import Config
from FDM.servicesFDM.umrechnungen_koordinaten import UmrechnungenKoordinaten
from FDM.physikFDM.steuerflaechen_motor_stellung import SteuerflaechenUndMotorStellung


class Aircraft(object):
    def __init__(self, DatComClass):

        self.config = Config()
        geometrieClass = self.config.mappingDict[DatComClass] #welche Geometrie/DatCom Klasse wird benutzt
        self.geometry = geometrieClass()
        self.umrechnungenKoordinaten = UmrechnungenKoordinaten()  #todo: nicht als self, sondern als extra Dienst
        self.SteuerflaechenUndMotorStellung = SteuerflaechenUndMotorStellung()

        # Ziel: ist die Flugbahngleichung
        #todo: prüfen ob der state in ein Dictonary soll
        #state/Zustand des Objects
        #np.array([u,v,w,x,y,z,p,q,r,theta,phi,psi])
        # 1. Position
        # im flugzeugfesten System | f_ks
        self.x = 0  # m
        self.y = 0  # m
        self.z = 0  # m

        # 2. Gechwindigkeit
        # im flugzeugfesten System | f_ks
        self.u = 0  # m/s
        self.v = 0  # m/s
        self.w = 0  # m/s

        # 3. Attitude | Winkel um die Drehachsen f-KS zu g-KS
        self.phi = 0  # rad
        self.theta = 0  # rad
        self.psi = 0  # rad

        # 4. Winkelgeschwindigkeiten p,q,r
        self.p = 0  # rad/s
        self.q = 0  # rad/s
        self.r = 0  # rad/s

        # der gesamte State
        self.state = np.array([self.u, self.v, self.w, self.x, self.y, self.z, self.p, self.q, self.r, self.phi, self.theta, self.psi])

        # abgeleitete Werte
        self.alpha = 0.1e-10 #Deg
        self.alphaDot = 0.1e-10
        self.beta = 0
        self.mass = self.geometry.mass
        self.inertia = self.geometry.inertia
        self.TAS = 0.1e-10
        self.staudruck = 0
        self.Sw = 0

        # abgeleitete Koordinaten geodaetisch mit
        self.x_geo = 0
        self.y_geo = 0
        self.z_geo = 0
        self.x_dot_g_ks = 0
        self.y_dot_g_ks = 0
        self.z_dot_g_ks = 0


        # EARTH CONSTANTS
        # todo: hat nichts mit dem ac zu tun und muss ins Enviroment
        self.gravity = 9.80665  # Gravity of Ethe Earth (m/s^2) - [1]
        self.rho = 1.225
        self.mu = 1.983e-5  # kg/m/s

        # Forces & moments
        self.total_forces_f_ks = np.zeros(3)
        self.total_moments_f_ks = np.zeros(3)


    def getState(self):
        #([u,v,w,x,y,z,p,q,r,theta,phi,psi])
        self.state = np.array([self.u, self.v, self.w, self.x, self.y, self.z, self.p, self.q, self.r, self.phi, self.theta, self.psi])
        return self.state

    def setState(self, stateInput):
        # ACHTUNG: Wahrscheinlich falsch... Rückgabe ist phi, theta, psi
        self.u, self.v, self.w, self.x, self.y, self.z, self.p, self.q, self.r, self.phi, self.theta, self.psi = stateInput[0], stateInput[1], stateInput[2], stateInput[3], \
                                                         stateInput[4], stateInput[5], stateInput[6], stateInput[7], stateInput[8], stateInput[9], stateInput[10], stateInput[11]
        # at new State: aktualisieren der Werte
        #todo: delta x,y,z in earthX, earthY, earthZ und lat, long, höhe umwandeln

        self.alpha = np.arctan2(self.w, self.u)
        self.beta = np.arctan2(self.v, (np.sqrt(np.power(self.u, 2) + np.power(self.w, 2))))
        self.TAS = np.sqrt(np.power(self.u, 2) + np.power(self.v, 2) + np.power(self.w, 2))  # Va = Vk - Vw
        # reynolds = self.rho * self.TAS * self.geometry.radius / self.mu  # Reynolds number
        self.staudruck = 0.5 * self.rho * np.power(self.TAS, 2)
        self.state = stateInput

    def getForces(self):
        self._calculate_aerodynamic_longitudinal_coeffs()
        self._calculate_aerodynamic_lateral_coeffs()
        self._calculate_engine_coeffs()
        self.total_forces_f_ks = self._calculate_aerodynamic_forces() + self._calculate_engine_forces()
        return self.total_forces_f_ks

    def getMoments(self):
        self._calculate_aerodynamic_longitudinal_coeffs()
        self._calculate_aerodynamic_lateral_coeffs()
        self.total_moments_f_ks = self._calculate_aerodynamic_moments()
        return self.total_moments_f_ks

    '''
    3. Compute the coefficients of aerodynamic forces CL,CLt,CD,Cyβ,Cyδr
    4. Compute the coefficients of aerodynamic moments in pitch Cm0, Cα, Cδe, Cmq, Cmα ̇
    5. Compute the coefficients of aerodynamic moments in roll Clβ , Clδr, Clδa, Clp, Clr
    6. Compute the coefficients of aerodynamic moments in yaw Cnβ,Cnβ ̇,Cnδr,Cnδa,Cnp,Cnr
    
    δa is the aileron deflection 
    δe is the elevator deflection 
    δr is the rudder deflection
    
    Allerton S. 111 ff
    '''

    #Längs- und Querrichtung vereinfachend entkoppelt. S. 142 ff Flugregelung
    # berechnet: Ca, Cw, CM
    def _calculate_aerodynamic_longitudinal_coeffs(self):
        self.Sw = self.geometry.Sw
        #Preparation:
        alphaDegree = np.rad2deg(self.alpha)
        deltaElevator = np.rad2deg(self.SteuerflaechenUndMotorStellung.deltaElevator)  # deg

        Coeff_Lift_alpha = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_Lift_data)
        Coeff_Lift_alphaDot = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_Lift_alphadot_data)
        Coeff_Lift_q = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_Lift_q_data)
        Coeff_Lift_deltaElevator = np.interp(deltaElevator, self.geometry.delta_elev_data, self.geometry.Coeff_Lift_delta_elev_data)

        Coeff_Drag_alpha = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_Drag_data)
        Coeff_Drag_deltaElevator = RectBivariateSpline(self.geometry.delta_elev_data, self.geometry.alpha_data, self.geometry.Coeff_Drag_delta_elev_data)
        Coeff_Drag_deltaElevator = Coeff_Drag_deltaElevator(deltaElevator, alphaDegree)[0, 0]

        Coeff_PitchMoment_alpha = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_PitchMoment_data)
        Coeff_PitchMoment_deltaElevator = np.interp(deltaElevator, self.geometry.delta_elev_data, self.geometry.Coeff_Lift_delta_elev_data)
        Coeff_PitchMoment_alphaDot = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_PitchMoment_alphadot_data)
        Coeff_PitchMoment_q = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_PitchMoment_q_data)
        self.Coeff_Lift = (
                Coeff_Lift_alpha +
                Coeff_Lift_deltaElevator +
                self.geometry.chord / (2 * self.TAS) * (Coeff_Lift_q * self.q + Coeff_Lift_alphaDot * self.alphaDot)
        )
        self.Coeff_Drag = Coeff_Drag_alpha + Coeff_Drag_deltaElevator
        self.Coeff_PitchMoment = (
                Coeff_PitchMoment_alpha +
                Coeff_PitchMoment_deltaElevator +
                self.geometry.chord / (2 * self.TAS) * (2 * Coeff_PitchMoment_q * self.q + Coeff_PitchMoment_alphaDot * self.alphaDot)
        )

    def _calculate_aerodynamic_lateral_coeffs(self):
        #berechnet: Cq, CL, CN -> Querkraft und Momente um die x-, z-Achse
        alphaDegree = np.rad2deg(self.alpha) #deg
        deltaAileronDegree = np.rad2deg(self.SteuerflaechenUndMotorStellung.deltaAileron)  # deg
        deltaRudderRadient = self.SteuerflaechenUndMotorStellung.deltaRudder # rad
        #todo: das war vorher cq... ist das falsch und wo wird das verwendet
        Coeff_SideForce_beta = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_SideForce_beta_data)
        Coeff_SideForce_p = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_SideForce_p_data)
        Coeff_SideForce_r = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_SideForce_r_data)
        Coeff_SideForce_deltaRudder = 0

        Coeff_RollMoment_beta = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_RollMoment_beta_data)
        Coeff_RollMoment_p = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_RollMoment_p_data)
        Coeff_RollMoment_r = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_RollMoment_r_data)
        Coeff_RollMoment_deltaRudder = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_RollMoment_delta_rud_data)
        Coeff_RollMoment_deltaAileron = np.interp(deltaAileronDegree, self.geometry.delta_aile_data, self.geometry.Coeff_RollMoment_delta_aile_data)

        Coeff_YawMoment_beta = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_YawMoment_beta_data)
        Coeff_YawMoment_p = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_YawMoment_p_data)
        Coeff_YawMoment_r = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_YawMoment_r_data)
        Coeff_YawMoment_deltaRudder = np.interp(alphaDegree, self.geometry.alpha_data, self.geometry.Coeff_YawMoment_delta_rud_data)
        Coeff_YawMoment_deltaAileron = RectBivariateSpline(self.geometry.delta_aile_data, self.geometry.alpha_data, self.geometry.Coeff_YawMoment_delta_aile_data)
        Coeff_YawMoment_deltaAileron = Coeff_YawMoment_deltaAileron(deltaAileronDegree, alphaDegree)[0, 0]
        self.Coeff_YawMoment = (
                Coeff_SideForce_beta * self.beta +
                Coeff_SideForce_deltaRudder * deltaRudderRadient +
                self.geometry.span / (2 * self.TAS) * (Coeff_SideForce_p * self.p + Coeff_SideForce_r * self.r)
        )
        self.Coeff_RollMoment = (
                0.1 * Coeff_RollMoment_beta * self.beta +
                Coeff_RollMoment_deltaAileron +
                0.075 * Coeff_RollMoment_deltaRudder * deltaRudderRadient +
                self.geometry.span / (2 * self.TAS) * (Coeff_RollMoment_p * self.p + Coeff_RollMoment_r * self.r)
        )
        self.Coeff_YawMoment = (
                Coeff_YawMoment_beta * self.beta +
                Coeff_YawMoment_deltaAileron +
                0.075 * Coeff_YawMoment_deltaRudder * deltaRudderRadient +
                self.geometry.span / (2 * self.TAS) * (Coeff_YawMoment_p * self.p + Coeff_YawMoment_r * self.r)
        )


    def _calculate_engine_coeffs(self):
        deltaThrust = self.SteuerflaechenUndMotorStellung.deltaThrust
        # linearer Zusammenhang zwischen Drehzahl und Kraft
        motordrehzahl = np.interp(deltaThrust, self.geometry.stellbereichThrust, self.geometry.rpmMotor)  # rpm
        #todo: warum: motordrehzahl * 2
        self.motordrehzahlRadientProSekunde = (motordrehzahl * 2 * np.pi) / 60.0  # rad/s
        # Umdrehung fuehrt zu Kraft:
        # 1. Berechnung des Beiwerts
        # advance ratio J using the program JavaProp
        J = (self.TAS) / (self.motordrehzahlRadientProSekunde * self.geometry.propellerRadius)  # non-dimensional
        self.Coeff_Thrust = np.interp(J, self.geometry.J_data, self.geometry.CThrust_data)  # non-dimensional -> Beiwert des Propellers

    def _calculate_aerodynamic_forces(self):
        # Kräfteberechung im a_ks
        lift_a_ks = self.staudruck * self.Sw * self.Coeff_Lift
        drag_a_ks = self.staudruck * self.Sw * self.Coeff_Drag
        # conversion to f_ks
        lift_f_ks = self.umrechnungenKoordinaten.aero2flug([0, 0, -lift_a_ks], self.alpha, self.beta)
        drag_f_ks = self.umrechnungenKoordinaten.aero2flug([-drag_a_ks, 0, 0], self.alpha, self.beta)
        # Kräftenberechnung im g_ks
        weightForce_g_ks = [0, 0, self.geometry.mass * self.gravity]
        # conversion to f_ks
        weightForce_f_ks = self.umrechnungenKoordinaten.geo2flug(weightForce_g_ks, self.phi, self.theta, self.psi)
        #Fx, Fy, Fz im f_ks
        return (drag_f_ks + weightForce_f_ks + lift_f_ks)

    def _calculate_aerodynamic_moments(self):
        #L, M, N
        L_RollMoment = self.staudruck * self.Sw * self.geometry.span * self.Coeff_RollMoment
        M_PitchMoment = self.staudruck * self.Sw * self.geometry.chord * self.Coeff_PitchMoment
        N_YawMoment = self.staudruck * self.Sw * self.geometry.span * self.Coeff_YawMoment
        self.total_moments_f_ks = np.array([L_RollMoment, M_PitchMoment, N_YawMoment])
        return self.total_moments_f_ks

    def _calculate_engine_forces(self):
        forceThrust_f_ks = (2 / np.pi) ** 2 * self.rho * (self.motordrehzahlRadientProSekunde * self.geometry.propellerRadius) ** 2 * self.Coeff_Thrust
        return np.array([forceThrust_f_ks, 0, 0])


    # todo: _calculate_engine_moments
    def _calculate_engine_moments(self):
        pass


def main():
    aircraft = Aircraft()
    alphaDegree = 0
    aircraft.alpha = np.deg2rad(alphaDegree)
    aircraft._calculate_aerodynamic_longitudinal_coeffs()
    print(aircraft.getMoments())

if __name__ == '__main__':
    main()