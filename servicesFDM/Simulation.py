import struct
import sys



from FDM.physikFDM.aircraft import Aircraft
from FDM.lfz_controlFDM.pid_regler import PidRegler
from FDM.servicesFDM.umrechnungen_koordinaten import UmrechnungenKoordinaten
from FDM.servicesFDM.plot_state import PlotState
from FDM.physikFDM.dynamic_system6DoF import *
from FDM.config import *
from FDM.servicesFDM.udp_client import UdpClient
from FDM.physikFDM.aircraft_beaver import Aircraft_baever



config = Config()
umrechnungenKoordinaten = UmrechnungenKoordinaten()
plotter = PlotState()
# todo: config anpassen
#aircraft = Aircraft(config.geometrieClass)  # Ball oder C172
aircraft = Aircraft_baever()
pid = PidRegler()
dynamicSystem = DynamicSystem6DoF()
udpClient = UdpClient('127.0.0.1', 5566)


#zeitmessung für develop
import time
summe = 0
start = time.time()
#zeitmessung für develop

#todo: echtzeitanpassung über time und pause...
stepweite = 0.05
steps = 3000
# set initial conditions of a/c
aircraft.setState([40., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
for step in range(steps):

    aircraftState0 = aircraft.getState()
    # Integral berechnen
    solver = dynamicSystem.integrate(aircraft.getState(), aircraft.getForces(), aircraft.getMoments(), aircraft.mass, aircraft.inertia, stepweite) # def integrate(self, state, mass, inertia, forces, moments, stepweite):
    #State1
    aircraft.setState([solver.y[0][0], solver.y[1][0], solver.y[2][0], solver.y[3][0], solver.y[4][0], solver.y[5][0], solver.y[6][0], solver.y[7][0], solver.y[8][0], solver.y[9][0], solver.y[10][0], solver.y[11][0]])
    aircraftState1 = aircraft.getState()
    #Delta State
    aircraftStateDelta0_1 = aircraftState1  - aircraftState0
    #neue Position in g_ks
    [aircraft.x_geo, aircraft.y_geo, aircraft.z_geo] = [aircraft.x_geo, aircraft.y_geo, aircraft.z_geo] + umrechnungenKoordinaten.flug2geo([aircraftStateDelta0_1[3], aircraftStateDelta0_1[4], aircraftStateDelta0_1[5]], aircraft.phi, aircraft.theta, aircraft.psi)
    # Sinkgeschwindigkeit im g_ks
    [x_dot_g_ks, y_dot_g_ks, z_dot_g_ks] = umrechnungenKoordinaten.flug2geo([aircraftState1[0], aircraftState1[1], aircraftState1[2]], aircraft.phi, aircraft.theta, aircraft.psi)
    #send Position and angle to Unity
    udpClient.send((struct.pack('fff', aircraft.phi, aircraft.theta, 0)))
    # Headline: PID-Regler
    aircraft.SteuerflaechenUndMotorStellung.deltaElevator = pid._innerLoopElevator(np.deg2rad(5), aircraft.theta, aircraft.q, np.deg2rad(aircraft.SteuerflaechenUndMotorStellung.deltaElevator))
    aircraft.SteuerflaechenUndMotorStellung.deltaAileron = pid._innerLoopAileron((0), aircraft.phi, aircraft.p, np.deg2rad(aircraft.SteuerflaechenUndMotorStellung.deltaAileron))
    aircraft.SteuerflaechenUndMotorStellung.deltaThrust = 0.0
    # Headline: ab hier für plotten
    plotter.addData(aircraft.getState(), aircraft.getForces(), aircraft.getMoments(), aircraft.alpha, aircraft.beta, np.rad2deg(aircraft.SteuerflaechenUndMotorStellung.getSteuerflaechenUndMotorStellung()), step)
    plotter.add_data_xyz([aircraft.x_geo, aircraft.y_geo, aircraft.z_geo], z_dot_g_ks, step)
    if step % 1000 == 0:
        print("dfghj")

#zeitmessung für develop
ende = time.time()
print('runtime: {:5.3f}s'.format(ende-start))
#zeitmessung für develop

#plotten
listData2Plot = ['theta/grad', 'phi/grad', 'u']  #['u', 'v', 'w', 'x', 'y', 'z', 'p', 'q', 'r', 'phi/grad', 'theta/grad', 'psi/grad', 'forceX', 'forceY', 'forceZ', 'momentsX', 'momentsY', 'momentsZ', 'alpha','beta', 'deltaElevator', 'deltaAileron', 'deltaRudder', 'deltaThrust','tVerlauf']
plotter.plot(listData2Plot)


print('Dauer der Simulation: ', stepweite*steps, ' Sekunden')
print('theta: ', np.rad2deg(aircraft.theta))
print('Kräfte f_ks: ', aircraft.total_forces_f_ks)
print('Geschwindigkeit f_ks: ', aircraft.u, aircraft.v, aircraft.w)
print('TAS: ', np.sqrt(np.power(aircraft.u, 2) + np.power(aircraft.v, 2) + np.power(aircraft.w, 2)))


