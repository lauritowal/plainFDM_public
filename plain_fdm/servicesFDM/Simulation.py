import json
import struct

from plain_fdm.lfz_controlFDM.pid_regler import PidRegler
from plain_fdm.lfz_controlFDM.track_angle_controller import TrackAngleController
from plain_fdm.physikFDM.dynamic_system6DoF import DynamicSystem6DoF
from plain_fdm.servicesFDM.umrechnungen_koordinaten import UmrechnungenKoordinaten
from plain_fdm.servicesFDM.plot_state import PlotState
from plain_fdm.config import *
from plain_fdm.servicesFDM.udp_client import UdpClient
from plain_fdm.physikFDM.aircraft_beaver import AircraftBaever
import numpy as np
from plain_fdm.servicesFDM.utils import Utils

umrechnungenKoordinaten = UmrechnungenKoordinaten()
plotter = PlotState()
aircraft = AircraftBaever()
pid = PidRegler()
track_angle_controller = TrackAngleController()
dynamicSystem = DynamicSystem6DoF()
udpClient = UdpClient('127.0.0.1', 5566)


#zeitmessung für develop
import time
summe = 0
start = time.time()
#zeitmessung für develop
utils = Utils()
#todo: echtzeitanpassung über time und pause...
stepweite = 0.02
steps = 5000
# set initial conditions of a/c
aircraft.setState(40., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., psi=10.)
for step in range(steps):

    aircraftState0 = aircraft.getState()
    # Integral berechnen
    solver = dynamicSystem.integrate(aircraft.getState(), aircraft.getForces(), aircraft.getMoments(), aircraft.mass, aircraft.inertia, stepweite) # def integrate(self, state, mass, inertia, forces, moments, stepweite):
    #State1
    aircraft.setState(solver.y[0][0], solver.y[1][0], solver.y[2][0], solver.y[3][0], solver.y[4][0], solver.y[5][0], solver.y[6][0], solver.y[7][0], solver.y[8][0], solver.y[9][0], solver.y[10][0], solver.y[11][0])
    aircraftState1 = aircraft.getState()
    #Delta State
    aircraftStateDelta0_1 = aircraftState1  - aircraftState0
    #neue Position in g_ks
    [aircraft.x_geo, aircraft.y_geo, aircraft.z_geo] = [aircraft.x_geo, aircraft.y_geo, aircraft.z_geo] + umrechnungenKoordinaten.flug2geo([aircraftStateDelta0_1[3], aircraftStateDelta0_1[4], aircraftStateDelta0_1[5]], aircraft.phi, aircraft.theta, aircraft.psi)
    # Sinkgeschwindigkeit im g_ks
    [x_dot_g_ks, y_dot_g_ks, z_dot_g_ks] = umrechnungenKoordinaten.flug2geo([aircraftState1[0], aircraftState1[1], aircraftState1[2]], aircraft.phi, aircraft.theta, utils.normalize_angle(aircraft.psi))
    #send Position and angle to Unity
    udpClient.send((struct.pack('fff', aircraft.phi, aircraft.theta, 0)))
    # Headline: PID-Regler
    # aircraft.delta_aileron = track_angle_controller.bank_angle_hold(np.deg2rad(20), aircraft.phi, aircraft.p, aircraft.delta_aileron)
    aircraft.delta_aileron = track_angle_controller.heading_hold(heading_reference_deg=80,
                                                                 heading_current_deg=aircraft.phi,
                                                                 roll_angle_rate=aircraft.p,
                                                                 airspeed=aircraft.u,
                                                                 delta_aileron=aircraft.delta_aileron)

    aircraft.delta_elevator = pid._innerLoopElevator(np.deg2rad(0), aircraft.theta, aircraft.q, aircraft.delta_elevator)

    aircraft.delta_thrust = 1.0
    # aircraft.delta_rudder = track_angle_controller.turn_coordinator(aircraft.beta)
    # Headline: ab hier für plotten
    plotter.addData(aircraft.getState(), aircraft.getForces(), aircraft.getMoments(), aircraft.alpha, aircraft.beta, aircraft.getSteuerflaechenUndMotorStellung(), step)
    plotter.add_data_xyz([aircraft.x_geo, aircraft.y_geo, aircraft.z_geo], z_dot_g_ks, step)
    if step % 1000 == 0:
        print("theta", np.rad2deg(aircraft.theta))
        print("phi", np.rad2deg(aircraft.phi))
        print("psi", utils.normalize_angle(np.rad2deg(aircraft.psi)))
        print("beta", np.rad2deg(aircraft.beta))
        print("p", np.rad2deg(aircraft.p))
        print("delta_aileron", np.rad2deg(aircraft.delta_aileron))
        print("#############################")

#zeitmessung für develop
ende = time.time()
print('runtime: {:5.3f}s'.format(ende-start))
#zeitmessung für develop

#plotten
listData2Plot = ['theta/grad', 'deltaElevator', 'phi/grad', 'deltaThrust', 'u']
listData2Plot = ['psi/grad', 'theta/grad',  'phi/grad','deltaAileron', 'beta', 'p']
# listData2Plot = ['u', 'v', 'w', 'x', 'y', 'z', 'p', 'q', 'r', 'phi/grad', 'theta/grad', 'psi/grad', 'forceX', 'forceY', 'forceZ', 'momentsX', 'momentsY', 'momentsZ', 'alpha','beta', 'deltaElevator', 'deltaAileron', 'deltaRudder', 'deltaThrust','tVerlauf']
plotter.plot(listData2Plot)

print("theta", np.rad2deg(aircraft.theta))
print("phi", np.rad2deg(aircraft.phi))
print("psi", np.rad2deg(aircraft.psi))
print("beta", np.rad2deg(aircraft.beta))
print("p", np.rad2deg(aircraft.p))
print("delta_aileron", np.rad2deg(aircraft.delta_aileron))
print("#############################")

print('Dauer der Simulation: ', stepweite*steps, ' Sekunden')
print('theta: ', np.rad2deg(aircraft.theta))
print('Kräfte f_ks: ', aircraft.total_forces_f_ks)
print('Geschwindigkeit f_ks: ', aircraft.u, aircraft.v, aircraft.w)
print('TAS: ', np.sqrt(np.power(aircraft.u, 2) + np.power(aircraft.v, 2) + np.power(aircraft.w, 2)))


