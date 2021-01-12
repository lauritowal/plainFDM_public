import gym
from gym import spaces
import numpy as np
import struct
import sys
sys.path.append('..')
sys.path.append('.')

from plain_fdm.lfz_controlFDM.pid_regler import PidRegler
from plain_fdm.physikFDM.dynamic_system6DoF import DynamicSystem6DoF
from plain_fdm.config import Config
from plain_fdm.servicesFDM.udp_client import UdpClient
from plain_fdm.servicesFDM.plot_state import PlotState
from plain_fdm.servicesFDM.umrechnungen_koordinaten import UmrechnungenKoordinaten
config = Config()

from plain_fdm.physikFDM.aircraft_beaver import AircraftBaever


class WrapperOpenAI (gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, stepweite=0.05):
        super(WrapperOpenAI, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # nur aileron
        high_action_space = np.array([1.], dtype=np.float32)
        self.action_space = spaces.Box(low=-high_action_space, high=high_action_space, dtype=np.float32)
        # Zustandsraum 4 current_phi_dot, current_phi, abs(error_current_phi_target_phi), integration_error
        high_observation_space = np.array([np.inf, np.inf, np.inf, np.inf, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_observation_space, high=high_observation_space, dtype=np.float32)
        # reward
        self.reward_range = np.array([-np.inf, np.inf], dtype=np.float32)
        # spezielle Parameter für das Enviroment FDM
        #self.aircraft = Aircraft(config.geometrieClass)  # Ball oder C172
        self.aircraft_beaver = AircraftBaever()
        self.pid = PidRegler()
        self.dynamicSystem = DynamicSystem6DoF()
        self.umrechnungenKoordinaten = UmrechnungenKoordinaten()
        self.stepweite = stepweite
        self.udpClient = UdpClient('127.0.0.1', 5566)
        # frage: ist das die richtige Stelle, oder besser im DDPG-Controller
        self.targetValues = {'targetPhi': 0,
                             'targetTheta': 0,
                             'targetPsi': 0,
                             'targetSpeed': 0,
                             'target_z_dot': 0.0}
        self.envelopeBounds = {'phiMax': 20,
                               'phiMin': -20,
                               'thetaMax': 10,
                               'thetaMin': -30,
                               'speedMax': 80,
                               'speedMin': 33
                               }

        self.integration_stepsize = 3
        self.error_array_u = np.zeros(self.integration_stepsize)
        self.integration_error_u = 0
        self.error_array_theta = np.zeros(self.integration_stepsize)
        self.integration_error_theta = 0

        # fuer plotten
        self.plotter = PlotState()
        self.anzahlSteps = 0
        self.anzahlEpisoden = 0

    def reset(self):
        self.error_array_u = np.zeros(self.integration_stepsize)
        self.integration_error_u = 0
        self.error_array_theta = np.zeros(self.integration_stepsize)
        self.integration_error_theta = 0

        self.anzahlSteps = 1
        self.anzahlEpisoden += 1

        # set targets
        self.targetValues['targetSpeed'] = np.random.uniform(45, 55)  # m/s
        self.targetValues['targetTheta'] = np.random.uniform(-5, 5)  # m/s
        print('new Targets: ', self.targetValues)

        # set state at initial
        u_as_random = np.random.uniform(45, 55)
        #phi_as_random = np.deg2rad(np.random.uniform(0, 0))
        theta_as_random = np.deg2rad(np.random.uniform(-5, 5))

        self.aircraft_beaver.setState(
            np.array([u_as_random, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, theta_as_random, 0.0]))
        observation = self.user_defined_observation(self.aircraft_beaver.getState(), self.aircraft_beaver.z_dot_g_ks)
        return observation  # reward, done, info can't be included

    def step(self, action_command):
        self.anzahlSteps += 1
        self.aircraft_beaver.delta_elevator = np.deg2rad(np.clip(action_command[0], -1, 1) * (20))
        self.aircraft_beaver.delta_thrust = np.interp(action_command[1], [-1, 1], [0, 1])

        # Headline: phi wird mit PID-Regler stabilisiert
        self.aircraft_beaver.delta_aileron = self.pid._innerLoopAileron(np.deg2rad(0), self.aircraft_beaver.phi, self.aircraft_beaver.p, self.aircraft_beaver.delta_aileron)
        # Headline: integrate step
        solver = self.dynamicSystem.integrate(self.aircraft_beaver.getState(), self.aircraft_beaver.getForces(),
                                              self.aircraft_beaver.getMoments(),
                                              self.aircraft_beaver.mass, self.aircraft_beaver.inertia,
                                              self.stepweite)  # def integrate(self, state, mass, inertia, forces, moments, stepweite):
        # State1 in f_ks
        self.aircraft_beaver.setState(np.array(
            [solver.y[0][0], solver.y[1][0], solver.y[2][0], solver.y[3][0], solver.y[4][0], solver.y[5][0],
             solver.y[6][0], solver.y[7][0], solver.y[8][0], solver.y[9][0], solver.y[10][0], solver.y[11][0]]))
        #set Values for g_ks
        self.aircraft_beaver.x_dot_g_ks, self.aircraft_beaver.y_dot_g_ks, self.aircraft_beaver.z_dot_g_ks = self.umrechnungenKoordinaten.flug2geo(
            [self.aircraft_beaver.u, self.aircraft_beaver.v, self.aircraft_beaver.w], self.aircraft_beaver.phi, self.aircraft_beaver.theta, self.aircraft_beaver.psi)
        observation = self.user_defined_observation(self.aircraft_beaver.getState(), self.aircraft_beaver.z_dot_g_ks)
        reward = self.compute_reward(self.aircraft_beaver.getState(), self.aircraft_beaver.z_dot_g_ks)
        done = self.check_done(self.aircraft_beaver.getState())
        # Headline: ab hier für plotten
        self.plotter.addData(self.aircraft_beaver.getState(), self.aircraft_beaver.getForces(), self.aircraft_beaver.getMoments(),
                             self.aircraft_beaver.alpha, self.aircraft_beaver.beta,
                             np.rad2deg(
                                 self.aircraft_beaver.getSteuerflaechenUndMotorStellung()),
                             self.anzahlSteps + self.anzahlEpisoden * 1000)  # Headline ist anzupassen
        self.plotter.add_data_xyz([self.aircraft_beaver.x_geo, self.aircraft_beaver.y_geo, self.aircraft_beaver.z_geo], self.aircraft_beaver.z_dot_g_ks, self.anzahlSteps + self.anzahlEpisoden * 1000)

        return observation, reward, done, {}

    def render(self, mode='human'):
        self.udpClient.send((struct.pack('fff', self.aircraft_beaver.phi, 0, 0)))


    def close(self):
        pass

    def seed(self, seed=None):
        return

    def user_defined_observation(self, aircraft_state_f_ks, z_dot_g_ks):
        current_u = aircraft_state_f_ks[0]
        current_u_to_target_u = np.abs(current_u - self.targetValues['targetSpeed'])
        current_theta_dot = aircraft_state_f_ks[7]
        current_theta_rad = aircraft_state_f_ks[10]
        current_theta_to_target_theta_rad = np.abs(current_theta_rad - np.deg2rad(self.targetValues['targetTheta']))

        # enrichment of Observation: akkumulated error -> 1. rotate and add element 2. reduce to skalar
        self.error_array_u = np.roll(self.error_array_u, 1)
        self.error_array_u[-1] = current_u_to_target_u
        self.integration_error_u = np.add.reduce(self.error_array_u)

        self.error_array_theta = np.roll(self.error_array_theta, 1)
        self.error_array_theta[-1] = current_theta_to_target_theta_rad
        self.integration_error_theta = np.add.reduce(self.error_array_theta)


        user_defined_observation = np.asarray(
            [current_u, current_u_to_target_u,
             current_theta_rad, current_theta_dot, current_theta_to_target_theta_rad])

        return user_defined_observation

    def compute_reward(self, aircraft_state_f_ks, z_dot_g_ks):
        reward0 = self.reward_elevator(aircraft_state_f_ks, z_dot_g_ks)
        reward1 = self.reward_thrust(aircraft_state_f_ks, z_dot_g_ks)
        return reward0 + reward1

    def reward_elevator(self, aircraft_state_f_ks, z_dot_g_ks):
        current_theta_grad = np.rad2deg(aircraft_state_f_ks[10])
        reward0 = 0
        if current_theta_grad < self.envelopeBounds['thetaMin'] or current_theta_grad > self.envelopeBounds['thetaMax']:
            reward0 += -1000
        if np.abs(current_theta_grad - self.targetValues['targetTheta']) > 1:
            reward0 += -1 * np.abs(current_theta_grad - self.targetValues['targetTheta'])
        if np.abs(current_theta_grad - self.targetValues['targetTheta']) <= 1:
            reward0 += 0
        return reward0

    def reward_thrust(self, aircraft_state_f_ks, z_dot_g_ks):
        current_u = aircraft_state_f_ks[0]
        reward1 = 0
        if current_u < self.envelopeBounds['speedMin'] or current_u > self.envelopeBounds['speedMax']:
            reward1 += -1000
        if np.abs(current_u - self.targetValues['targetSpeed']) > 1:
            reward1 += -0.1 * np.abs(current_u - self.targetValues['targetSpeed'])
        if np.abs(current_u - self.targetValues['targetSpeed']) <= 1:
            reward1 += 0
        return reward1

    def check_done(self, observation):
        done = 0
        #conditions_if_reset =  all( [30 <= observation[0] <= 50, 30 <= observation[1] <= 50])
        if observation[0] < self.envelopeBounds['speedMin'] or observation[0] > self.envelopeBounds['speedMax']:
            print("speed limits")
            done = 1
        #conditions_if_reset_speed = any([observation[0] < 30, observation[0] > 50])
        if np.rad2deg(observation[9]) < self.envelopeBounds['phiMin'] or np.rad2deg(observation[9]) > self.envelopeBounds['phiMax']:
            print("roll limits", np.rad2deg(observation[9]))
            done = 1
        # conditions_if_reset_phi = any([self.envelopeBounds['phiMin'] > np.rad2deg(observation[9]), np.rad2deg(observation[9]) > self.envelopeBounds['phiMax']])
        if np.rad2deg(observation[10]) < self.envelopeBounds['thetaMin'] or np.rad2deg(observation[10]) > self.envelopeBounds['thetaMax']:
            print("pitch limits", np.rad2deg(observation[10]))
            done = 1
        return done

