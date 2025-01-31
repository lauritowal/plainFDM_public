import gym
from gym import spaces
import numpy as np
import struct
import sys
sys.path.append('..')
sys.path.append('.')

from plain_fdm.lfz_controlFDM import PidRegler
from plain_fdm.physikFDM import DynamicSystem6DoF
from plain_fdm.config import Config
from plain_fdm.servicesFDM import UdpClient
from plain_fdm.servicesFDM import PlotState
from plain_fdm.servicesFDM import UmrechnungenKoordinaten
config = Config()

from plain_fdm.physikFDM import Aircraft_baever


class WrapperOpenAI (gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, stepweite=0.02):
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
        self.aircraft_beaver = Aircraft_baever()
        self.pid = PidRegler()
        self.dynamicSystem = DynamicSystem6DoF()
        self.umrechnungenKoordinaten = UmrechnungenKoordinaten()
        self.stepweite = stepweite
        self.udpClient = UdpClient('127.0.0.1', 5566)
        # frage: ist das die richtige Stelle, oder besser im DDPG-Controller
        self.targetValues = {'targetPhi': 0,
                             'targetTheta_grad': 0,
                             'targetPsi': 0,
                             'targetSpeed': 0,
                             'target_z_dot': 0.0}
        self.envelopeBounds = {'phiMax': 20,
                               'phiMin': -20,
                               'thetaMax_grad': 14,
                               'thetaMin_grad': -30,
                               'speedMax': 72,
                               'speedMin': 33
                               }

        self.servo_command = 0
        self.action_servo_command_history = np.zeros(2)
        self.bandbreite_servo_actions = 0

        # fuer plotten
        self.plotter = PlotState()
        self.anzahlSteps = 0
        self.anzahlEpisoden = 0

    def reset(self):
        np.random.seed = 42
        self.servo_command = 0
        self.action_servo_command_history = np.zeros(2)
        self.bandbreite_servo_actions = 0

        self.anzahlSteps = 1
        self.anzahlEpisoden += 1

        # set targets
        self.targetValues['targetSpeed'] = np.random.uniform(48, 52)  # m/s
        self.targetValues['targetTheta_grad'] = np.random.uniform(-5, 5)  # m/s
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
        self.servo_command = action_command[0]
        self.anzahlSteps += 1
        self.aircraft_beaver.delta_elevator = np.deg2rad(np.clip(action_command[0], -1, 1) * (20))

        # Headline: phi wird mit PID-Regler stabilisiert
        self.aircraft_beaver.delta_aileron = self.pid._innerLoopAileron(np.deg2rad(0), self.aircraft_beaver.phi, self.aircraft_beaver.p, self.aircraft_beaver.delta_aileron)
        self.aircraft_beaver.delta_thrust = 0.8
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
        self.plotter.add_data_Ziel(self.targetValues['targetTheta_grad'], self.anzahlSteps + self.anzahlEpisoden * 1000)
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
        current_w = aircraft_state_f_ks[2]
        current_theta_grad = np.rad2deg(aircraft_state_f_ks[10])

        self.action_servo_command_history = np.roll(self.action_servo_command_history,
                                                    len(self.action_servo_command_history) - 1)
        self.action_servo_command_history[-1] = self.servo_command
        command_minimum = np.min(self.action_servo_command_history)
        command_maximum = np.max(self.action_servo_command_history)
        self.bandbreite_servo_actions = np.abs(command_minimum - command_maximum)


        user_defined_observation = np.asarray(
            [z_dot_g_ks, current_theta_grad, current_u, command_minimum, command_maximum])

        return user_defined_observation

    def compute_reward(self, aircraft_state_f_ks, z_dot_g_ks):
        reward0 = self.reward_elevator(aircraft_state_f_ks, z_dot_g_ks)
        return reward0

    def reward_elevator(self, aircraft_state_f_ks, z_dot_g_ks):
        current_theta_grad = np.rad2deg(aircraft_state_f_ks[10])
        current_theta_dot = aircraft_state_f_ks[7]
        current_u = aircraft_state_f_ks[0]
        reward0 = 0
        # out of bounds
        if current_theta_grad < self.envelopeBounds['thetaMin_grad'] or current_theta_grad > self.envelopeBounds['thetaMax_grad']:
            reward0 += -1000
        if current_u < self.envelopeBounds['speedMin'] or current_u > self.envelopeBounds['speedMax']:
            reward0 += -1000
        # Zielgröße Sinken/steigen
        if not 0.5 > z_dot_g_ks >= -0.5:
            reward0 += -1
        else:
            reward0 += 10

        reward0 += -1 * self.bandbreite_servo_actions
        return reward0

    def check_done(self, observation):
        done = 0
        #conditions_if_reset =  all( [30 <= observation[0] <= 50, 30 <= observation[1] <= 50])
        if observation[0] < self.envelopeBounds['speedMin'] or observation[0] > self.envelopeBounds['speedMax']:
            print("speed limits", observation[0])
            done = 1
        #conditions_if_reset_speed = any([observation[0] < 30, observation[0] > 50])
        if np.rad2deg(observation[9]) < self.envelopeBounds['phiMin'] or np.rad2deg(observation[9]) > self.envelopeBounds['phiMax']:
            print("roll limits", np.rad2deg(observation[9]))
            done = 1
        # conditions_if_reset_phi = any([self.envelopeBounds['phiMin'] > np.rad2deg(observation[9]), np.rad2deg(observation[9]) > self.envelopeBounds['phiMax']])
        if np.rad2deg(observation[10]) < self.envelopeBounds['thetaMin_grad'] or np.rad2deg(observation[10]) > self.envelopeBounds['thetaMax_grad']:
            print("pitch limits", np.rad2deg(observation[10]))
            done = 1
        return done

