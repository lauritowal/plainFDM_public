import gym
from gym import spaces
import numpy as np
import struct
import sys
sys.path.append('..')
sys.path.append('.')

from FDM.physikFDM.aircraft import Aircraft
from FDM.lfz_controlFDM.pid_regler import PidRegler
from FDM.physikFDM.dynamic_system6DoF import DynamicSystem6DoF
from FDM.config import Config
from FDM.servicesFDM.udp_client import UdpClient
from FDM.servicesFDM.plot_state import PlotState
from FDM.servicesFDM.umrechnungen_koordinaten import UmrechnungenKoordinaten
config = Config()


class WrapperOpenAI (gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, stepweite=0.01):
        super(WrapperOpenAI, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # nur aileron
        high_action_space = np.array([1.], dtype=np.float32)
        self.action_space = spaces.Box(low=-high_action_space, high=high_action_space, dtype=np.float32)
        # Zustandsraum 4 current_phi_dot, current_phi, abs(error_current_phi_target_phi), integration_error
        high_observation_space = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_observation_space, high=high_observation_space, dtype=np.float32)
        # reward
        self.reward_range = np.array([-np.inf, np.inf], dtype=np.float32)
        # spezielle Parameter für das Enviroment FDM
        self.aircraft = Aircraft(config.geometrieClass)  # Ball oder C172
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
                               'thetaMax': 30,
                               'thetaMin': -30,
                               'speedMax': 80,
                               'speedMin': 30
                               }

        self.observationErrorAkkumulation_for_u = np.zeros(3)
        self.integration_error_stepsize_for_u = 0
        # fuer plotten
        self.plotter = PlotState()
        self.anzahlSteps = 0
        self.anzahlEpisoden = 0

    def reset(self):
        self.observationErrorAkkumulation_for_u = np.zeros(3)
        self.integration_error_stepsize_for_u = 0

        self.anzahlSteps = 1
        self.anzahlEpisoden += 1

        # set targets
        self.targetValues['targetSpeed'] = np.random.uniform(45, 55)  # m/s
        self.targetValues['target_z_dot'] = 0  # m/s
        print('new Targets: ', self.targetValues)

        # set state at initial
        phi_as_random = np.deg2rad(np.random.uniform(0, 0))
        theta_as_random = np.deg2rad(np.random.uniform(-5, 5))

        self.aircraft.setState(
            np.array([50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, phi_as_random, theta_as_random, 0.0]))
        observation = self.user_defined_observation(self.aircraft.getState(), self.aircraft.z_dot_g_ks)
        return observation  # reward, done, info can't be included

    def step(self, action_command):
        self.anzahlSteps += 1

        self.servo_command_elevator = action_command[0]
        set_elevator = np.interp(self.servo_command_elevator, [-1, 1],
                                 self.aircraft.SteuerflaechenUndMotorStellung.controlLimits['deltaElevatorBorder'])
        self.aircraft.SteuerflaechenUndMotorStellung.deltaElevator = set_elevator

        self.servo_command_thrust = action_command[1]
        set_thrust = np.interp(self.servo_command_thrust, [-1, 1],
                               self.aircraft.SteuerflaechenUndMotorStellung.controlLimits['deltaThrustBorder'])
        self.aircraft.SteuerflaechenUndMotorStellung.deltaThrust = set_thrust

        # Headline: phi wird mit PID-Regler stabilisiert
        set_aileron = self.pid._innerLoopAileron(0, np.rad2deg(self.aircraft.phi), self.aircraft.p,
                                                 self.aircraft.SteuerflaechenUndMotorStellung.deltaAileron)
        self.aircraft.SteuerflaechenUndMotorStellung.deltaAileron = set_aileron
        # Headline: integrate step
        solver = self.dynamicSystem.integrate(self.aircraft.getState(), self.aircraft.getForces(),
                                              self.aircraft.getMoments(),
                                              self.aircraft.mass, self.aircraft.inertia,
                                              self.stepweite)  # def integrate(self, state, mass, inertia, forces, moments, stepweite):
        # State1 in f_ks
        self.aircraft.setState(np.array(
            [solver.y[0][0], solver.y[1][0], solver.y[2][0], solver.y[3][0], solver.y[4][0], solver.y[5][0],
             solver.y[6][0], solver.y[7][0], solver.y[8][0], solver.y[9][0], solver.y[10][0], solver.y[11][0]]))
        #set Values for g_ks
        self.aircraft.x_dot_g_ks, self.aircraft.y_dot_g_ks, self.aircraft.z_dot_g_ks = self.umrechnungenKoordinaten.flug2geo(
            [self.aircraft.u, self.aircraft.v, self.aircraft.w], self.aircraft.phi, self.aircraft.theta, self.aircraft.psi)
        observation = self.user_defined_observation(self.aircraft.getState(), self.aircraft.z_dot_g_ks)
        reward = self.compute_reward(self.aircraft.getState(), self.aircraft.z_dot_g_ks)
        done = self.check_done(self.aircraft.getState())
        # Headline: ab hier für plotten
        self.plotter.addData(self.aircraft.getState(), self.aircraft.getForces(), self.aircraft.getMoments(),
                             self.aircraft.alpha, self.aircraft.beta,
                             np.rad2deg(
                                 self.aircraft.SteuerflaechenUndMotorStellung.getSteuerflaechenUndMotorStellung()),
                             self.anzahlSteps + self.anzahlEpisoden * 1000)  # Headline ist anzupassen
        self.plotter.add_data_xyz([self.aircraft.x_geo, self.aircraft.y_geo, self.aircraft.z_geo], self.aircraft.z_dot_g_ks, self.anzahlSteps + self.anzahlEpisoden * 1000)

        return observation, reward, done, {}

    def render(self, mode='human'):
        self.udpClient.send((struct.pack('fff', self.aircraft.phi, 0, 0)))


    def close(self):
        pass

    def seed(self, seed=None):
        return

    def user_defined_observation(self, aircraft_state_f_ks, z_dot_g_ks):
        current_u = aircraft_state_f_ks[0]
        current_u_to_target_u = np.abs(current_u - self.targetValues['targetSpeed'])

        current_z_dot = z_dot_g_ks
        current_z_dot_to_target_z_dot = np.abs(current_z_dot - self.targetValues['target_z_dot'])

        current_theta = aircraft_state_f_ks[10]
        current_q = aircraft_state_f_ks[7]

        self.observationErrorAkkumulation_for_u = np.roll(self.observationErrorAkkumulation_for_u, 1)
        self.observationErrorAkkumulation_for_u[-1] = current_u_to_target_u
        self.integration_error_stepsize_for_u = np.add.reduce(self.observationErrorAkkumulation_for_u)

        # enrichment of Observation: akkumulated error -> 1. rotate and add element 2. reduce to skalar
        aircraft_state_f_ks = np.asarray(
            [current_u, current_u_to_target_u, self.integration_error_stepsize_for_u, current_theta, current_q, current_z_dot, current_z_dot_to_target_z_dot])

        return aircraft_state_f_ks

    def compute_reward(self, aircraft_state_f_ks, z_dot_g_ks):
        current_u = aircraft_state_f_ks[0]
        current_theta = aircraft_state_f_ks[10]
        current_z_dot = z_dot_g_ks
        reward = 0
        # exceeds bounds [-20, 20] -> -100
        if current_u < self.envelopeBounds['speedMin'] or current_u > self.envelopeBounds['speedMax']:
            reward += -1000
        if np.rad2deg(current_theta) < self.envelopeBounds['thetaMin'] or np.rad2deg(current_theta) > self.envelopeBounds['thetaMax']:
            reward += -1000
        # Abweichung abs(target-current) > 1 -> -1
        if np.abs(current_u - self.targetValues['targetSpeed']) > 1:
            reward += -1 * np.abs(current_u - self.targetValues['targetSpeed'])
        # Abweichung abs(target-current) <= 1 -> 10
        if np.abs(current_u - self.targetValues['targetSpeed']) <= 1:
            reward += 10
        # Abweichung abs(target-current) > 1 -> -1
        if np.abs(current_z_dot - self.targetValues['target_z_dot']) > 1:
            reward += -1 * np.abs(current_u - self.targetValues['target_z_dot'])
        # Abweichung abs(target-current) <= 1 -> 10
        if np.abs(current_z_dot - self.targetValues['target_z_dot']) <= 1:
            reward += 10
        #reward += -0.01 * np.power((1 * self.action_servo_command_history[0] - 1 * self.action_servo_command_history[1]), 2)
        return reward

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

