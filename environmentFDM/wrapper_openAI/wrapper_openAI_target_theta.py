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
        high_observation_space = np.array([np.inf, np.inf, np.inf, np.inf, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_observation_space, high=high_observation_space, dtype=np.float32)
        # reward
        self.reward_range = np.array([-np.inf, np.inf], dtype=np.float32)
        # spezielle Parameter für das Enviroment FDM
        self.aircraft = Aircraft(config.geometrieClass)  # Ball oder C172
        self.pid = PidRegler()
        self.dynamicSystem = DynamicSystem6DoF()
        self.stepweite = stepweite
        self.udpClient = UdpClient('127.0.0.1', 5566)
        # frage: ist das die richtige Stelle, oder besser im DDPG-Controller
        self.targetValues = {'targetPhi': 0,
                             'targetTheta': 0,
                             'targetPsi': 0}
        self.envelopeBounds = {'phiMax': 20,
                               'phiMin': -20,
                               'thetaMax': 30,
                               'thetaMin': -30
                               }

        self.observationErrorAkkumulation = np.zeros(3)
        self.integration_error_stepsize_ = 0
        # fuer plotten
        self.plotter = PlotState()
        self.anzahlSteps = 0
        self.anzahlEpisoden = 0

        self.servo_command = 0
        self.action_servo_command_history = np.zeros(2)
        self.bandbreite_servo_actions = 0

    def reset(self):
        self.observationErrorAkkumulation = np.zeros(3)
        self.integration_error_stepsize_ = 0

        self.servo_command = 0
        self.action_servo_command_history = np.zeros(2)
        self.bandbreite_servo_actions = 0

        self.anzahlSteps = 1
        self.anzahlEpisoden += 1

        # set targets for different angles
        self.targetValues['targetTheta'] = np.random.uniform(-5, 5)
        print('new Target (deg): ', self.targetValues)

        # set state at initial
        phi_as_random = np.deg2rad(np.random.uniform(0, 0))
        theta_as_random = np.deg2rad(np.random.uniform(-3, 3))

        self.aircraft.setState(
            np.array([40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, phi_as_random, theta_as_random, 0.0]))
        observation = self.user_defined_observation(self.aircraft.getState())
        return observation  # reward, done, info can't be included

    def step(self, actionElevator):
        self.anzahlSteps += 1
        # action im Intervall [-1,1]
        # mapping auf Begrenzung der Steuerflächen
        self.servo_command = actionElevator[0]
        set_elevator = np.interp(actionElevator[0], [-1, 1],
                                 self.aircraft.SteuerflaechenUndMotorStellung.controlLimits['deltaElevatorBorder'])
        # print(set_aileron)
        self.aircraft.SteuerflaechenUndMotorStellung.deltaElevator = set_elevator
        # Headline: phi wird mit PID-Regler stabilisiert
        set_aileron = self.pid._innerLoopAileron(0, np.rad2deg(self.aircraft.phi), self.aircraft.p,
                                                 self.aircraft.SteuerflaechenUndMotorStellung.deltaAileron)
        self.aircraft.SteuerflaechenUndMotorStellung.deltaAileron = set_aileron
        # Headline: integrate step
        solver = self.dynamicSystem.integrate(self.aircraft.getState(), self.aircraft.getForces(), self.aircraft.getMoments(),
                                         self.aircraft.mass, self.aircraft.inertia,
                                         self.stepweite)  # def integrate(self, state, mass, inertia, forces, moments, stepweite):
        # State1
        self.aircraft.setState(np.array(
            [solver.y[0][0], solver.y[1][0], solver.y[2][0], solver.y[3][0], solver.y[4][0], solver.y[5][0],
             solver.y[6][0], solver.y[7][0], solver.y[8][0], solver.y[9][0], solver.y[10][0], solver.y[11][0]]))
        observation = self.user_defined_observation(self.aircraft.getState())
        reward = self.compute_reward(self.aircraft.getState())
        done = self.check_done(self.aircraft.getState())
        # Headline: ab hier für plotten
        self.plotter.addData(self.aircraft.getState(), self.aircraft.getForces(), self.aircraft.getMoments(), self.aircraft.alpha, self.aircraft.beta,
                        np.rad2deg(self.aircraft.SteuerflaechenUndMotorStellung.getSteuerflaechenUndMotorStellung()),
                        self.anzahlSteps + self.anzahlEpisoden * 400)  #Headline ist anzupassen

        return observation, reward, done, {}

    def render(self, mode='human'):
        self.udpClient.send((struct.pack('fff', self.aircraft.phi, 0, 0)))


    def close(self):
        pass

    def seed(self, seed=None):
        return

    def user_defined_observation(self, observation):
        # return: current_phi_dot, current_phi, abs(error_current_phi_target_phi), integration_error
        current_theta_dot = observation[7]
        current_theta = observation[10]
        error_current_theta_to_target_theta = np.abs(np.rad2deg(observation[10]) - self.targetValues['targetTheta'])

        self.observationErrorAkkumulation = np.roll(self.observationErrorAkkumulation, 1)
        self.observationErrorAkkumulation[-1] = error_current_theta_to_target_theta
        self.integration_error_stepsize_ = np.add.reduce(self.observationErrorAkkumulation)

        self.action_servo_command_history = np.roll(self.action_servo_command_history, len(self.action_servo_command_history) - 1)
        self.action_servo_command_history[-1] = self.servo_command
        self.bandbreite_servo_actions = np.abs(np.min(self.action_servo_command_history) - np.max(self.action_servo_command_history))

        # enrichment of Observation: akkumulated error -> 1. rotate and add element 2. reduce to skalar
        observation = np.asarray(
            [current_theta_dot, current_theta, error_current_theta_to_target_theta, self.integration_error_stepsize_, self.action_servo_command_history[0]])

        return observation

    def compute_reward(self, observation):
        reward = 0
        # exceeds bounds [-20, 20] -> -100
        if np.rad2deg(observation[10]) > self.envelopeBounds['thetaMax'] or np.rad2deg(observation[10]) < self.envelopeBounds['thetaMin']:
            reward += -100
        # Abweichung abs(target-current) > 1 -> -1
        if np.abs(np.rad2deg(observation[10]) - self.targetValues['targetTheta']) > 1:
            reward += -1
        # Abweichung abs(target-current) <= 1 -> 10
        if np.abs(np.rad2deg(observation[10]) - self.targetValues['targetTheta']) <= 1:
            reward += 10
        #reward += -0.01 * np.power((1 * self.action_servo_command_history[0] - 1 * self.action_servo_command_history[1]), 2)
        return reward

    def check_done(self, observation):
        done = 0
        #conditions_if_reset =  all( [30 <= observation[0] <= 50, 30 <= observation[1] <= 50])
        if observation[0] < 30 or observation[0] > 50:
            print("speed limits")
            done = 1
        #conditions_if_reset_speed = any([observation[0] < 30, observation[0] > 50])
        if self.envelopeBounds['phiMin'] > np.rad2deg(observation[9]) or np.rad2deg(observation[9]) > self.envelopeBounds['phiMax']:
            print("roll limits", np.rad2deg(observation[9]))
            done = 1
        # conditions_if_reset_phi = any([self.envelopeBounds['phiMin'] > np.rad2deg(observation[9]), np.rad2deg(observation[9]) > self.envelopeBounds['phiMax']])
        if self.envelopeBounds['thetaMin'] > np.rad2deg(observation[10]) or np.rad2deg(observation[10]) > self.envelopeBounds['thetaMax']:
            print("pitch limits", np.rad2deg(observation[10]))
            done = 1
        return done



if __name__ == "__main__":

    wrapper = WrapperOpenAI()
    #print(wrapper.reset())
    #print(wrapper.step(np.array([-1])))
    #print((wrapper.aircraft.getState()))
    #print(wrapper.check_done([40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, 0.0]))
    print(wrapper.compute_reward([40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(-21.0), -0.0, 0.0]))