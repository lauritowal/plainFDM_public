import gym
from gym import spaces
import numpy as np
import struct
import sys
sys.path.append('..')
sys.path.append('.')

from plain_fdm.physikFDM import Aircraft_baever
from plain_fdm.lfz_controlFDM import PidRegler
from plain_fdm.physikFDM import DynamicSystem6DoF
from plain_fdm.config import Config
from plain_fdm.servicesFDM import UdpClient
from plain_fdm.servicesFDM import PlotState

config = Config()


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
        high_observation_space = np.array([np.inf, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_observation_space, high=high_observation_space, dtype=np.float32)
        # reward
        self.reward_range = np.array([-np.inf, np.inf], dtype=np.float32)
        # spezielle Parameter für das Enviroment FDM
        self.aircraft_beaver = Aircraft_baever()
        self.pid = PidRegler()
        self.dynamicSystem = DynamicSystem6DoF()
        self.stepweite = stepweite
        self.udpClient = UdpClient('127.0.0.1', 5566)
        # frage: ist das die richtige Stelle, oder besser im DDPG-Controller
        self.targetValues = {'targetPhi': 0,
                             'targetTheta': 0,
                             'targetPsi': 0,
                             'targetSpeed': 0}
        self.envelopeBounds = {'phiMax': 20,
                               'phiMin': -20,
                               'thetaMax': 30,
                               'thetaMin': -30,
                               'speedMax': 72,
                               'speedMin': 30
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

        # set targets
        self.targetValues['targetSpeed'] = np.random.uniform(47, 63)
        print('new Target: ', self.targetValues)

        # set state at initial
        phi_as_random = np.deg2rad(np.random.uniform(0, 0))
        theta_as_random = np.deg2rad(np.random.uniform(2.5, 3.5))

        self.aircraft_beaver.setState(
            np.array([40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, phi_as_random, theta_as_random, 0.0]))
        observation = self.user_defined_observation(self.aircraft_beaver.getState())
        return observation  # reward, done, info can't be included

    def step(self, actionThrust):
        self.anzahlSteps += 1
        # action im Intervall [-1,1]
        # mapping auf Begrenzung der Steuerflächen
        self.servo_command = actionThrust[0]
        self.aircraft_beaver.delta_thrust = np.interp(actionThrust[0], [-1, 1], [0.0, 1])  # Übersetzung der Ausgabe KNN zu Thrust-Setting
        # Headline: theta wird mit PID-Regler stabilisiert
        self.aircraft_beaver.delta_elevator = self.pid._innerLoopElevator(np.deg2rad(0), self.aircraft_beaver.theta,
                                                                        self.aircraft_beaver.q,
                                                                        self.aircraft_beaver.delta_elevator)
        # Headline: phi wird mit PID-Regler stabilisiert
        self.aircraft_beaver.delta_aileron = self.pid._innerLoopAileron(np.deg2rad(0), self.aircraft_beaver.phi,
                                                                        self.aircraft_beaver.p,
                                                                        self.aircraft_beaver.delta_aileron)
        # Headline: integrate step
        solver = self.dynamicSystem.integrate(self.aircraft_beaver.getState(), self.aircraft_beaver.getForces(), self.aircraft_beaver.getMoments(),
                                              self.aircraft_beaver.mass, self.aircraft_beaver.inertia,
                                              self.stepweite)  # def integrate(self, state, mass, inertia, forces, moments, stepweite):
        # State1
        self.aircraft_beaver.setState(np.array(
            [solver.y[0][0], solver.y[1][0], solver.y[2][0], solver.y[3][0], solver.y[4][0], solver.y[5][0],
             solver.y[6][0], solver.y[7][0], solver.y[8][0], solver.y[9][0], solver.y[10][0], solver.y[11][0]]))
        observation = self.user_defined_observation(self.aircraft_beaver.getState())
        reward = self.compute_reward(self.aircraft_beaver.getState())
        done = self.check_done(self.aircraft_beaver.getState())
        # Headline: ab hier für plotten
        self.plotter.addData(self.aircraft_beaver.getState(), self.aircraft_beaver.getForces(), self.aircraft_beaver.getMoments(), self.aircraft_beaver.alpha, self.aircraft_beaver.beta,
                             (self.aircraft_beaver.getSteuerflaechenUndMotorStellung()),
                             self.anzahlSteps + self.anzahlEpisoden * 1000)  #Headline ist anzupassen

        return observation, reward, done, {}

    def render(self, mode='human'):
        self.udpClient.send((struct.pack('fff', self.aircraft_beaver.phi, 0, 0)))


    def close(self):
        pass

    def seed(self, seed=None):
        return

    def user_defined_observation(self, aircraft_state_f_ks):
        current_speed = aircraft_state_f_ks[0]
        error_current_speed_to_target_speed = aircraft_state_f_ks[0] - self.targetValues['targetSpeed']

        self.action_servo_command_history = np.roll(self.action_servo_command_history, len(self.action_servo_command_history) - 1)
        self.action_servo_command_history[-1] = self.servo_command
        self.bandbreite_servo_actions = np.abs(np.min(self.action_servo_command_history) - np.max(self.action_servo_command_history))

        aircraft_state_f_ks = np.asarray(
            [current_speed, error_current_speed_to_target_speed])

        return aircraft_state_f_ks

    def compute_reward(self, aircraft_state_f_ks):
        current_u = aircraft_state_f_ks[0]
        reward1 = 0
        if current_u < self.envelopeBounds['speedMin'] or current_u > self.envelopeBounds['speedMax']:
            reward1 += -1000
        if np.abs(current_u - self.targetValues['targetSpeed']) > 1:
            reward1 += -1
        else:
            reward1 += 10
        return reward1

    def check_done(self, observation):
        done = 0
        #conditions_if_reset =  all( [30 <= observation[0] <= 50, 30 <= observation[1] <= 50])
        if observation[0] < 30 or observation[0] > 80:
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