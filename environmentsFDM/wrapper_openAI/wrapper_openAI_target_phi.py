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

    def __init__(self, stepweite=0.01):
        super(WrapperOpenAI, self).__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # nur aileron
        high_action_space = np.array([1.], dtype=np.float32)
        self.action_space = spaces.Box(low=-high_action_space, high=high_action_space, dtype=np.float32)
        # Zustandsraum 4 current_phi_dot, current_phi, abs(error_current_phi_target_phi), integration_error
        high_observation_space = np.array([np.inf, np.inf, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_observation_space, high=high_observation_space, dtype=np.float32)
        # reward
        self.reward_range = np.array([-np.inf, np.inf], dtype=np.float32)
        # spezielle Parameter für das Enviroment FDM
        self.aircraft = Aircraft_baever()  # Ball oder C172
        self.pid = PidRegler()
        self.dynamicSystem = DynamicSystem6DoF()
        self.stepweite = stepweite
        self.udpClient = UdpClient('127.0.0.1', 5566)
        # frage: ist das die richtige Stelle, oder besser im DDPG-Controller
        self.targetValues = {'targetPhi_grad': 0,
                             'targetTheta_grad': 0,
                             'targetPsi': 0,
                             'targetSpeed': 0,
                             'target_z_dot': 0.0}
        self.envelopeBounds = {'phiMax_grad': 30,
                               'phiMin_grad': -30,
                               'thetaMax_grad': 30,
                               'thetaMin_grad': -30,
                               'speedMax': 72,
                               'speedMin': 33
                               }


        # fuer plotten
        self.plotter = PlotState()
        self.anzahlSteps = 0
        self.anzahlEpisoden = 0

        self.observationErrorAkkumulation = np.zeros(3)
        self.integration_error_stepsize_ = 0

        self.servo_command = 0
        self.action_servo_command_history = np.zeros(2)
        self.bandbreite_servo_actions = 0

    def reset(self):
        np.random.seed()
        self.observationErrorAkkumulation = np.zeros(3)
        self.integration_error_stepsize_ = 0

        self.servo_command = 0
        self.action_servo_command_history = np.zeros(2)
        self.bandbreite_servo_actions = 0
        self.anzahlSteps = 1
        self.anzahlEpisoden += 1
        self.targetValues['targetPhi_grad'] = np.random.uniform(-20, 20)
        print('new Target (deg): ', self.targetValues)
        phi_as_random = np.deg2rad(np.random.uniform(-25, 25))
        self.aircraft.setState(
            np.array([40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, phi_as_random, np.deg2rad(-1), 0.0]))
        observation = self.user_defined_observation(self.aircraft.getState())
        return observation  # reward, done, info can't be included

    def step(self, actionAileron):
        self.anzahlSteps += 1
        # action im Intervall [-1,1]
        # mapping auf Begrenzung der Steuerflächen
        self.servo_command = actionAileron[0]
        self.aircraft.delta_aileron = np.deg2rad(np.clip(self.servo_command, -1, 1) * (-15))  # im FDM Beaver ist das
        # Headline: pitch wird mit PID-Regler stabilisiert
        self.aircraft.delta_elevator = self.pid._innerLoopElevator(np.deg2rad(6), self.aircraft.theta, self.aircraft.q, self.aircraft.delta_elevator)
        # Headline: integrate steps
        for x in range(10):
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
                        np.rad2deg(self.aircraft.getSteuerflaechenUndMotorStellung()),
                        self.anzahlSteps + self.anzahlEpisoden * 400)  #Headline ist anzupassen
        self.plotter.add_data_Ziel(self.targetValues['targetPhi_grad'], self.anzahlSteps + self.anzahlEpisoden * 400)

        return observation, reward, done, {}

    def render(self, mode='human'):
        self.udpClient.send((struct.pack('fff', self.aircraft.phi, 0, 0)))


    def close(self):
        pass

    def seed(self, seed=None):
        return

    def user_defined_observation(self, observation):
        # return: current_phi_dot, current_phi, abs(error_current_phi_target_phi), integration_error
        current_phi_dot = observation[6]
        current_phi = np.rad2deg(observation[9])
        error_current_phi_to_target_phi = current_phi - self.targetValues['targetPhi_grad']

        self.observationErrorAkkumulation = np.roll(self.observationErrorAkkumulation, 1)
        self.observationErrorAkkumulation[-1] = (error_current_phi_to_target_phi)
        self.integration_error_stepsize_ = np.add.reduce(self.observationErrorAkkumulation)

        self.action_servo_command_history = np.roll(self.action_servo_command_history, len(self.action_servo_command_history) - 1)
        self.action_servo_command_history[-1] = self.servo_command
        command_minimum = np.min(self.action_servo_command_history)
        command_maximum = np.max(self.action_servo_command_history)
        self.bandbreite_servo_actions = np.abs(command_minimum - command_maximum)

        observation = np.asarray(
            [current_phi_dot, current_phi, error_current_phi_to_target_phi])

        return observation

    def compute_reward(self, observation):
        reward = 0
        # exceeds bounds [-20, 20] -> -100
        if np.rad2deg(observation[9]) > self.envelopeBounds['phiMax_grad'] or np.rad2deg(observation[9]) < self.envelopeBounds['phiMin_grad']:
            reward += -1000
        # Abweichung abs(target-current) > 1 -> -1
        if (np.abs(np.rad2deg(observation[9]) - self.targetValues['targetPhi_grad'])) > 1.0:
            reward += -1
        if (np.abs(np.rad2deg(observation[9]) - self.targetValues['targetPhi_grad'])) < 1.0:
            reward += 100
        return reward

    def check_done(self, observation):
        done = 0
        # conditions_if_reset =  all( [30 <= observation[0] <= 50, 30 <= observation[1] <= 50])
        if observation[0] < self.envelopeBounds['speedMin'] or observation[0] > self.envelopeBounds['speedMax']:
            print("speed limits", observation[0])
            done = 1
        # conditions_if_reset_speed = any([observation[0] < 30, observation[0] > 50])
        if np.rad2deg(observation[9]) < self.envelopeBounds['phiMin_grad'] or np.rad2deg(observation[9]) > \
                self.envelopeBounds['phiMax_grad']:
            print("roll limits", np.rad2deg(observation[9]))
            done = 1
        # conditions_if_reset_phi = any([self.envelopeBounds['phiMin'] > np.rad2deg(observation[9]), np.rad2deg(observation[9]) > self.envelopeBounds['phiMax']])
        if np.rad2deg(observation[10]) < self.envelopeBounds['thetaMin_grad'] or np.rad2deg(observation[10]) > \
                self.envelopeBounds['thetaMax_grad']:
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