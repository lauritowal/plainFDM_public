import json

import gym
from gym import spaces
import numpy as np
import sys

from plain_fdm.config import Config
from plain_fdm.lfz_controlFDM.pid_regler import PidRegler
from plain_fdm.physikFDM.aircraft_beaver import AircraftBaever
from plain_fdm.physikFDM.dynamic_system6DoF import DynamicSystem6DoF
from plain_fdm.servicesFDM.plot_state import PlotState
from plain_fdm.servicesFDM.udp_client import UdpClient

config = Config()


class WrapperOpenAI (gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, stepweite=0.01):
        super(WrapperOpenAI, self).__init__()

        self.action_space = spaces.Box(np.array([-np.pi]), np.array([np.pi]), dtype=np.float32)

       # Zustandsraum 4 current_phi_dot, current_phi, abs(error_current_phi_target_phi), integration_error
        high_observation_space = np.array([np.inf, np.inf, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high_observation_space, high=high_observation_space, dtype=np.float32)

        self.reward_range = np.array([-np.inf, np.inf], dtype=np.float32)

        self.aircraft = AircraftBaever()
        self.pid = PidRegler()
        self.dynamicSystem = DynamicSystem6DoF()
        self.stepweite = stepweite
        self.udpClient = UdpClient('127.0.0.1', 5566)

        # frage: ist das die richtige Stelle, oder besser im DDPG-Controller
        self.targetValues = {
            'targetPhi_grad': 0,
            'targetTheta_grad': 0,
            'targetPsi': 0,
            'targetSpeed': 0,
            'target_z_dot': 0.0
        }

        self.envelopeBounds = {
            'phiMax_grad': 30,
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
        self.aircraft.setState(np.random.uniform(40, 55), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, phi_as_random, np.deg2rad(-1), 0.0)
        observation = self.__create_observation(self.aircraft.getState())
        return observation  # reward, done, info can't be included

    def step(self, heading_reference):
        self.anzahlSteps += 1

        # mapping auf Begrenzung der Steuerflächen
        self.servo_command = heading_reference
        self.aircraft.delta_aileron = self.pid.get_aileron_command(heading_reference=heading_reference,
                                                                   heading=self.aircraft.psi,
                                                                   roll_angle=self.aircraft.phi,
                                                                   roll_angle_rate=self.aircraft.p,
                                                                   delta_aileron=self.aircraft.delta_aileron)
        print("delta_aileron: ", self.aircraft.delta_aileron)

        solver = self.dynamicSystem.integrate(state=self.aircraft.getState(),
                                              forces=self.aircraft.getForces(),
                                              moments=self.aircraft.getMoments(),
                                              mass=self.aircraft.mass,
                                              inertia=self.aircraft.inertia,
                                              stepweite=self.stepweite)
        # State1
        self.aircraft.setState(*solver.y.flatten())

        observation = self.__create_observation(self.aircraft.getState())
        reward = self.__compute_reward(self.aircraft.getState())
        done = self.__is_done(self.aircraft.getState())

        self.__plot()

        return observation, reward, done, {}

    def render(self, mode='human'):
        message = json.dumps({
            "pitchAngle": self.aircraft.theta,
            "rollAngle": self.aircraft.phi,
            "yawAngle": self.aircraft.psi
        })
        print("udp package: ", message)
        self.udpClient.send(message.encode())


    def close(self):
        pass

    def seed(self, seed=None):
        return

    """
    private methods
    """
    def __create_observation(self, observation):
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

    def __compute_reward(self, observation):
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

    def __is_done(self, observation):
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

    def __plot(self):
        self.plotter.addData(
            self.aircraft.getState(),
            self.aircraft.getForces(),
            self.aircraft.getMoments(),
            self.aircraft.alpha,
            self.aircraft.beta,
            np.rad2deg(self.aircraft.getSteuerflaechenUndMotorStellung()),
            self.anzahlSteps + self.anzahlEpisoden * 400
        )
        self.plotter.add_data_Ziel(self.targetValues['targetPhi_grad'], self.anzahlSteps + self.anzahlEpisoden * 400)

if __name__ == "__main__":
    env = WrapperOpenAI()
    print(env.reset())

    action = env.action_space.sample()
    # env.step(np.deg2rad(20))

    for _ in range(1000):
        env.render()
        print("env.action_space.sample()", env.action_space.sample())
        action = env.action_space.sample()
        env.step(np.rad2deg(20))  # take a random action
    env.close()