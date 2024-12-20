# Author: Changan Chen <changanvr@gmail.com>
# Modified by: Keyu Li <kyli@link.cuhk.edu.hk>

from __future__ import absolute_import
import logging
import gym
import matplotlib.lines as mlines
import numpy as np
import rvo2
import torch
from matplotlib import patches
from numpy.linalg import norm
from crowd_sim.envs.utils.human import Human
from crowd_sim.envs.utils.info import *
from crowd_sim.envs.utils.utils import point_to_segment_dist
import random
import math


class CrowdSim(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        """
        Movement simulation for n+1 agents
        Agent can either be human or robot.
        humans are controlled by a unknown and fixed policy.
        robot is controlled by a known and learnable policy.

        """
        self.time_limit = None
        self.time_step = None
        self.robot = None
        self.robot_path_length = 0  # @lky
        self.humans = None
        self.global_time = None
        self.human_times = None
        # reward function
        self.success_reward = None
        self.collision_penalty = None
        self.discomfort_dist = None
        self.discomfort_penalty_factor = None
        self.heading_reward = None
        self.maintaining_reward = None
        self.standard_variation = None
        self.collision_wall_penalty = None
	self.R_safe = None
	self.R_min = None
        # simulation configuration
        self.config = None
        self.case_capacity = None
        self.case_size = None
        self.case_counter = None
        self.randomize_attributes = None
        self.train_val_sim = None
        self.test_sim = None
        self.square_width = None
        self.circle_radius = None
        self.human_num = None
        # for visualization
        self.states = None
        self.action_values = None
        self.attention_weights = None
        # for block_area
        self.block_area1 = None
        self.block_area2 = None
        self.block_area3 = None
        self.block_area4 = None

    def set_human_num(self, human_num):
        self.human_num = human_num

    def set_humans(self, humans):
        self.humans = humans


    def configure(self, config):
        self.config = config
        self.time_limit = config.getint('env', 'time_limit')
        self.time_step = config.getfloat('env', 'time_step')
        self.randomize_attributes = config.getboolean('env', 'randomize_attributes')
        self.success_reward = config.getfloat('reward', 'success_reward')
        self.collision_penalty = config.getfloat('reward', 'collision_penalty')
        self.discomfort_dist = config.getfloat('reward', 'discomfort_dist')
        self.discomfort_penalty_factor = config.getfloat('reward', 'discomfort_penalty_factor')
        self.heading_reward = config.getfloat('reward', 'heading_reward')
        self.maintaining_reward = config.getfloat('reward', 'maintaining_reward')
        self.standard_variation = config.getfloat('reward', 'standard_variation')
        self.collision_wall_penalty = config.getfloat('reward', 'collision_wall_penalty')
	self.R_safe = config.getfloat('reward', 'R_safe')
	self.R_min = config.getfloat('reward', 'R_min')        
	# Please input four points to make block area
        # Please input the coordinates of the top-right point of the rectangle in clockwise order.
        self.block_area1 = [[6,6],[2,6],[2,2],[6,2]]
        self.block_area2 = [[-2,6],[-6,6],[-6,2],[-2,2]]
        self.block_area3 = [[-2,-2],[-6,-2],[-6,-6],[-2,-6]]
        self.block_area4 = [[6,-2],[2,-2],[2,-6],[6,-6]]
	self.blocks = [self.block_area1, self.block_area2, self.block_area3, self.block_area4]
        if self.config.get('humans', 'policy') == 'orca':
            self.case_capacity = {'train': np.iinfo(np.uint32).max - 2000, 'val': 1000, 'test': 1000}
            self.case_size = {'train': np.iinfo(np.uint32).max - 2000, 'val': config.getint('env', 'val_size'),
                              'test': config.getint('env', 'test_size')}
            self.train_val_sim = config.get('sim', 'train_val_sim')
            self.test_sim = config.get('sim', 'test_sim')
            self.square_width = config.getfloat('sim', 'square_width')
            self.circle_radius = config.getfloat('sim', 'circle_radius')
            self.human_num = config.getint('sim', 'human_num')
        else:
            raise NotImplementedError
        self.case_counter = {'train': 0, 'test': 0, 'val': 0}

        logging.info('human number: {}'.format(self.human_num))
        if self.randomize_attributes:
            logging.info("Randomize human's radius and preferred speed")
        else:
            logging.info("Not randomize human's radius and preferred speed")
        logging.info('Training simulation: {}, test simulation: {}'.format(self.train_val_sim, self.test_sim))
        logging.info('Square width: {}, circle width: {}'.format(self.square_width, self.circle_radius))

    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def farthest_point_and_distance(self, robot_position, wall_areas):
        farthest_point = None
        max_distance = float('-inf')

        for area in wall_areas:
            for vertex in area:
                dist = self.distance(robot_position, vertex)
                if dist > max_distance:
                    max_distance = dist
                    farthest_point = vertex

        return farthest_point, max_distance

    def compute_min_distance_to_block(self, robot_position, wall_areas):
        min_distance = float('inf')
        max_distance = 0

        for wall_area in wall_areas:
            distance_to_wall, point_on_line = self.compute_distance_to_wall(robot_position, wall_area)

            if distance_to_wall < min_distance:
                min_distance = distance_to_wall
                closest_point = point_on_line

        return min_distance, closest_point

    def compute_distance_to_wall(self, robot_position, wall_area):
	x, y = robot_position
	distances = {}
	distances_list = []

	for i in range(len(wall_area)):
		
	    x1, y1 = wall_area[i]

	    x2, y2 = wall_area[(i + 1) % len(wall_area)]  

	    distance, point = self.point_to_line_segment_distance(x, y, x1, y1, x2, y2)
		
	    distances[distance] = point
	    distances_list.append(distance)

	min_dist = min(distances_list)
	min_point = distances[min_dist]

	return min_dist, min_point

    def point_to_line_segment_distance(self, x, y, x1, y1, x2, y2):
        length_squared = (x2 - x1)**2 + (y2 - y1)**2

        if length_squared == 0:
            return math.sqrt((x - x1)**2 + (y - y1)**2)

        t = max(0, min(1, ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / length_squared))
        px = x1 + t * (x2 - x1)
        py = y1 + t * (y2 - y1)

        distance = math.sqrt((x - px)**2 + (y - py)**2)

        return distance, [px, py]


    def set_robot(self, robot):
        self.robot = robot

    def generate_random_human_position(self, human_num, rule):
        """
        Generate human position according to certain rule
        Rule square_crossing: generate start/goal position at two sides of y-axis
        Rule circle_crossing: generate start position on a circle, goal position is at the opposite side

        :param human_num:
        :param rule:
        :return:
        """
        # initial min separation distance to avoid danger penalty at beginning
        if rule == 'square_crossing':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_square_crossing_human())
        elif rule == 'circle_crossing':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_circle_crossing_human())
        elif rule == 'static':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_static_human())
        elif rule == 'no':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_fake_human())
        elif rule == 'circle_static':
            self.humans = []
            for i in range(human_num):
                if i < 2:
                    self.humans.append(self.generate_static_human())
                else:
                    self.humans.append(self.generate_circle_crossing_human())
        elif rule == 'square_static':
            self.humans = []
            for i in range(human_num):
                if i < 2:
                    self.humans.append(self.generate_static_human())
                else:
                    self.humans.append(self.generate_square_crossing_human())
        elif rule == 'mixed':
            # mix different raining simulation with certain distribution
            static_human_num = {0: 0.05, 1: 0.2, 2: 0.2, 3: 0.3, 4: 0.1, 5: 0.15}
            dynamic_human_num = {1: 0.3, 2: 0.3, 3: 0.2, 4: 0.1, 5: 0.1}
            static = True if np.random.random() < 0.2 else False
            prob = np.random.random()
            for key, value in sorted(static_human_num.items() if static else dynamic_human_num.items()):
                if prob - value <= 0:
                    human_num = key
                    break
                else:
                    prob -= value
            self.humans = []
            if static:
                print("mode: static")
                # randomly initialize static objects in a square of (width, height)
                width = 4
                height = 8
                if human_num == 0:
                    print("human num: 0, set fake human:(0, -10, 0, -10, 0, 0, 0)")
                    human = Human(self.config, 'humans')
                    human.set(0, -10, 0, -10, 0, 0, 0)
                    self.humans.append(human)
                for i in range(human_num):
                    human = Human(self.config, 'humans')
                    if np.random.random() > 0.5:
                        sign = -1
                    else:
                        sign = 1
                    while True:
                        px = np.random.random() * width * 0.5 * sign
                        py = (np.random.random() - 0.5) * height
                        collide = False
                        for agent in [self.robot] + self.humans:
                            if norm((px - agent.px, py - agent.py)) < human.radius + agent.radius + self.discomfort_dist:
                                collide = True
                                break
                        if not collide:
                            break
                    human.set(px, py, px, py, 0, 0, 0)
                    self.humans.append(human)
            else:
                # the first 2 two humans will be in the circle crossing scenarios
                # the rest humans will have a random starting and end position
                print("mode: dynamic")
                for i in range(human_num):
                    if i < 2:
                        human = self.generate_circle_crossing_human()
                    else:
                        human = self.generate_square_crossing_human()
                    self.humans.append(human)
            self.human_num = len(self.humans)
            self.human_times = [0] * self.human_num
            print("human number:", self.human_num)
        else:
            raise ValueError("Rule doesn't exist")

    def generate_fake_human(self):
        human = Human(self.config, 'humans')
        human.v_pref = 0
        human.radius = 0.3
        while True:
            angle = np.random.random() * np.pi * 2
            # add some noise to simulate all the possible cases robot could meet with human
            px = (20) * np.cos(angle)
            py = (20) * np.sin(angle)
            collide = False
            for agent in [self.robot] + self.humans:
                min_dist = human.radius + agent.radius + self.discomfort_dist
                if norm((px - agent.px, py - agent.py)) < min_dist or \
                        norm((px - agent.gx, py - agent.gy)) < min_dist:
                    collide = True
                    break    # jump out of 'for' loop
            if (px<self.block_area1[0][0]+human.radius and px>self.block_area1[2][0]-human.radius) and (py<self.block_area1[0][1]+human.radius and py>self.block_area1[2][1]-human.radius):
                collide = True
            if (px<self.block_area2[0][0]+human.radius and px>self.block_area2[2][0]-human.radius) and (py<self.block_area2[0][1]+human.radius and py>self.block_area2[2][1]-human.radius):
                collide = True
            if (px<self.block_area3[0][0]+human.radius and px>self.block_area3[2][0]-human.radius) and (py<self.block_area3[0][1]+human.radius and py>self.block_area3[2][1]-human.radius):
                collide = True
            if (px<self.block_area4[0][0]+human.radius and px>self.block_area4[2][0]-human.radius) and (py<self.block_area4[0][1]+human.radius and py>self.block_area4[2][1]-human.radius):
                collide = True
            if not collide:
                break        # jump out of 'while' loop
        human.set(px, py, px, py, 0, 0, 0)
        return human


    def generate_static_human(self):
        human = Human(self.config, 'humans')
        """if self.randomize_attributes:
            human.v_pref = human0.v_pref
            human.radius = human0.radius"""
        if self.randomize_attributes:
            human.random_radius()
        human.v_pref = 0
        while True:
            px = random.uniform(-4, 4)
            py = random.uniform(-4, 4)
            collide = False
            for agent in [self.robot] + self.humans:
                min_dist = human.radius + agent.radius + self.discomfort_dist
                if norm((px - agent.px, py - agent.py)) < min_dist or \
                        norm((px - agent.gx, py - agent.gy)) < min_dist:
                    collide = True
                    break    # jump out of 'for' loop
            if (px<self.block_area1[0][0]+human.radius and px>self.block_area1[2][0]-human.radius) and (py<self.block_area1[0][1]+human.radius and py>self.block_area1[2][1]-human.radius):
                collide = True
            if (px<self.block_area2[0][0]+human.radius and px>self.block_area2[2][0]-human.radius) and (py<self.block_area2[0][1]+human.radius and py>self.block_area2[2][1]-human.radius):
                collide = True
            if (px<self.block_area3[0][0]+human.radius and px>self.block_area3[2][0]-human.radius) and (py<self.block_area3[0][1]+human.radius and py>self.block_area3[2][1]-human.radius):
                collide = True
            if (px<self.block_area4[0][0]+human.radius and px>self.block_area4[2][0]-human.radius) and (py<self.block_area4[0][1]+human.radius and py>self.block_area4[2][1]-human.radius):
                collide = True
            if not collide:
                break        # jump out of 'while' loop
        human.set(px, py, px, py, 0, 0, 0)
        return human

    def generate_circle_crossing_human(self):
        human = Human(self.config, 'humans')
        if self.randomize_attributes:
            human.sample_random_attributes()
        while True:
            angle = np.random.random() * np.pi * 2
            # add some noise to simulate all the possible cases robot could meet with human
            px_noise = (np.random.random() - 0.5) * human.v_pref
            py_noise = (np.random.random() - 0.5) * human.v_pref
            px = (self.circle_radius) * np.cos(angle) + px_noise
            py = (self.circle_radius) * np.sin(angle) + py_noise
            #px = (self.circle_radius) * np.cos(angle) + px_noise
            #py = (self.circle_radius) * np.sin(angle) + py_noise
            collide = False
            for agent in [self.robot] + self.humans:
                min_dist = human.radius + agent.radius + self.discomfort_dist
                if norm((px - agent.px, py - agent.py)) < min_dist or \
                        norm((px - agent.gx, py - agent.gy)) < min_dist:
                    collide = True
                    break
                    # jump out of 'for' loop
            if (px<=self.block_area1[0][0]+human.radius and px>=self.block_area1[2][0]-human.radius) and (py<=self.block_area1[0][1]+human.radius and py>=self.block_area1[2][1]-human.radius):
                collide = True
            if (-px<=self.block_area1[0][0]+human.radius and -px>=self.block_area1[2][0]-human.radius) and (-py<=self.block_area1[0][1]+human.radius and -py>=self.block_area1[2][1]-human.radius):
                collide = True
            if (px<=self.block_area2[0][0]+human.radius and px>=self.block_area2[2][0]-human.radius) and (py<=self.block_area2[0][1]+human.radius and py>=self.block_area2[2][1]-human.radius):
                collide = True
            if (-px<=self.block_area2[0][0]+human.radius and -px>=self.block_area2[2][0]-human.radius) and (-py<=self.block_area2[0][1]+human.radius and -py>=self.block_area2[2][1]-human.radius):
                collide = True
            if not collide:
                break        # jump out of 'while' loop
        human.set(px, py, -px, -py, 0, 0, 0)
        return human

    def generate_square_crossing_human(self):
        human = Human(self.config, 'humans')
        if self.randomize_attributes:
            human.sample_random_attributes()
        sign = 0
        while True:
            #px = float(random.sample((-self.circle_radius-3, self.circle_radius+3),1)[0])
            #py = random.uniform(-self.circle_radius-3 * 1.5 / 2.0, self.circle_radius+3 * 1.5 / 2.0)
            px = float(random.sample((-self.circle_radius, self.circle_radius),1)[0])
            py = random.uniform(-self.circle_radius * 1.5 / 2.0, self.circle_radius * 1.5 / 2.0)
            collide = False
            for agent in [self.robot] + self.humans:
                sign += 1
                if norm((px - agent.px, py - agent.py)) < human.radius + agent.radius + self.discomfort_dist:
                    collide = True
                    break    # jump out of 'for' loop
            if py<=self.block_area1[0][1]+human.radius and py>=self.block_area1[2][1]-human.radius:
                collide = True
            elif py<=self.block_area3[0][1]+human.radius and py>=self.block_area3[2][1]-human.radius:
                collide = True
            if not collide:
                break        # jump out of 'while' loop
        '''while True:
            gx = np.random.random() * self.square_width * 0.5 * -sign
            gy = (np.random.random() - 0.5) * self.square_width
            collide = False
            for agent in [self.robot] + self.humans:
                if norm((gx - agent.gx, gy - agent.gy)) < human.radius + agent.radius + self.discomfort_dist:
                    collide = True
                    break
            if not collide:
                break'''
        if sign % 2 == 1:
            gx = -px
            gy = py
        else:
            temp = px
            px = py
            py = temp
            gx = px
            gy = -py
        human.set(px, py, gx, gy, 0, 0, 0)
        return human

    def get_human_times(self):
        """
        Run the whole simulation to the end and compute the average time for human to reach goal.
        Once an agent reaches the goal, it stops moving and becomes an obstacle
        (doesn't need to take half responsibility to avoid collision).

        :return:
        """
        # centralized orca simulator for all humans
        if not self.robot.reached_destination():
            raise ValueError('Episode is not done yet')
        params = (10, 10, 5, 5)
        sim = rvo2.PyRVOSimulator(self.time_step, params[0],params[1],params[2],params[3], 0.3, 1)
        sim.addAgent(self.robot.get_position(), params[0],params[1],params[2],params[3], self.robot.radius, self.robot.v_pref,
                     self.robot.get_velocity())
        for human in self.humans:
            sim.addAgent(human.get_position(), params[0],params[1],params[2],params[3], human.radius, human.v_pref, human.get_velocity())


        max_time = 1000
        while not all(self.human_times):
            for i, agent in enumerate([self.robot] + self.humans):
                vel_pref = np.array(agent.get_goal_position()) - np.array(agent.get_position())
                if norm(vel_pref) > 1:
                    vel_pref /= norm(vel_pref)
                sim.setAgentPrefVelocity(i, tuple(vel_pref))
            sim.doStep()
            self.global_time += self.time_step
            if self.global_time > max_time:
                logging.warning('Simulation cannot terminate!')
            for i, human in enumerate(self.humans):
                if human.reached_destination():
                    self.human_times[i] = self.global_time
                    """if self.randomize_attributes:
                        goal = human.get_goal_position() # makes humans move continuously
                        new_goal = [-goal[0],-goal[1]]
                        human.set_goal_position(new_goal)
                        self.human_times[i] = 0"""
                    if self.train_val_sim == 'square_crossing' or self.test_sim == 'square_crossing':
                        goal = human.get_goal_position()
                        if goal[0] == self.circle_radius or goal[0] == -self.circle_radius:
                            new_goal = [-goal[0],goal[1]]
                        else:
                            new_goal = [goal[0],-goal[1]]
                        human.set_goal_position(new_goal)
                    elif self.train_val_sim == 'circle_crossing' or self.test_sim == 'circle_crossing':
                        goal = human.get_goal_position()
                        new_goal = [-goal[0],-goal[1]]
                        human.set_goal_position(new_goal)
                    elif self.train_val_sim == 'square_static' or self.test_sim == 'square_static':
                        if i >= 2:
                            goal = human.get_goal_position()
                            if goal[0] == self.circle_radius or goal[0] == -self.circle_radius:
                                new_goal = [-goal[0],goal[1]]
                            else:
                                new_goal = [goal[0],-goal[1]]
                            human.set_goal_position(new_goal)
                    elif self.train_val_sim == 'circle_static' or self.test_sim == 'circle_static':
                        if i >= 2:
                            goal = human.get_goal_position()
                            new_goal = [-goal[0],-goal[1]]
                            human.set_goal_position(new_goal)

            # for visualization
            self.robot.set_position(sim.getAgentPosition(0))
            for i, human in enumerate(self.humans):
                human.set_position(sim.getAgentPosition(i + 1))
            self.states.append([self.robot.get_full_state(), [human.get_full_state() for human in self.humans]])

        del sim
        return self.human_times

    def reset(self, number, phase='test', test_case=None):
        """
        Set px, py, gx, gy, vx, vy, theta for robot and humans
        :return:
        """
        if self.robot is None:
            raise AttributeError('robot has to be set!')
        assert phase in ['train', 'val', 'test']
        if test_case is not None:
            self.case_counter[phase] = test_case
        self.global_time = 0
        if phase == 'test':
            self.human_times = [0] * self.human_num
        else:
            self.human_times = [0] * (self.human_num if self.robot.policy.multiagent_training else 1)
        if not self.robot.policy.multiagent_training:
            self.train_val_sim = 'circle_crossing'
        if number %6 == 0:
            self.train_val_sim = 'circle_crossing'
            self.test_sim = 'circle_crossing'
        elif number %6 == 1:
            self.train_val_sim = 'static'
            self.test_sim = 'static'
        elif number %6 == 2:
            self.train_val_sim = 'no'
            self.test_sim = 'no'
        elif number %6 == 3:
            self.train_val_sim = 'square_crossing'
            self.test_sim = 'square_crossing'
        elif number %6 == 4:
            self.train_val_sim = 'circle_static'
            self.test_sim = 'circle_static'
        elif number %6 == 5:
            self.train_val_sim = 'square_static'
            self.test_sim = 'square_static'
        if self.config.get('humans', 'policy') == 'trajnet':
            raise NotImplementedError
        else:
            counter_offset = {'train': self.case_capacity['val'] + self.case_capacity['test'],
                              'val': 0, 'test': self.case_capacity['val']}
            self.robot.set(0, -4, 0, 4, 0, 0, np.pi / 2)
            if self.case_counter[phase] >= 0:
                #np.random.seed(counter_offset[phase] + self.case_counter[phase]) #training
                #np.random.seed(counter_offset[phase] + self.case_counter[phase]) #testing in the same scenarios 
                #random.seed(counter_offset[phase] + self.case_counter[phase]) #testing in the same scenarios
                if phase in ['train', 'val']:
                    human_num = self.human_num if self.robot.policy.multiagent_training else 1
                    self.generate_random_human_position(human_num=human_num, rule=self.train_val_sim)

                else:
                    self.generate_random_human_position(human_num=self.human_num, rule=self.test_sim)
                # case_counter is always between 0 and case_size[phase]
                self.case_counter[phase] = (self.case_counter[phase] + 1) % self.case_size[phase]
            else:
                assert phase == 'test'
                if self.case_counter[phase] == -1:
                    # for debugging purposes
                    self.human_num = 3
                    self.humans = [Human(self.config, 'humans') for _ in range(self.human_num)]
                    self.humans[0].set(0, -6, 0, 5, 0, 0, np.pi / 2)
                    self.humans[1].set(-5, -5, -5, 5, 0, 0, np.pi / 2)
                    self.humans[2].set(5, -5, 5, 5, 0, 0, np.pi / 2)
                else:
                    raise NotImplementedError

        for agent in [self.robot] + self.humans:
            agent.time_step = self.time_step
            agent.policy.time_step = self.time_step

        self.states = list()
        if hasattr(self.robot.policy, 'action_values'):
            self.action_values = list()
        if hasattr(self.robot.policy, 'get_attention_weights'):
            self.attention_weights = list()

        # get current observation
        if self.robot.sensor == 'coordinates':
            ob = [human.get_observable_state() for human in self.humans]
        elif self.robot.sensor == 'RGB':
            raise NotImplementedError

        return ob

    def onestep_lookahead(self, action):
        return self.step(action, update=False)

    def step(self, action, update=True):
        """
        Compute actions for all agents, detect collision, update environment and return (ob, reward, done, info)

        """
        human_actions = []
        for human in self.humans:
            # observation for humans is always coordinates
            ob = [other_human.get_observable_state() for other_human in self.humans if other_human != human]
            if self.robot.visible:
                ob += [self.robot.get_observable_state()]
            human_actions.append(human.act(ob))
    
        # collision detection
        dmin = float('inf')
        collision = False
        for i, human in enumerate(self.humans):
            px = human.px - self.robot.px
            py = human.py - self.robot.py
            if self.robot.kinematics == 'holonomic':
                vx = human.vx - action.vx
                vy = human.vy - action.vy
            else:
                vx = human.vx - action.v * np.cos(action.r + self.robot.theta)
                vy = human.vy - action.v * np.sin(action.r + self.robot.theta)
            ex = px + vx * self.time_step
            ey = py + vy * self.time_step
            # closest distance between boundaries of two agents
            closest_dist = point_to_segment_dist(px, py, ex, ey, 0, 0) - human.radius - self.robot.radius
            if closest_dist < 0:
                collision = True
                # logging.debug("Collision: distance between robot and p{} is {:.2E}".format(i, closest_dist))
                break
            elif closest_dist < dmin:
                dmin = closest_dist
        collision_wall = False
        if self.robot.kinematics == 'holonomic':
            next_px = self.robot.px + action.vx * self.time_step
            next_py = self.robot.py + action.vy * self.time_step
        else:
            next_px = self.robot.px + action.v * np.cos(action.r + self.robot.theta) * self.time_step
            next_py = self.robot.py + action.v * np.sin(action.r + self.robot.theta) * self.time_step
        if (next_px<self.block_area1[0][0]+self.robot.radius and next_px>self.block_area1[2][0]-self.robot.radius) and (next_py<self.block_area1[0][1]+self.robot.radius and next_py>self.block_area1[2][1]-self.robot.radius):
            collision_wall = True
        if (next_px<self.block_area2[0][0]+self.robot.radius and next_px>self.block_area2[2][0]-self.robot.radius) and (next_py<self.block_area2[0][1]+self.robot.radius and next_py>self.block_area2[2][1]-self.robot.radius):
            collision_wall = True
        if (next_px<self.block_area3[0][0]+self.robot.radius and next_px>self.block_area3[2][0]-self.robot.radius) and (next_py<self.block_area3[0][1]+self.robot.radius and next_py>self.block_area3[2][1]-self.robot.radius):
            collision_wall = True
        if (next_px<self.block_area4[0][0]+self.robot.radius and next_px>self.block_area4[2][0]-self.robot.radius) and (next_py<self.block_area4[0][1]+self.robot.radius and next_py>self.block_area4[2][1]-self.robot.radius):
            collision_wall = True
        # collision detection between humans
        human_num = len(self.humans)
        for i in range(human_num):
            for j in range(i + 1, human_num):
                dx = self.humans[i].px - self.humans[j].px
                dy = self.humans[i].py - self.humans[j].py
                dist = (dx ** 2 + dy ** 2) ** (1 / 2) - self.humans[i].radius - self.humans[j].radius
                if dist < 0:
                    # detect collision but don't take humans' collision into account
                    logging.debug('Collision happens between humans in step()')
        # check if reaching the goal
        end_position = np.array(self.robot.compute_position(action, self.time_step))
        start_dg = norm(self.robot.get_position() - np.array(self.robot.get_goal_position()))
        end_dg = norm(end_position - np.array(self.robot.get_goal_position()))
        reaching_goal = end_dg < self.robot.radius
        position_variation = norm(end_position - self.robot.get_position())
        reward = 0
	if end_dg - start_dg < -0.01:
	    R_forward = 0.01
	else:
	    R_forward = -0.01
	

	#check if robot keeps moving
	if position_variation > 0.03:
	    R_km = 0.01
	else:
	    R_km = -0.01

	# check whether robot is stopped
	is_stopped = False
	R_stop_t = 0
	if self.robot.kinematics == 'holonomic':
	    if norm([action.vx, action.vy]) < 0.01:
		is_stopped = True

        else:
	    if action.v < 0.01:
		is_stopped = True

	
	# calculate the closest distance between robot and humans
	static_dmin = float('inf')
	dynamic_dmin = float('inf')
	R_danger = 0
	R_goal = 0
	R_collision = 0
	R_stop = 0
	dot_prod = 0
	R_col_wall = 0
	timeout = 0
        for i, human in enumerate(self.humans):
	    if (human.vx == 0 and human.vy == 0):
                s_px = human.px - self.robot.px
                s_py = human.py - self.robot.py
                if self.robot.kinematics == 'holonomic':
                    s_vx = human.vx - action.vx
                    s_vy = human.vy - action.vy
                else:
                    s_vx = human.vx - action.v * np.cos(action.r + self.robot.theta)
                    s_vy = human.vy - action.v * np.sin(action.r + self.robot.theta)
                s_ex = s_px + s_vx * self.time_step
                s_ey = s_py + s_vy * self.time_step
                # closest distance between boundaries of two agents
                static_closest = point_to_segment_dist(s_px, s_py, s_ex, s_ey, 0, 0) - human.radius - self.robot.radius
                if static_closest < static_dmin:
                    static_dmin = static_closest
	    else:
		d_px = human.px - self.robot.px
                d_py = human.py - self.robot.py
                if self.robot.kinematics == 'holonomic':
                    d_vx = human.vx - action.vx
                    d_vy = human.vy - action.vy
                else:
                    d_vx = human.vx - action.v * np.cos(action.r + self.robot.theta)
                    d_vy = human.vy - action.v * np.sin(action.r + self.robot.theta)
                d_ex = d_px + d_vx * self.time_step
                d_ey = d_py + d_vy * self.time_step
                # closest distance between boundaries of two agents
                dynamic_closest = point_to_segment_dist(d_px, d_py, d_ex, d_ey, 0, 0) - human.radius - self.robot.radius
                if dynamic_closest < dynamic_dmin:
		    closest_index = i
                    dynamic_dmin = dynamic_closest
	    	clo_rel_po = [self.robot.px - self.humans[closest_index].px, self.robot.py - self.humans[closest_index].py]
	    	clo_rel_ve = [self.robot.vx - self.humans[closest_index].vx, self.robot.vy - self.humans[closest_index].vy]
	    	dot_prod = np.dot(clo_rel_po, clo_rel_ve)

	if static_dmin >= self.R_safe and dynamic_dmin >= self.R_min:
	    R_danger = 0.001
	else:
	    if static_dmin >= self.R_safe and dot_prod < 0:
		R_danger = 0.001
	    else:
		R_danger = -0.001
		if dynamic_dmin < self.R_min and is_stopped:
		    R_stop = 0.1

	min_distance, closest_point = self.compute_min_distance_to_block(self.robot.get_position(), self.blocks)
	min_distance -= self.robot.radius
	R_wall_min_dist = 0
	min_wall_bool = False
	if min_distance < 0.01:
	    R_wall_min_dist = -0.5
	    min_wall_bool = True
	else:
	    R_wall_min_dist = 0.001

	farthest_point, max_distance = self.farthest_point_and_distance(self.robot.get_position(), self.blocks)
	R_wall_max_dist = 0
	if max_distance > 20:
	    R_wall_max_dist = -0.5

	left_path = 0
	position = None
	
	R_end = 0
        if reaching_goal:
            done = True
            info = ReachGoal()
	    R_goal = self.success_reward
	    left_path = 0

	    position = self.robot.get_position()
        elif collision_wall:
            done = True
            R_col_wall = self.collision_wall_penalty
            info = CollisionWall()
	    left_path = end_dg - self.robot.radius
	    R_end = 3 * (1 / (1 + np.exp(3*end_dg - 5.5)))**0.1
	    position = self.robot.get_position()
        elif collision:
            done = True
            info = Collision()
	    R_collision = self.collision_penalty
	    left_path = end_dg - self.robot.radius
	    position = self.robot.get_position()
	    R_end = 3 * (1 / (1 + np.exp(3*end_dg - 5.5)))**0.1
        elif self.global_time >= self.time_limit - 1:
            done = True
            info = Timeout()
	    left_path = end_dg - self.robot.radius
	    timeout = -0.5
	    position = self.robot.get_position()
	    R_end = 3 * (1 / (1 + np.exp(3*end_dg - 5.5)))**0.1
        else:
            done = False
            info = Nothing()
	
	reward = R_danger + R_forward + R_goal + R_collision + R_km + R_col_wall + timeout + R_stop_t + R_wall_min_dist + R_wall_max_dist
	reward_values = {"Total Reward": reward,"R_dan": R_danger, "R_for": R_forward, "R_goal": R_goal,"R_col": R_collision,"R_km": R_km, "R_col_wall": R_col_wall, "timeout": timeout, "R_stop_t": R_stop_t, "R_wall_min_dist": R_wall_min_dist, "R_wall_max_dist": R_wall_max_dist}      
	
        if update:
            # store state, action value and attention weights
            self.states.append([self.robot.get_full_state(), [human.get_full_state() for human in self.humans]])
            if hasattr(self.robot.policy, 'action_values'):
                self.action_values.append(self.robot.policy.action_values)
            if hasattr(self.robot.policy, 'get_attention_weights'):
                self.attention_weights.append(self.robot.policy.get_attention_weights())

            # update all agents
            self.robot.step(action)
            for i, human_action in enumerate(human_actions):
                self.humans[i].step(human_action)
            self.global_time += self.time_step
            for i, human in enumerate(self.humans):
                # only record the first time the human reaches the goal
                if human.reached_destination():
                    self.human_times[i] = self.global_time
                    """if self.randomize_attributes:
                        goal = human.get_goal_position() # makes humans move continuously
                        new_goal = [-goal[0],-goal[1]]
                        human.set_goal_position(new_goal)
                        self.human_times[i] = 0"""
                    if self.train_val_sim == 'square_crossing' or self.test_sim == 'square_crossing':
                        goal = human.get_goal_position()
                        if goal[0] == self.circle_radius or goal[0] == -self.circle_radius:
                            new_goal = [-goal[0],goal[1]]
                        else:
                            new_goal = [goal[0],-goal[1]]
                        human.set_goal_position(new_goal)
                    elif self.train_val_sim == 'circle_crossing' or self.test_sim == 'circle_crossing':
                        goal = human.get_goal_position()
                        new_goal = [-goal[0],-goal[1]]
                        human.set_goal_position(new_goal)
                    elif self.train_val_sim == 'square_static' or self.test_sim == 'square_static':
                        if i >= 2:
                            goal = human.get_goal_position()
                            if goal[0] == self.circle_radius or goal[0] == -self.circle_radius:
                                new_goal = [-goal[0],goal[1]]
                            else:
                                new_goal = [goal[0],-goal[1]]
                            human.set_goal_position(new_goal)
                    elif self.train_val_sim == 'circle_static' or self.test_sim == 'circle_static':
                        if i >= 2:
                            goal = human.get_goal_position()
                            new_goal = [-goal[0],-goal[1]]
                            human.set_goal_position(new_goal)

            # compute the observation
            if self.robot.sensor == 'coordinates':
                ob = [human.get_observable_state() for human in self.humans]
            elif self.robot.sensor == 'RGB':
                raise NotImplementedError
        else:
            if self.robot.sensor == 'coordinates':
                ob = [human.get_next_observable_state(action) for human, action in zip(self.humans, human_actions)]
            elif self.robot.sensor == 'RGB':
                raise NotImplementedError

        
        return ob, reward, done, info, reward_values, left_path, position, is_stopped, min_wall_bool
    

    def render(self, mode='human', output_file=None):
        from matplotlib import animation
        import matplotlib.pyplot as plt
        plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'
        #matplotlib.use("Agg") #download the video
        

        x_offset = 0.11
        y_offset = 0.11
        cmap = plt.cm.get_cmap('hsv', 10)
        robot_color = 'teal'
        goal_color = 'orange'
        arrow_color = 'red'
        arrow_style = patches.ArrowStyle("->", head_length=2, head_width=2)

        if mode == 'human':
            fig, ax = plt.subplots(figsize=(7, 7))
            ax.set_xlim(-4, 4)
            ax.set_ylim(-4, 4)
            for human in self.humans:
                human_circle = plt.Circle(human.get_position(), human.radius, fill=False, color='b')
                ax.add_artist(human_circle)
            ax.add_artist(plt.Circle(self.robot.get_position(), self.robot.radius, fill=True, color='r'))
            plt.show()
        elif mode == 'traj':
            fig, ax = plt.subplots(figsize=(7, 7))
            ax.tick_params(labelsize=16)
            ax.set_xlim(-5, 5)
            ax.set_ylim(-5, 5)
            ax.set_xlabel('x(m)', fontsize=16)
            ax.set_ylabel('y(m)', fontsize=16)

            robot_positions = [self.states[i][0].position for i in range(len(self.states))]
            human_positions = [[self.states[i][1][j].position for j in range(len(self.humans))]
                               for i in range(len(self.states))]
            for k in range(len(self.states)):
                if k % 4 == 0 or k == len(self.states) - 1:
                    robot = plt.Circle(robot_positions[k], self.robot.radius, fill=True, color=robot_color)
                    humans = [plt.Circle(human_positions[k][i], self.humans[i].radius, fill=False, color=cmap(i))
                              for i in range(len(self.humans))]
                    ax.add_artist(robot)
                    for human in humans:
                        ax.add_artist(human)
                # add time annotation
                global_time = k * self.time_step
                if global_time % 4 == 0 or k == len(self.states) - 1:
                    agents = humans + [robot]
                    times = [plt.text(agents[i].center[0] - x_offset, agents[i].center[1] - y_offset,
                                      '{:.1f}'.format(global_time),
                                      color='black', fontsize=14) for i in range(self.human_num + 1)]
                    for time in times:
                        ax.add_artist(time)
                if k != 0:
                    nav_direction = plt.Line2D((self.states[k - 1][0].px, self.states[k][0].px),
                                               (self.states[k - 1][0].py, self.states[k][0].py),
                                               color=robot_color, ls='solid')
                    human_directions = [plt.Line2D((self.states[k - 1][1][i].px, self.states[k][1][i].px),
                                                   (self.states[k - 1][1][i].py, self.states[k][1][i].py),
                                                   color=cmap(i), ls='solid')
                                        for i in range(self.human_num)]
                    ax.add_artist(nav_direction)
                    for human_direction in human_directions:
                        ax.add_artist(human_direction)
            plt.legend([robot], ['Robot'], fontsize=16)
            plt.show()
        elif mode == 'video':
            fig, ax = plt.subplots(figsize=(7, 7))
            ax.tick_params(labelsize=16)
            ax.set_xlim(-6, 6) # NABIH map size
            ax.set_ylim(-6, 6)  # NABIH map size
            ax.set_xlabel('x position (m)', fontsize=16)  # NABIH legend x
            ax.set_ylabel('y position (m)', fontsize=16)  # NABIH legend y
            # add robot and its goal
            robot_positions = [state[0].position for state in self.states]
            # draw a star at the goal position (0,4)
            goal = mlines.Line2D([0], [4], color=goal_color, marker='*', linestyle='None', markersize=15, label='Goal')
            robot_legend = mlines.Line2D([0], [4], color=robot_color, marker='o', linestyle='None', markersize=15, label='Goal') # NABIH marker
            robot = plt.Circle(robot_positions[0], self.robot.radius, fill=False, color=robot_color)
            ax.add_artist(robot)
            ax.add_artist(goal)
            plt.legend([robot_legend, goal], ['Robot', 'Goal'], fontsize=16, numpoints=1)   # numpoints=1: only 1 star in the legend

            # add humans and their numbers
            human_positions = [[state[1][j].position for j in range(len(self.humans))] for state in self.states]
            humans = [plt.Circle(human_positions[0][i], self.humans[i].radius, fill=False)
                      for i in range(len(self.humans))]
            human_numbers = [plt.text(humans[i].center[0] - x_offset, humans[i].center[1] - y_offset, str(i),
                                      color='green', fontsize=12) for i in range(len(self.humans))] # nabih human number colors
            for i, human in enumerate(humans):
                ax.add_artist(human)
                ax.add_artist(human_numbers[i])
            for i in range(len(self.block_area1)):
                plt.plot([self.block_area1[i][0],self.block_area1[(i+1)%len(self.block_area1)][0]],[self.block_area1[i][1],self.block_area1[(i+1)%len(self.block_area1)][1]], color = 'b')
            for i in range(len(self.block_area2)):
                plt.plot([self.block_area2[i][0],self.block_area2[(i+1)%len(self.block_area2)][0]],[self.block_area2[i][1],self.block_area2[(i+1)%len(self.block_area2)][1]], color = 'b')
            for i in range(len(self.block_area3)):
                plt.plot([self.block_area3[i][0],self.block_area3[(i+1)%len(self.block_area3)][0]],[self.block_area3[i][1],self.block_area3[(i+1)%len(self.block_area3)][1]], color = 'b')
            for i in range(len(self.block_area2)):
                plt.plot([self.block_area4[i][0],self.block_area4[(i+1)%len(self.block_area4)][0]],[self.block_area4[i][1],self.block_area4[(i+1)%len(self.block_area4)][1]], color = 'b')
            
            
            


            # add time annotation
            time = plt.text(-1.5, 5, 'Time: {}'.format(0), fontsize=16) #nabih modify text time
            ax.add_artist(time)

            # compute attention scores
            if self.attention_weights is not None:
                attention_scores = [
                    plt.text(-11.5, 11 - 1.0 * i, 'Human {}: {:.2f}'.format(i + 1, self.attention_weights[0][i]),
                             fontsize=16) for i in range(len(self.humans))]

            # compute orientation in each step and use arrow to show the direction
            radius = self.robot.radius
            if self.robot.kinematics == 'unicycle':
                orientation = [((state[0].px, state[0].py), (state[0].px + radius * np.cos(state[0].theta),
                                                             state[0].py + radius * np.sin(state[0].theta))) for state
                               in self.states]
                orientations = [orientation]
            else:
                orientations = []
                for i in range(self.human_num + 1):
                    orientation = []
                    for state in self.states:
                        if i == 0:
                            agent_state = state[0]
                        else:
                            agent_state = state[1][i - 1]
                        theta = np.arctan2(agent_state.vy, agent_state.vx)
                        orientation.append(((agent_state.px, agent_state.py), (agent_state.px + radius * np.cos(theta),
                                             agent_state.py + radius * np.sin(theta))))
                    orientations.append(orientation)
            arrows = {}
            arrows[0] = [patches.FancyArrowPatch(*orientation[0], color=arrow_color, arrowstyle=arrow_style)
                      for orientation in orientations]
            for arrow in arrows[0]:
                ax.add_artist(arrow)
            global_step = {}
            global_step[0] = 0

            def update(frame_num):
                # nonlocal global_step
                # nonlocal arrows
                global_step[0] = frame_num
                robot.center = robot_positions[frame_num]
                for i, human in enumerate(humans):
                    human.center = human_positions[frame_num][i]
                    human_numbers[i].set_position((human.center[0] - x_offset, human.center[1] - y_offset))
                    for arrow in arrows[0]:
                        arrow.remove()
                    arrows[0] = [patches.FancyArrowPatch(*orientation[frame_num], color=arrow_color,
                                                      arrowstyle=arrow_style) for orientation in orientations]
                    for arrow in arrows[0]:
                        ax.add_artist(arrow)
                    if self.attention_weights is not None:
                        human.set_color(str(self.attention_weights[frame_num][i]))
                        attention_scores[i].set_text('human {}: {:.2f}'.format(i, self.attention_weights[frame_num][i]))
            
                human_traj_update = [mlines.Line2D([(human_positions[frame_num][i])[0]], [(human_positions[frame_num][i])[1]], color=cmap(i), marker='.', linestyle='None', markersize=4) for i in range(len(self.humans))]     #nabih human trajectory
                robot_traj_update = [mlines.Line2D([robot.center[0]], [robot.center[1]], color='C4', marker='.', linestyle='None', markersize=4)] # nabih robot trajectory
                for trajec in human_traj_update:
                    ax.add_artist(trajec)           #nabih human trajectory
                for r_trajec in robot_traj_update: 
            
                    ax.add_artist(r_trajec)         # nabih robot trajectory
            
                time.set_text('Time: {:.2f}'.format(frame_num * self.time_step))

            def plot_value_heatmap():
                assert self.robot.kinematics == 'holonomic'
                # when any key is pressed draw the action value plot
                fig, axis = plt.subplots()
                speeds = self.robot.policy.speeds
                rotations = self.robot.policy.rotations + [np.pi * 2]
                r, th = np.meshgrid(speeds, rotations)
                z = np.array(self.action_values[global_step[0] % len(self.states)][1:])
                z = (z - np.min(z)) / (np.max(z) - np.min(z))  # z: normalized action values
                z = np.append(z, z[:6])
                z = z.reshape(16, 6)  # rotations: 16   speeds:6
                polar = plt.subplot(projection="polar")
                polar.tick_params(labelsize=16)
                mesh = plt.pcolormesh(th, r, z, cmap=plt.cm.viridis)
                plt.plot(rotations, r, color='k', ls='none')
                plt.grid()
                cbaxes = fig.add_axes([0.85, 0.1, 0.03, 0.8])
                cbar = plt.colorbar(mesh, cax=cbaxes)
                cbar.ax.tick_params(labelsize=16)
                plt.show()

            def on_click(event):
                anim.running ^= True
                if anim.running:
                    anim.event_source.stop()
                    if hasattr(self.robot.policy, 'action_values'):
                        plot_value_heatmap()
                else:
                    anim.event_source.start()

            fig.canvas.mpl_connect('key_press_event', on_click)
            anim = animation.FuncAnimation(fig, update, frames=len(self.states), interval=self.time_step * 1000)
            anim.running = True
            anim.save('testcase_animation.gif', writer='imagemagick')

            if output_file is not None:
                ffmpeg_writer = animation.writers['ffmpeg']
                writer = ffmpeg_writer(fps=8, metadata=dict(artist='Me'), bitrate=1800)
                anim.save(output_file, writer=writer)
            else:
                plt.show()
        else:
            raise NotImplementedError
