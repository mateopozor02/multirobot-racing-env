from __future__ import division
import numpy as np
import rvo2
from crowd_sim.envs.policy.policy import Policy
from crowd_sim.envs.utils.action import ActionXY
import logging
import math


class ORCA(Policy):
    def __init__(self):
        """
        timeStep        The time step of the simulation.
                        Must be positive.
        neighborDist    The default maximum distance (center point
                        to center point) to other agents a new agent
                        takes into account in the navigation. The
                        larger this number, the longer the running
                        time of the simulation. If the number is too
                        low, the simulation will not be safe. Must be
                        non-negative.
        maxNeighbors    The default maximum number of other agents a
                        new agent takes into account in the
                        navigation. The larger this number, the
                        longer the running time of the simulation.
                        If the number is too low, the simulation
                        will not be safe.
        timeHorizon     The default minimal amount of time for which
                        a new agent's velocities that are computed
                        by the simulation are safe with respect to
                        other agents. The larger this number, the
                        sooner an agent will respond to the presence
                        of other agents, but the less freedom the
                        agent has in choosing its velocities.
                        Must be positive.
        timeHorizonObst The default minimal amount of time for which
                        a new agent's velocities that are computed
                        by the simulation are safe with respect to
                        obstacles. The larger this number, the
                        sooner an agent will respond to the presence
                        of obstacles, but the less freedom the agent
                        has in choosing its velocities.
                        Must be positive.
        radius          The default radius of a new agent.
                        Must be non-negative.
        maxSpeed        The default maximum speed of a new agent.
                        Must be non-negative.
        velocity        The default initial two-dimensional linear
                        velocity of a new agent (optional).

        ORCA first uses neighborDist and maxNeighbors to find neighbors that need to be taken into account.
        Here set them to be large enough so that all agents will be considered as neighbors.
        Time_horizon should be set that at least it's safe for one time step

        In this work, obstacles are not considered. So the value of time_horizon_obst doesn't matter.

        """
        super(ORCA, self).__init__()
        self.name = 'ORCA'
        self.trainable = False
        self.multiagent_training = None
        self.kinematics = 'holonomic'
        self.safety_space = 0
        self.neighbor_dist = 10
        self.max_neighbors = 10
        self.time_horizon = 5
        self.time_horizon_obst = 5
        self.radius = 0.3
        self.max_speed = 5.5
        self.sim = None

    def configure(self, config):
        # self.time_step = config.getfloat('orca', 'time_step')
        # self.neighbor_dist = config.getfloat('orca', 'neighbor_dist')
        # self.max_neighbors = config.getint('orca', 'max_neighbors')
        # self.time_horizon = config.getfloat('orca', 'time_horizon')
        # self.time_horizon_obst = config.getfloat('orca', 'time_horizon_obst')
        # self.radius = config.getfloat('orca', 'radius')
        # self.max_speed = config.getfloat('orca', 'max_speed')
        return

    def set_phase(self, phase):
        return

    def predict(self, state):
        """
        Create a rvo2 simulation at each time step and run one step
        Python-RVO2 API: https://github.com/sybrenstuvel/Python-RVO2/blob/master/src/rvo2.pyx
        How simulation is done in RVO2: https://github.com/sybrenstuvel/Python-RVO2/blob/master/src/Agent.cpp

        Agent doesn't stop moving after it reaches the goal, because once it stops moving, the reciprocal rule is broken

        :param state:
        :return:
        """
        self_state = state.self_state
        params = self.neighbor_dist, self.max_neighbors, self.time_horizon, self.time_horizon_obst
        if self.sim is not None and self.sim.getNumAgents() != len(state.human_states) + 1:
            del self.sim
            self.sim = None
        if self.sim is None:
            self.sim = rvo2.PyRVOSimulator(self.time_step, params[0],params[1],params[2],params[3], self.radius, self.max_speed)
            self.sim.addAgent(self_state.position, params[0],params[1],params[2],params[3], self_state.radius + 0.01 + self.safety_space,
                              self_state.v_pref, self_state.velocity)
            for human_state in state.human_states:
                self.sim.addAgent(human_state.position, params[0],params[1],params[2],params[3], human_state.radius + 0.01 + self.safety_space,
                                  self.max_speed, human_state.velocity)

            # @lky add the obstacles
            '''self.sim.addObstacle([[2,9],[-2,9],[-2,5],[2,5]])
            self.sim.addObstacle([[2.5,2.5],[-2.5,2.5],[-2.5,-2.5],[2.5,-2.5]])
            self.sim.addObstacle([[2,-5],[-2,-5],[-2,-9],[2,-9]])
            self.sim.addObstacle([[-5,2],[-9,2],[-9,-2],[-5,-2]])'''

            #Get rectangle obstacles from file 
            #rectangle_obstacles = []
            #with open('rectangle_obstacles.txt', 'r') as file:
            #    for line in file:
            #        x1, y1, x2, y2, x3, y3, x4, y4 = line.split() 
            #        rectangle_obstacles.append([(float(x1), float(y1)), (float(x2), float(y2)), (float(x3), float(y3)), (float(x4), float(y4))])
            #file.close()

            #Add rectangle obstacles to the simulation
            #for rectangle_obstacle in rectangle_obstacles:
            #    self.sim.addObstacle(rectangle_obstacle)
            #self.sim.processObstacles()

            circle_center = (8, -8)
            circle_radius1 = 10.0
            circle_radius2 = 16.6
            circle_limits1 = [(-8.6, -8), (-2, -8)]
            circle_limits2 = [(18, -8), (24.59, -8.1)]
        
            num_segments = 30  
            for circle_radius in [circle_radius1, circle_radius2]:
                for i in range(num_segments):
                    angle = 2 * math.pi * i / num_segments
                    next_angle = 2 * math.pi * (i + 1) / num_segments

                    x1 = circle_center[0] + circle_radius * math.cos(angle)
                    y1 = circle_center[1] + circle_radius * math.sin(angle)
                    x2 = circle_center[0] + circle_radius * math.cos(next_angle)
                    y2 = circle_center[1] + circle_radius * math.sin(next_angle)

                    self.sim.addObstacle([(x1, y1), (x2, y2)])
                self.sim.processObstacles()

            self.sim.addObstacle(circle_limits1)
            self.sim.addObstacle(circle_limits2)
            self.sim.processObstacles()

        else:
            self.sim.setAgentPosition(0, self_state.position)
            self.sim.setAgentVelocity(0, self_state.velocity)
            for i, human_state in enumerate(state.human_states):
                self.sim.setAgentPosition(i + 1, human_state.position)
                self.sim.setAgentVelocity(i + 1, human_state.velocity)

        # Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        velocity = np.array((self_state.gx - self_state.px, self_state.gy - self_state.py))
        speed = np.linalg.norm(velocity)
        human_vmax = 3
        if speed != 0:
            # human_vmax = np.random.uniform(0, 1.5)
            pref_vel = human_vmax * (velocity / speed)
        else:
            pref_vel = np.array([0,0])

        # Perturb a little to avoid deadlocks due to perfect symmetry.
        # perturb_angle = np.random.random() * 2 * np.pi
        # perturb_dist = np.random.random() * 0.01
        # perturb_vel = np.array((np.cos(perturb_angle), np.sin(perturb_angle))) * perturb_dist
        # pref_vel += perturb_vel

        self.sim.setAgentPrefVelocity(0, tuple(pref_vel))
        for i, human_state in enumerate(state.human_states):
            # unknown goal position of other humans
            self.sim.setAgentPrefVelocity(i + 1, (0, 0))

        self.sim.doStep()
        action = ActionXY(*self.sim.getAgentVelocity(0))
        self.last_state = state

        return action
