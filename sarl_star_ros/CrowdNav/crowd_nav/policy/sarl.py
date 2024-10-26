# Author: Changan Chen <changanvr@gmail.com>
# Modified by: Keyu Li <kyli@link.cuhk.edu.hk>

from __future__ import division
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.nn.functional import softmax
import logging
from crowd_sim.envs.utils.action import ActionRot, ActionXY
from crowd_nav.policy.cadrl import mlp
from crowd_nav.policy.multi_human_rl import MultiHumanRL
import threading
import concurrent.futures
import pickle


class ValueNetwork(nn.Module):
    def __init__(self, input_dim, self_state_dim, mlp1_dims, mlp2_dims, mlp3_dims, attention_dims, with_global_state,
                 cell_size, cell_num):
        super(ValueNetwork, self).__init__()
        self.self_state_dim = self_state_dim
        self.global_state_dim = mlp1_dims[-1]
        self.mlp1 = mlp(input_dim, mlp1_dims, last_relu=True)
        self.mlp2 = mlp(mlp1_dims[-1], mlp2_dims)
        self.with_global_state = with_global_state
        if with_global_state:
            self.attention = mlp(mlp1_dims[-1] * 2, attention_dims)
        else:
            self.attention = mlp(mlp1_dims[-1], attention_dims)
        self.cell_size = cell_size
        self.cell_num = cell_num
        mlp3_input_dim = mlp2_dims[-1] + self.self_state_dim
        self.mlp3 = mlp(mlp3_input_dim, mlp3_dims)
        self.attention_weights = None

    def forward(self, state):
        """
        First transform the world coordinates to self-centric coordinates and then do forward computation
        :param state: tensor of shape (batch_size, # of humans, length of a rotated state)
        :return:
        """
        size = state.shape
        self_state = state[:, 0, :self.self_state_dim]
        mlp1_output = self.mlp1(state.view((-1, size[2])))
        mlp2_output = self.mlp2(mlp1_output)

        if self.with_global_state:
            # compute attention scores
            global_state = torch.mean(mlp1_output.view(size[0], size[1], -1), 1, keepdim=True)
            global_state = global_state.expand((size[0], size[1], self.global_state_dim)).\
                contiguous().view(-1, self.global_state_dim)
            attention_input = torch.cat([mlp1_output, global_state], dim=1)
        else:
            attention_input = mlp1_output
        scores = self.attention(attention_input).view(size[0], size[1], 1).squeeze(dim=2)  

        # masked softmax
        # weights = softmax(scores, dim=1).unsqueeze(2)
        scores_exp = torch.exp(scores) * (scores != 0).float()
        weights = (scores_exp / torch.sum(scores_exp, dim=1, keepdim=True)).unsqueeze(2)
        self.attention_weights = weights[0, :, 0].data.cpu().numpy()

        # output feature is a linear combination of input features
        features = mlp2_output.view(size[0], size[1], -1)
        # for converting to onnx
        # expanded_weights = torch.cat([torch.zeros(weights.size()).copy_(weights) for _ in range(50)], dim=2)
        weighted_feature = torch.sum(torch.mul(weights, features), dim=1)

        # concatenate agent's state with global weighted humans' state
        joint_state = torch.cat([self_state, weighted_feature], dim=1)
        value = self.mlp3(joint_state)
        return value


class SARL(MultiHumanRL): 
    def __init__(self):
        super(SARL, self).__init__() 
        self.name = 'SARL'
        self.with_costmap = False
        self.gc = None
        self.gc_resolution = None
        self.gc_width = None
        self.gc_ox = None
        self.gc_oy = None
        self.max_value_list = list() ###########
        self.global_time = 0
        self.obstacle_coords = self.get_obstacle_coords()
        #Store the rewards 
        self.reward = 0 
        self.r_wall_min_dist = 0 
        self.r_km = 0
        self.r_for = 0 
        self.r_stop_t = 0
        self.r_danger = 0 
        self.r_forward = 0
        self.r_collision = 0
        self.r_col_wall = 0
        self.r_goal = 0
        self.goal_reward = 0 
        #Variable to store the accumulated reward 
        self.accumulated_reward = 0 
        #Counter to check if the robot reached the goal
        self.goal_reached_counter = 0

        self.reward_list = list()

       
    

    def configure(self, config):
        self.set_common_parameters(config)
        mlp1_dims = [int(x) for x in config.get('sarl', 'mlp1_dims').split(', ')]
        mlp2_dims = [int(x) for x in config.get('sarl', 'mlp2_dims').split(', ')]
        mlp3_dims = [int(x) for x in config.get('sarl', 'mlp3_dims').split(', ')]
        attention_dims = [int(x) for x in config.get('sarl', 'attention_dims').split(', ')]
        self.with_om = config.getboolean('sarl', 'with_om')
        with_global_state = config.getboolean('sarl', 'with_global_state')
        self.model = ValueNetwork(self.input_dim(), self.self_state_dim, mlp1_dims, mlp2_dims, mlp3_dims,
                                  attention_dims, with_global_state, self.cell_size, self.cell_num)
        self.multiagent_training = config.getboolean('sarl', 'multiagent_training')
        if self.with_om:
            self.name = 'OM-SARL'
        logging.info('Policy: {} {} global state'.format(self.name, 'w/' if with_global_state else 'w/o'))

    def write_reward_to_file(self):
        with open('/home/heisenberg0702/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/policy/reward.pkl', 'wb') as f:
            pickle.dump(self.reward, f)

    def get_obstacle_coords(self):
        with open('/home/heisenberg0702/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/policy/obstacle_coords_meters.pkl', 'rb') as f:
            obstacle_coords = pickle.load(f)

        #Invert the y coordinates 
        obstacle_x_meters = [coord[0] for coord in obstacle_coords]
        obstacle_y_meters = [-coord[1] for coord in obstacle_coords]

        obstacle_coords = list(zip(obstacle_x_meters, obstacle_y_meters))
        
        return obstacle_coords
    
    def plot_obstacles_and_robot(self, obstacle_coords, nav):
        # Initialize the plot
        fig, ax = plt.subplots(figsize=(6, 12))

        # Plot the obstacles
        # Get the X and Y coordinates of the obstacles
        obstacle_x = [coord[0] for coord in obstacle_coords]
        obstacle_y = [coord[1] for coord in obstacle_coords]

        # Plot the obstacles
        ax.scatter(obstacle_x, obstacle_y, color='blue', marker='.')

        # Plot the robot as an unfilled circle
        robot = plt.Circle((nav.px, nav.py), nav.radius, color='red', fill=False)

        # Add the robot to the plot
        ax.add_artist(robot)
        ax.grid(True)
        ax.set_aspect('equal', adjustable='box')

        plt.tight_layout()

        # Show the plot
        plt.show()

    # predict the cost that the robot hits the static obstacles in the global map
    def compute_cost(self, state):
        costs = []
        x = state.px
        y = state.py
        min_x = x - 0.01
        min_y = y - 0.01
        max_x = x + 0.01
        max_y = y + 0.01
        grid_min_x = int(round((min_x - self.gc_ox) / self.gc_resolution))
        grid_min_y = int(round((min_y - self.gc_oy) / self.gc_resolution))
        grid_max_x = int(round((max_x - self.gc_ox) / self.gc_resolution))
        grid_max_y = int(round((max_y - self.gc_oy) / self.gc_resolution))
        for i in range(grid_min_x, grid_max_x+1):
            for j in range(grid_min_y, grid_max_y + 1):
                index = i + self.gc_width * j
                costs.append(self.gc[index])
        max_cost = max(costs)
        return max_cost


    def predict(self, state):
        """
        Takes pairwise joint state as input to value network and output action.
        The input to the value network is always of shape (batch_size, # humans, rotated joint state length).
        If with_costmap is True, the dangerous actions predicted by the value network will be screened out to avoid static obstacles on the map.
        """
        if self.phase is None or self.device is None:
            raise AttributeError('Phase, device attributes have to be set!')
        if self.phase == 'train' and self.epsilon is None:
            raise AttributeError('Epsilon attribute has to be set in training phase')

        if self.reach_destination(state):
            return ActionXY(0, 0) if self.kinematics == 'holonomic' else ActionRot(0, 0)
        if self.action_space is None:
            self.build_action_space(state.self_state.v_pref)

        occupancy_maps = None
        probability = np.random.random()
        if self.phase == 'train' and probability < self.epsilon:
            max_action = self.action_space[np.random.choice(len(self.action_space))]
        else:
            self.action_values = list()
            max_value = float('-inf')
            max_action = None
            '''#####################
            reward_dict = dict()
            value_dict = dict()
            r_list = []
            first_passed = False
            '''#####################
            for action in self.action_space:
                next_self_state = self.propagate(state.self_state, action)
                next_self_state_further = self.propagate_more(state.self_state, action)

                # abort actions which will probably cause collision with static obstacles in the costmap         
                if self.with_costmap is True:
                    cost = self.compute_cost(next_self_state_further)
                    if cost > 0:
                        print("********** Abort action:", action, " with cost:", cost, " that will hit the obstacles.")
                        continue
                
            
                if self.query_env:
                    next_human_states, self.reward, done, info, _, _,_,_ = self.env.onestep_lookahead(action)
                else:
                    next_human_states = [self.propagate(human_state, ActionXY(human_state.vx, human_state.vy))
                                       for human_state in state.human_states]
                    self.reward = self.compute_reward(next_self_state, next_human_states)
                    #self.reward = self.compute_reward_dg(state.self_state, next_self_state, next_human_states)
                    #self.plot_obstacles_and_robot(self.obstacle_coords, next_self_state)
                    #self.accumulated_reward += self.reward
            
                batch_next_states = torch.cat([torch.Tensor([next_self_state + next_human_state]).to(self.device)
                                                   for next_human_state in next_human_states], dim=0)
                rotated_batch_input = self.rotate(batch_next_states).unsqueeze(0)

                if self.with_om:
                    if occupancy_maps is None:
                        occupancy_maps = self.build_occupancy_maps(next_human_states).unsqueeze(0)
                    rotated_batch_input = torch.cat([rotated_batch_input, occupancy_maps], dim=2)
                # VALUE UPDATE
                next_state_value = self.model(rotated_batch_input).data.item()
                # Equation (5)
                value = self.reward + pow(self.gamma, self.time_step * state.self_state.v_pref) * next_state_value
                self.action_values.append(value)
                if value > max_value:
                    max_value = value
                    max_action = action
                    #print("********** choose action:", action)
                    #print("********** cost:", cost)
                '''#######################################
                if action.v != 0 or action.r != 0:
                    first_passed = True
                if not action.v in reward_dict:
                    reward_dict[action.v] = []
                    value_dict[action.v] = []
                if first_passed:
                    reward_dict[action.v].append(reward)
                    value_dict[action.v].append(value)
                    if not action.r in r_list:
                        r_list.append(action.r)

            if reward_dict and value_dict:
                #print('--------------------------reward_dict-----------------------------')
                #print(reward_dict)
                #print('--------------------------r_list-----------------------------')
                #print(r_list)
                reward_pd = pd.DataFrame(reward_dict, index=r_list)
                value_pd = pd.DataFrame(value_dict, index=r_list)

                print('------------------------------reward---------------------------------')
                print(reward_pd)
                print('----------------------------total value------------------------------')
                #print(value_pd)
                print("Chosen action:", max_action)
                print("Max value:", max_value)
                '''###########################################
            if max_action is None:
                # if the robot is trapped, choose the turning action to escape
                max_action = ActionRot(0, 0.78)
                print("The robot is trapped. Rotate in place to escape......")

        if self.phase == 'train':
            self.last_state = self.transform(state)

        '''####################################
        self.max_value_list.append(max_value)
        print(self.max_value_list)
        '''####################################
            
        return max_action

    def compute_reward(self, nav, humans):
        # collision detection
        dmin = float('inf')
        collision = False
        if len(humans):
            for i, human in enumerate(humans):
                dist = np.linalg.norm((nav.px - human.px, nav.py - human.py)) - nav.radius - human.radius
                if dist < 0:
                    collision = True
                    break
                if dist < dmin:
                    dmin = dist

        # check if reaching the goal
        dg = np.linalg.norm((nav.px - nav.gx, nav.py - nav.gy))
        reaching_goal = dg < nav.radius

        if collision:
            reward = self.env.collision_penalty
            #print("Collision detected")
        elif reaching_goal:
            reward = 1
        elif dmin < self.env.discomfort_dist:
            reward = (dmin - self.env.discomfort_dist) * self.env.discomfort_penalty_factor * self.env.time_step
        else:
            #reward = 0
            reward = 1 - 0.32*dg*dg # function 1
            # reward = 1 - 0.16*dg*dg # function 2
            # reward = 1 / (1 + np.exp(dg - 3)) #function 3
            #reward = 1 / (1 + np.exp(2.5*dg - 4)) #function 4
            # reward = 1 / (1 + np.exp(dg - 4)) #function 5
        #print("Position in SARL: ", nav.px, nav.py)
        #print("Goal in SARL: ", nav.gx, nav.gy)
        #print("End distance to goal: ", dg)
        #print("Calculated reward: ", reward)

        return reward

    def compute_reward_dg(self, prev_nav, nav, humans):
        """
        if len(humans):
            for i in range(len(humans)):
                print('Printing human number: ', i)
                print(humans[i])
            #print("Humans detected")
            #print(len(humans))  
            #print(type(humans))
        """
        #Reset the reward variables 
        self.reward = 0 
        self.r_wall_min_dist = 0 
        self.r_km = 0
        self.r_for = 0 
        self.r_stop_t = 0
        self.r_danger = 0 
        self.r_forward = 0
        self.r_collision = 0
        self.r_col_wall = 0
        self.r_goal = 0
        self.goal_reward = 0 

        # collision detection
        dmin = float('inf')
        collision = False
        if len(humans):
            for i, human in enumerate(humans):
                #print("Human in SARL code", str(human.px), str(human.py))
                dist = np.linalg.norm((nav.px - human.px, nav.py - human.py)) - nav.radius - human.radius
                if dist < 0:
                    collision = True
                    break
                if dist < dmin:
                    dmin = dist
        

        #Do collision wall and min distance to wall in the same loop
        
        collision_wall = False
        R_wall_min_dist = 0
        min_dist = float('inf')
        
        for i, obstacle in enumerate(self.obstacle_coords):
            #Adjust distance by adding a tolerance of 0.1
            dist = np.linalg.norm((nav.px - obstacle[0], nav.py - obstacle[1])) - nav.radius - 0.3
            if dist <= 0:
                collision_wall = True
            if dist < min_dist:
                min_dist = dist
        
        #dist < 0.1
        if min_dist < 0.2:
            R_wall_min_dist = -0.1
        else:
            R_wall_min_dist = 0.0025
        
        

        # compute dg at time t and t + 1
        start_dg = np.linalg.norm((prev_nav.px - prev_nav.gx, prev_nav.py - prev_nav.gy))
        end_dg = np.linalg.norm((nav.px - nav.gx, nav.py - nav.gy))

        # check if reaching the goal and adjust to goal tolerance
        reaching_goal = end_dg < nav.radius + 0.35
    
        reward = 0

        R_danger = 0 
        R_goal = 0
        R_collision = 0
        R_col_wall = 0

        #Calculate closest distance between robot and humans 
        static_dmin = float('inf')
        dynamic_dmin = float('inf')
        if len(humans):
            for i, human in enumerate(humans):
                dist = np.linalg.norm((nav.px - human.px, nav.py - human.py)) - nav.radius - human.radius
                if (human.vx and human.vy == 0):
                    if dist < static_dmin:
                        static_dmin = dist
                else:
                    if dist < dynamic_dmin:
                        closest_index = i
                        dynamic_dmin = dist
                    clo_rel_po = [nav.px - humans[closest_index].px, nav.py - humans[closest_index].py]
                    clo_rel_ve = [nav.vx - humans[closest_index].vx, nav.vy - humans[closest_index].vy]
                    dot_product = np.dot(clo_rel_po, clo_rel_ve)

        #Check if the robot is stopped 
        is_stopped = False
        R_stop_t = 0 
        if np.linalg.norm([nav.vx, nav.vy]) < 0.01:
            is_stopped = True
            R_stop_t = -0.01

        #Define R_danger 
        R_safe = 0.1
        R_min = 0.3

        if static_dmin >= R_safe and dynamic_dmin >= R_min:
            R_danger = 0.001
        else:
            if static_dmin >= R_safe and dot_product < 0:
                R_danger = 0.001
            else:
                R_danger = -0.001 

        # check if timeout
        Timeout = False
        self.global_time += self.time_step
        if self.global_time >= self.env.time_limit - 1 :
            Timeout = True

        #Define R_forward 
        R_forward = 0
        #Before -0.1
        if end_dg - start_dg < -0.001:
            R_forward = 0.01
        else:
            R_forward = -0.01

        #print("Variation:", end_dg - start_dg)

        #Define R_km
        position_variation = np.linalg.norm([nav.px - prev_nav.px, nav.py - prev_nav.py])
        if position_variation > 0.03:
            R_km = 0.01
        else:
            R_km = -0.01
        
        
        R_for = 0 
        if reaching_goal:
            #R_goal = self.env.success_reward 
            #self.plot_obstacles_and_robot(self.obstacle_coords, nav)
            R_goal = 10 
            self.goal_reward = self.env.success_reward
            self.goal_reached_counter += 1

            #print('Accumulated Reward: ', self.accumulated_reward)

            #save the reward_list sum to a file
            #reward_sum = sum(self.reward_list)
            #with open('/home/heisenberg0702/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/reward_sum.txt', 'a') as f:
            #    f.write(str(reward_sum) + '\n')
            #print("Goal Reached")
        elif collision_wall:
            R_col_wall = self.env.collision_wall_penalty
        #    print("Collision with wall detected")
        #    print("Collision with wall penalty, ", self.env.collision_wall_penalty)

        elif collision:
            print("Collision penalty, ", self.env.collision_penalty)
            R_collision = self.env.collision_penalty

        else:
            R_for = 1 - 0.32*end_dg*end_dg # function 1


        if self.goal_reached_counter > 1:
            R_goal = 0
        #reward = R_danger + R_goal + R_km + R_collision + R_col_wall + R_stop_t + R_wall_min_dist + R_for
        #reward = R_danger + R_forward + R_goal + R_km + R_collision + R_col_wall + R_stop_t + R_wall_min_dist
        reward = R_forward + R_km
        #reward = R_forward + R_km
        #print("Reward: ", reward)
        #print("R_goal: ", R_goal)

        #Store the rewards in the class variables
        self.r_wall_min_dist = R_wall_min_dist
        self.r_km = R_km
        self.r_for = R_for
        self.r_stop_t = R_stop_t
        self.r_danger = R_danger
        self.r_forward = R_forward
        self.r_collision = R_collision
        self.r_col_wall = R_col_wall
        self.r_goal = R_goal

        #print("r_goal: ", self.r_goal)

        self.reward_list.append(reward)

        #Store the reward in the class variable
        self.reward = reward

        #print("Accumulated Reward: ", self.accumulated_reward)
        #Store the accumulated reward value
        self.accumulated_reward += reward

        
        return reward

    def get_attention_weights(self):
        return self.model.attention_weights
