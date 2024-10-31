from __future__ import division
import logging
import copy
import torch
import numpy as np
from crowd_sim.envs.utils.info import *
from crowd_sim.envs.utils.action import ActionXY, ActionRot


class Explorer(object):
    def __init__(self, env, robot, device, memory=None, gamma=None, target_policy=None):
        self.env = env
        self.robot = robot
        # self.robot_path_length_list = []
        self.device = device
        self.memory = memory
        self.gamma = gamma
        self.target_policy = target_policy
        self.target_model = None

    def update_target_model(self, target_model):
        self.target_model = copy.deepcopy(target_model)

    # @profile
    def run_k_episodes(self, k, phase, update_memory=False, imitation_learning=False, episode=None,
                       print_failure=False):
        nos = 3
        self.robot.policy.set_phase(phase)
        robot_path_length_list = []
        success_times = []
        collision_times = []
        timeout_times = []
        collision_wall_times = []
        success = []
        collision = []
        timeout = []
        collision_wall = []
        too_close = 0
        min_dist = []
        cumulative_rewards = []
        collision_cases = []
        timeout_cases = []
        collision_wall_cases = []
        each_too_close = [0]*nos
        each_min_dist = [0]*nos
	
        for i in range(k):
            ob = self.env.reset(i,phase)
            logging.info("running %s/%s episode" %(i+1,k)+ ", simulation environment: " + str(self.env.test_sim))
            done = False
            each = i%nos
            states = []
            actions = []
            rewards = []
            length = 0
            stops = 0
            rewards_dict = {"Total Reward": 0,"R_dan": 0, "R_forward": 0, "R_goal": 0,"R_col": 0,"R_km": 0, "R_col_wall": 0, "R_wall_min_dist": 0, "R_stop_t": 0}
            far_points = 0
            close_points = 0
            while not done:
                action = self.robot.act(ob)
                if isinstance(action, ActionXY):
                    length = length + 0.25*np.linalg.norm([action.vx,action.vy])
                else:
                    length = length + 0.25*action.v
                ob, reward, done, info, reward_values, end_dg, coordinates, is_stopped = self.env.step(action)
                states.append(self.robot.policy.last_state)
                actions.append(action)
                rewards.append(reward)
                rewards_dict = {key: round(rewards_dict[key] + reward_values[key], 3) for key in rewards_dict}
                if is_stopped:
                    stops += 1
                if isinstance(info, Danger):
                    each_too_close[each] += 1
                    too_close += 1
                    each_min_dist[each] += info.min_dist
                    min_dist.append(info.min_dist)

            if isinstance(info, ReachGoal):
                logging.info("%s/%s episode: Success! Total Reward: %s, R_dan: %s, R_forward: %s, R_goal: %s, R_col: %s, R_km: %s, R_col_wall: %s, R_wall_min_dist: %s, R_stop_t: %s" % (i + 1, k, rewards_dict["Total Reward"], rewards_dict["R_dan"], rewards_dict["R_forward"], rewards_dict["R_goal"], rewards_dict["R_col"], rewards_dict["R_km"], rewards_dict["R_col_wall"], rewards_dict["R_wall_min_dist"], rewards_dict["R_stop_t"]))
                logging.info("Path length: %s" % length)
                logging.info("Path left: %s" % end_dg)
                logging.info("Actions: %s" % len(actions))
                logging.info("Coordinates: %s %s" % coordinates)
                logging.info("Stops times: %s\n" % stops)
                robot_path_length_list.append(length)
                success.append(1)
                collision.append(0)
                timeout.append(0)
                collision_wall.append(0)
                success_times.append(self.env.global_time)
                collision_times.append(0)
                timeout_times.append(0)
                collision_wall_times.append(0)
            elif isinstance(info, Collision):
                logging.info("%s/%s episode: Collision! Total Reward: %s, R_dan: %s, R_forward: %s, R_goal: %s, R_col: %s, R_km: %s, R_col_wall: %s, R_wall_min_dist: %s, R_stop_t: %s" % (i + 1, k, rewards_dict["Total Reward"], rewards_dict["R_dan"], rewards_dict["R_forward"], rewards_dict["R_goal"], rewards_dict["R_col"], rewards_dict["R_km"], rewards_dict["R_col_wall"], rewards_dict["R_wall_min_dist"], rewards_dict["R_stop_t"]))
                logging.info("Path length: %s" % length)
                logging.info("Path left: %s" % end_dg)
                logging.info("Actions: %s" % len(actions))
                logging.info("Coordinates: %s %s" % coordinates)
                logging.info("Stops times: %s\n" % stops)
                robot_path_length_list.append(0)
                success.append(0)
                collision.append(1)
                timeout.append(0)
                collision_wall.append(0)
                collision_cases.append(i)
                success_times.append(0)
                collision_times.append(self.env.global_time)
                timeout_times.append(0)
                collision_wall_times.append(0)
            elif isinstance(info, Timeout):
                logging.info("%s/%s episode: Timeout! Total Reward: %s, R_dan: %s, R_forward: %s, R_goal: %s, R_col: %s, R_km: %s, R_col_wall: %s, R_wall_min_dist: %s, R_stop_t: %s" % (i + 1, k, rewards_dict["Total Reward"], rewards_dict["R_dan"], rewards_dict["R_forward"], rewards_dict["R_goal"], rewards_dict["R_col"], rewards_dict["R_km"], rewards_dict["R_col_wall"], rewards_dict["R_wall_min_dist"], rewards_dict["R_stop_t"]))
                logging.info("Path length: %s" % length)
                logging.info("Path left: %s" % end_dg)
                logging.info("Actions: %s" % len(actions))
                logging.info("Coordinates: %s %s" % coordinates)
                logging.info("Stops times: %s\n" % stops)
                robot_path_length_list.append(0)
                success.append(0)
                collision.append(0)
                timeout.append(1)
                collision_wall.append(0)
                timeout_cases.append(i)
                success_times.append(0)
                collision_times.append(0)
                timeout_times.append(self.env.time_limit)
                collision_wall_times.append(0)
            elif isinstance(info, CollisionWall):
                logging.info("%s/%s episode: Collision Wall! Total Reward: %s, R_dan: %s, R_forward: %s, R_goal: %s, R_col: %s, R_km: %s, R_col_wall: %s, R_wall_min_dist: %s, R_stop_t: %s" % (i + 1, k, rewards_dict["Total Reward"], rewards_dict["R_dan"], rewards_dict["R_forward"], rewards_dict["R_goal"], rewards_dict["R_col"], rewards_dict["R_km"], rewards_dict["R_col_wall"], rewards_dict["R_wall_min_dist"], rewards_dict["R_stop_t"]))
                logging.info("Path length: %s" % length)
                logging.info("Path left: %s" % end_dg)
                logging.info("Actions: %s" % len(actions))
                logging.info("Coordinates: %s %s" % coordinates)
                logging.info("Stops times: %s\n" % stops)
                robot_path_length_list.append(0)
                success.append(0)
                collision.append(0)
                timeout.append(0)
                collision_wall.append(1)
                collision_wall_cases.append(i)
                success_times.append(0)
                collision_times.append(0)
                timeout_times.append(0)
                collision_wall_times.append(self.env.time_limit)
            else:
                raise ValueError('Invalid end signal from environment')

            if update_memory:
                if isinstance(info, ReachGoal) or isinstance(info, Collision) or isinstance(info, CollisionWall):
                    # only add positive(success) or negative(collision) experience in experience set
                    self.update_memory(states, actions, rewards, imitation_learning)

            cumulative_rewards.append(sum([pow(self.gamma, t * self.robot.time_step * self.robot.v_pref)
                                           * reward for t, reward in enumerate(rewards)]))  # enumerate from 0
        success_number = [0]*nos
        collision_number = [0]*nos
        timeout_number = [0]*nos
        collision_wall_number = [0]*nos
        nav_time = [0]*nos
        avg_nav_time = [0]*nos
        path_length = [0]*nos
        avg_path_length = [0]*nos
        reward_sum = [0]*nos
        s_times_sum = [0]*nos
        c_times_sum = [0]*nos
        t_times_sum = [0]*nos
        cw_times_sum = [0]*nos
        for index in range(len(success)):
            remainder = index % nos
            success_number[remainder] += success[index]
            collision_number[remainder] += collision[index]
            timeout_number[remainder] += timeout[index]
            collision_wall_number[remainder] += collision_wall[index]
            nav_time[remainder] += success_times[index]
            path_length[remainder] += robot_path_length_list[index]
            reward_sum[remainder] += cumulative_rewards[index]
            s_times_sum[remainder] += success_times[index]
            c_times_sum[remainder] += collision_times[index]
            t_times_sum[remainder] += timeout_times[index]
            cw_times_sum[remainder] += collision_wall_times[index]

        for i in range(nos):
            if success_number[i]:
                avg_nav_time[i] += nav_time[i]/success_number[i]
                avg_path_length[i] += path_length[i]/success_number[i]
            else:
                avg_nav_time[i] = 0
                avg_path_length[i] = 0
	
        total_nav_time = 0
        total_path_length = 0
        if sum(success_number) == 0:
            total_nav_time = 0
            total_path_length = 0
        else:
            for i in range(nos):
                total_nav_time += avg_nav_time[i]*success_number[i]/sum(success_number)
                total_path_length += avg_path_length[i]*success_number[i]/sum(success_number)
	     
        assert sum(success) + sum(collision) + sum(timeout) + sum(collision_wall) == k
        # avg_path_length = sum(self.robot_path_length_list) / len(self.robot_path_length_list)
        # logging.info("The average successful navigation path length: %s" % avg_path_length)
        divider = [k//nos] * nos
        remain = k % nos
        each_total_time = []
        for i in range(nos):
            each_total_time.append((s_times_sum[i]+c_times_sum[i]+t_times_sum[i]+cw_times_sum[i])*self.robot.time_step)
            if remain > 0:
                divider[i] += 1
            remain = remain - 1
        if total_nav_time:
            total_speed = total_path_length/total_nav_time
        else:
            total_speed = 0
        avg_speed = [0]*nos
        for i in range(nos):
            if avg_nav_time[i]:
                avg_speed[i] += avg_path_length[i]/avg_nav_time[i]
            else:
                avg_speed[i] = 0

        extra_info = '' if episode is None else 'in episode {} '.format(episode)
        logging.info('{:<5} {}has success rate: {:.2f}, collision rate: {:.2f}, timeout rate: {:.2f}, collisionwall rate: {:.2f}, nav time: {:.2f}, average speed: {:.2f}, path length: {:.2f}, total reward: {:.4f}'.
                     format(phase.upper(), extra_info, sum(success_number)/k, sum(collision_number)/k, sum(timeout_number)/k, sum(collision_wall_number)/k, total_nav_time, total_speed, total_path_length, average(cumulative_rewards)))
        logging.info('In each scenarios, {:<5} {}has success rate: {:.2f} {:.2f} {:.2f}, collision rate: {:.2f} {:.2f} {:.2f}, timeout rate: {:.2f} {:.2f} {:.2f}, collisionwall rate: {:.2f} {:.2f} {:.2f}, nav time: {:.2f} {:.2f} {:.2f}, average speed: {:.2f} {:.2f} {:.2f}, path length: {:.2f} {:.2f} {:.2f}, total reward: {:.4f} {:.4f} {:.4f}'.
                     format(phase.upper(), extra_info, success_number[0]/divider[0], success_number[1]/divider[1], success_number[2]/divider[2], collision_number[0]/divider[0], collision_number[1]/divider[1], collision_number[2]/divider[2], timeout_number[0]/divider[0], timeout_number[1]/divider[1], timeout_number[2]/divider[2], collision_wall_number[0]/divider[0], collision_wall_number[1]/divider[1], collision_wall_number[2]/divider[2], avg_nav_time[0], avg_nav_time[1], avg_nav_time[2], avg_speed[0], avg_speed[1], avg_speed[2], avg_path_length[0], avg_path_length[1], avg_path_length[2], reward_sum[0]/divider[0], reward_sum[1]/divider[1], reward_sum[2]/divider[2]))
        avg_min_dist = [0]*nos
        for i in range(nos):
            if each_too_close[i]:
                avg_min_dist[i] = each_min_dist[i]/each_too_close[i]
            else:
                avg_min_dist[i] = 0
        if phase in ['val', 'test']:
            total_time = sum(success_times + collision_times + timeout_times + collision_wall_times) * self.robot.time_step
            logging.info('Frequency in danger: %.2f and average min separate distance in danger: %.2f', too_close / total_time, average(min_dist))
            logging.info('In each scenarios, Frequency in danger: %.2f %.2f %.2f and average min separate distance in danger: %.2f %.2f %.2f', each_too_close[0] / each_total_time[0], each_too_close[1] / each_total_time[1], each_too_close[2] / each_total_time[2], avg_min_dist[0], avg_min_dist[1], avg_min_dist[2])


        if print_failure:
            logging.info('Collision cases: ' + ' '.join([str(x) for x in collision_cases]))
            logging.info('Timeout cases: ' + ' '.join([str(x) for x in timeout_cases]))
            logging.info('COllisionWall cases: ' + ' '.join([str(x) for x in collision_wall_cases]))

    def update_memory(self, states, actions, rewards, imitation_learning=False):
        if self.memory is None or self.gamma is None:
            raise ValueError('Memory or gamma value is not set!')

        for i, state in enumerate(states):
            reward = rewards[i]

            # VALUE UPDATE
            if imitation_learning:
                # define the value of states in IL as cumulative discounted rewards, which is the same in RL
                state = self.target_policy.transform(state)
                # value = pow(self.gamma, (len(states) - 1 - i) * self.robot.time_step * self.robot.v_pref)
                value = sum([pow(self.gamma, max(t - i, 0) * self.robot.time_step * self.robot.v_pref) * reward
                             for t, reward in enumerate(rewards)])
            else:
                if i == len(states) - 1:
                    # terminal state
                    value = reward
                else:
                    next_state = states[i + 1]
                    gamma_bar = pow(self.gamma, self.robot.time_step * self.robot.v_pref)
                    value = reward + gamma_bar * self.target_model(next_state.unsqueeze(0)).data.item()
            value = torch.Tensor([value]).to(self.device)

            # transform state of different human_num into fixed-size tensor
            if len(state.size()) == 1:
                human_num = 1
                feature_size = state.size()[0]
            else:
                human_num, feature_size = state.size()
            """if human_num != 5:
                padding = torch.zeros((15 - human_num, feature_size))
                state = torch.cat([state, padding])"""

            self.memory.push((state, value))


def average(input_list):
    if input_list:
        return sum(input_list) / len(input_list)
    else:
        return 0