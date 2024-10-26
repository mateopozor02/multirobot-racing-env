#!/usr/bin/python2.7
# Author: Keyu Li <kyli@link.cuhk.edu.hk>

from __future__ import division
import logging
import os
import torch
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
import configparser
import gym
import tf
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.state import ObservableState, FullState, JointState
#from yolact_ROS.msg import objects
import rospy
import rospkg
from geometry_msgs.msg import Point, Vector3, Twist, Pose, PoseStamped, PoseWithCovarianceStamped, TwistWithCovariance
from std_msgs.msg import Int32, ColorRGBA, Float32
from people_msgs.msg import Person, People
from obstacle_detector.msg import Obstacles, CircleObstacle
from visualization_msgs.msg import Marker, MarkerArray
import threading


HUMAN_RADIUS = 0.3
#Update robot radius to fit turtle bot 3 burger
ROBOT_RADIUS = 0.12
ROBOT_V_PREF = 0.22
DISCOMFORT_DIST = 0
TIME_LIMIT = 300
GOAL_TOLERANCE = 0.3
FAKE_HUMAN_PX = -12
FAKE_HUMAN_PY = -16

def add(v1, v2):
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

class Robot(object):
    def __init__(self):
        self.v_pref = ROBOT_V_PREF
        self.radius = ROBOT_RADIUS
        self.px = None
        self.py = None
        self.gx = None
        self.gy = None
        self.vx = None
        self.vy = None
        self.theta = None

    def set(self, px, py, gx, gy, vx, vy, theta):
        self.px = px
        self.py = py
        self.gx = gx
        self.gy = gy
        self.vx = vx
        self.vy = vy
        self.theta = theta

    def get_full_state(self):
        return FullState(self.px, self.py, self.vx, self.vy, self.radius, self.gx, self.gy, self.v_pref, self.theta)

    def get_position(self):
        return self.px, self.py

    def get_goal_position(self):
        return self.gx, self.gy

    def reached_destination(self):
        #print(self.get_goal_position())
        return np.linalg.norm(np.array(self.get_position()) - np.array(self.get_goal_position())) < GOAL_TOLERANCE
        #      || (position - goal position) ||


'''class Human(object):
    def __init__(self, px, py, vx, vy):
        self.radius = HUMAN_RADIUS
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
    def get_observable_state(self):
        return ObservableState(self.px, self.py, self.vx, self.vy, self.radius)'''

class Human(object):
    def __init__(self, px, py, vx, vy, radius):
        self.radius = radius
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
    def get_observable_state(self):
        return ObservableState(self.px, self.py, self.vx, self.vy, self.radius)

class RobotAction(object):
    def __init__(self):
        self.Is_lg_Received = False
        self.IsAMCLReceived = False
        self.IsObReceived = False
        self.Is_gc_Received = False
        self.getStartPoint = False
        self.Is_lg_Reached = False
        self.Is_gg_Reached = False
        self.received_gx = None
        self.received_gy = None
        self.px = None
        self.py = None
        self.vx = None
        self.vy = None
        self.gx = None
        self.gy = None
        self.v_pref = None
        self.theta = None
        self.humans = None
        #self.objects = None
        self.ob = None
        self.ob2 = None
        self.obs = None
        self.state = None
        self.cmd_vel = Twist()
        self.plan_counter = 0
        self.num_pos = 0
        self.num_lg = 0
        self.start_px = None
        self.start_py = None

        # ROS subscribers
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_robot_pos)
        self.robot_odom_sub = rospy.Subscriber('/odom', Odometry, self.robot_vel_on_map_calculator)
        #self.people_sub = rospy.Subscriber('/people', People, self.update_humans)
        #self.people_sub = rospy.Subscriber('/objects', objects, self.update_humans)
        self.objects_sub = rospy.Subscriber('/obstacles', Obstacles, self.update_objects)
        self.goal_sub = rospy.Subscriber('/local_goal', PoseStamped, self.get_goal_on_map)
        self.global_costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.get_gc)
        # ROS publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.action_marker_pub = rospy.Publisher('/action_marker', Marker, queue_size=1)
        self.trajectory_marker_pub = rospy.Publisher('/trajectory_marker', Marker, queue_size=1)
        self.vehicle_marker_pub = rospy.Publisher('/vehicle_marker', Marker, queue_size=1)
        self.markers_pub_ = rospy.Publisher('/human_marker', Marker, queue_size=10)
        # Include the publisher for the goal
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        #self.send_goal(7, 3, 0.0, 0.0)


    def update_robot_pos(self, msg):
        self.IsAMCLReceived = True
        self.num_pos += 1
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))  # bounded by [-pi, pi]
        if not self.getStartPoint:
            rospy.loginfo("Start point is:(%s,%s)" % (self.px,self.py))
            self.getStartPoint = True
        self.visualize_trajectory(position, orientation)

    def robot_vel_on_map_calculator(self, msg):
        vel_linear = msg.twist.twist.linear
        listener_v.waitForTransform('/map', '/tb3_2_tf/base_footprint', rospy.Time(0), rospy.Duration(10))
        trans, rot = listener_v.lookupTransform('/map', '/tb3_2_tf/base_footprint', rospy.Time(0))
        #listener_v.waitForTransform('/map', '/base_footprint', rospy.Time(0), rospy.Duration(10))
        #trans, rot = listener_v.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        # rotate vector 'vel_linear' by quaternion 'rot'
        q1 = rot
        q2 = list()
        q2.append(vel_linear.x)
        q2.append(vel_linear.y)
        q2.append(vel_linear.z)
        q2.append(0.0)
        output_vel = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]
        self.vx = output_vel[0]
        self.vy = output_vel[1]

    def update_objects(self, msg):
        # observable state: px,py,vx,vy,radius
        self.IsObReceived = True
        self.objects = list()
        self.ob = list()
        #print("Message in SARL Node: ", msg)
        for p in msg.circles:
            #print("CircleObstacle: ", p.center.x, p.center.y, p.velocity.x, p.velocity.y, p.true_radius)
            human = Human(p.center.x, p.center.y, p.velocity.x, p.velocity.y, p.true_radius)
            self.objects.append(human)

        for human in self.objects:
            self.ob.append(human.get_observable_state())

    def update_humans(self, msg):
        # observable state: px,py,vx,vy,radius
        self.IsObReceived = True
        self.humans = list()
        self.ob2 = list()
        listener_ob.waitForTransform('/map', msg.header.frame_id, rospy.Time(0), rospy.Duration(10))
        tran, rot = listener_ob.lookupTransform('/map', msg.header.frame_id, rospy.Time(0))
        for object in msg.objects:
            tf_msg = PoseStamped()
            tf_msg.header = msg.header
            tf_msg.header.frame_id = msg.header.frame_id
            tf_msg.pose.position.x = object.pos.x
            tf_msg.pose.position.y = object.pos.y
            tf_msg.pose.position.z = object.pos.z
            tf_msg.pose.orientation.x = 0
            tf_msg.pose.orientation.y = 0  
            tf_msg.pose.orientation.z = 0
            tf_msg.pose.orientation.w = 1

            q1 = rot
            q2 = list()
            q2.append(object.vel.x)
            q2.append(object.vel.y)
            q2.append(object.vel.z)
            q2.append(0.0)
            output_vel = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(q1, q2),
                tf.transformations.quaternion_conjugate(q1)
            )[:3]

            tf_pos = listener_ob.transformPose('/map', tf_msg)
            output_pos = [tf_pos.pose.position.x, tf_pos.pose.position.y, tf_pos.pose.position.z]

            human = Human(output_pos[0], output_pos[1], output_vel[0], output_vel[1], HUMAN_RADIUS)
            self.visualize_human(object.object_id, output_pos)
            self.humans.append(human)

        for human in self.humans:
            self.ob2.append(human.get_observable_state())

    def get_goal_on_map(self, msg):
        self.Is_lg_Received = True
        listener_g.waitForTransform('/map', '/tb3_2_tf/odom', rospy.Time(0), rospy.Duration(10))
        #listener_g.waitForTransform('/map', '/odom', rospy.Time(0), rospy.Duration(10))
        tfmsg = listener_g.transformPose("/map", msg)
        #print("Local goal is received: ", tfmsg.pose.position.x, tfmsg.pose.position.y)
        self.received_gx = tfmsg.pose.position.x
        self.received_gy = tfmsg.pose.position.y

    def get_gc(self, msg):
        if not self.Is_gc_Received:
            policy.gc = msg.data
            policy.gc_resolution = msg.info.resolution
            policy.gc_width = msg.info.width
            policy.gc_ox = msg.info.origin.position.x
            policy.gc_oy = msg.info.origin.position.y
            # print(policy.gc_resolution, policy.gc_width, policy.gc_ox, policy.gc_oy)
            print("************ Global costmap is received. **************")
            self.Is_gc_Received = True

    def visualize_goal(self):
        # red cube for local goals
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal"
        marker.id = self.num_lg
        marker.type = marker.CUBE
        marker.action = marker.ADD
        #print("Local goal is visualized: ", self.gx, self.gy)
        marker.pose.position.x = self.gx
        marker.pose.position.y = self.gy
        marker.pose.position.z = 0.2
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker.color = ColorRGBA(r=1.0, a=1.0)
        marker.lifetime = rospy.Duration()
        self.goal_marker_pub.publish(marker)

    def visualize_trajectory(self, position, orientation):
        # Purple track for robot trajectory over time
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = '/map'
        marker.ns = 'robot'
        marker.id = self.num_pos
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.position = position
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker.color = ColorRGBA(r=0.5, b=0.8, a=1.0)
        marker.lifetime = rospy.Duration()
        self.trajectory_marker_pub.publish(marker)

    def visualize_action(self):
        robot_pos = Point(x=self.px, y=self.py, z=0)
        next_theta = self.theta + self.cmd_vel.angular.z
        next_vx = self.cmd_vel.linear.x * np.cos(next_theta)
        next_vy = self.cmd_vel.linear.x * np.sin(next_theta)
        action = Vector3(x=next_vx, y=next_vy, z=0)
        # green arrow for action (command velocity)
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "/map"
        marker.ns = "action"
        marker.id = 0
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.points = [robot_pos, add(robot_pos, action)]
        marker.scale = Vector3(x=0.1, y=0.3, z=0)
        marker.color = ColorRGBA(g=1.0, a=1.0)
        marker.lifetime = rospy.Duration(0.5)
        self.action_marker_pub.publish(marker)

    def visualize_human(self,id,position_current):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = '/map'
        name = "PEOPLE" + str(id)
        marker.ns = name
        marker.id = id
        marker.type = marker.SPHERE
        marker.pose.position.x = position_current[0]
        marker.pose.position.y = position_current[1]
        marker.pose.position.z = position_current[2]
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1
        marker.color.g = 1
        marker.lifetime = rospy.Duration(1)
        self.markers_pub_.publish(marker)

    def planner(self):
        # update robot
        robot.set(self.px, self.py, self.gx, self.gy, self.vx, self.vy, self.theta)

        # compute command velocity
        if robot.reached_destination():
            print("Robot reached destination.")
            self.cmd_vel.linear.x = 0
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = 0
            self.Is_lg_Reached = True
            print("Gx and Gy: ", self.gx, self.gy)
            #print("Received Gx and Gy: ", self.received_gx, self.received_gy)
            if self.gx == self.received_gx and self.gy == self.received_gy:
                self.Is_gg_Reached = True
        else:
            """
            self state: FullState(px, py, vx, vy, radius, gx, gy, v_pref, theta)
            ob:[ObservableState(px1, py1, vx1, vy1, radius1),
                ObservableState(px1, py1, vx1, vy1, radius1),
                   .......                    
                ObservableState(pxn, pyn, vxn, vyn, radiusn)]
            """
            if self.ob2 == None:
                self.obs = self.ob   # using only obstacle_detector
            else:
                self.obs = self.ob+self.ob2   # using both obstacle_detector and Yolact
            
            if len(self.obs)==0:
                #print("No obstacles detected.")
                self.obs = [ObservableState(self.px+FAKE_HUMAN_PX, self.py+FAKE_HUMAN_PY, 0, 0, HUMAN_RADIUS)]

            #for observation in self.obs:
            #    print("Observation: ", observation.px, observation.py)
            self.state = JointState(robot.get_full_state(), self.obs)
            action = policy.predict(self.state)  # max_action
            self.cmd_vel.linear.x = action.v
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = action.r

        ########### for debug ##########
        # dist_to_goal = np.linalg.norm(np.array(robot.get_position()) - np.array(robot.get_goal_position()))
        # if self.plan_counter % 10 == 0:
        #     rospy.loginfo("robot position:(%s,%s)" % (self.px, self.py))
        #     rospy.loginfo("Distance to goal is %s" % dist_to_goal)
        #     rospy.loginfo("self state:\n %s" % self.state.self_state)
        #     for i in range(len(self.state.human_states)):
        #         rospy.loginfo("human %s :\n %s" % (i+1, self.state.human_states[i]))
        #     rospy.loginfo("%s-th action is planned: \n v: %s m/s \n r: %s rad/s"
        #                   % (self.plan_counter, self.cmd_vel.linear.x, self.cmd_vel.angular.z))


        # publish command velocity
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.plan_counter += 1
        self.visualize_action()
    
    

    #Function to send the goal programmatically
    def send_goal(self, position_x, position_y, orientation_z, orientation_w):
        # Create a PoseStamped message for the goal 
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()

        goal_msg.pose.position.x = position_x
        goal_msg.pose.position.y = position_y
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0 
        goal_msg.pose.orientation.w = 1.0 

        # Check if the global costmap has been received
        while not self.Is_gc_Received:
            rospy.loginfo("Waiting for the global costmap to be received.")
            rate.sleep()

        # Publish the goal
        self.goal_publisher.publish(goal_msg)

        #Display a confirmation message
        rospy.loginfo("Goal has been sent: ({},{})".format(position_x, position_y))
    


if __name__ == '__main__':
    begin_travel = False
    # set file dirs
    pack_path = rospkg.RosPack().get_path('sarl_star_ros')
    model_dir = pack_path + '/CrowdNav/crowd_nav/data/output_alisher_curvepath_5_5/'
    env_config_file = model_dir + 'env.config'
    policy_config_file = model_dir + 'policy.config'

    if os.path.exists(os.path.join(model_dir, 'resumed_rl_model.pth')):
        model_weights = os.path.join(model_dir, 'resumed_rl_model.pth')
    else:
        model_weights = os.path.join(model_dir, 'rl_model.pth')

    # configure logging and device
    logging.basicConfig(level=logging.INFO, format='%(asctime)s, x%(levelname)s: %(message)s',
                        datefmt="%Y-%m-%d %H:%M:%S")
    device = torch.device("cpu")
    logging.info('Using device: %s', device)

    # configure RL policy
    policy = 'sarl'
    phase = 'test'
    env_config = configparser.RawConfigParser()
    env_config.read(env_config_file)
    env = gym.make('CrowdSim-v0')
    env.configure(env_config)
    env.discomfort_dist = DISCOMFORT_DIST
    policy = policy_factory[policy]()
    policy_config = configparser.RawConfigParser()
    policy_config.read(policy_config_file)
    policy.configure(policy_config)
    policy.with_costmap = True
    # use constant velocity model to predict next state
    policy.query_env = False  
    policy.get_model().load_state_dict(torch.load(model_weights))
    policy.set_phase(phase)
    policy.set_device(device)
    policy.set_env(env)
    policy.time_step = 0.25
    policy.gc = []
    robot = Robot()

    #partial_goals = [(7, 7.7), (6.8, 9.2), (5.3, 10), (-3.75, 10.65), (-5.29, 10.6), (-6.55, 12.7), (-6.72, 15.96),
    #                 (-7.39, 17.1), (-12.98, 17.6), (-15.12, 16.7), (-15.37, 10.33), (-11.18, 5.6), (-15.29, -5.74),
    #                 (-0.69, -9.6), (-0.28, -19.11), (6.15, -25.74), (13.71, -20.51), (11.09, -20.43), (7.67, -16.71), (7.45, -4.65)]

    #partial_goals = [(7, 7.7), (4.69, 10.54), (-3.75, 10.65), (-6.86, 13.48), (-7.27, 16.53), (-14.31, 17.48), 
    #                 (-15.37, 10.33), (-11.18, 5.6), (-11.29, -0.51), (-15.41, -5.25), (-0.69, -9.6), (-0.28, -19.11),
    #                 (6.15, -25.74), (-13.3, -25.7), (13.71, -20.51), (10.91, -20.47), (7.87, -16.71), (7.45, -4.65)]
    
    partial_goals = [(5.3, 10), (7, 8.2)]

    goals_reached = 0

    try:
        rospy.init_node('sarl_star_node', anonymous=True)
        rate = rospy.Rate(4)  # 4Hz, time_step=0.25
        robot_act = RobotAction()
        listener_v = tf.TransformListener()
        listener_g = tf.TransformListener()
        listener_ob = tf.TransformListener()

        #robot_act.send_goal(partial_goals[0][0], partial_goals[0][1], 0.0, 0.0)

        #robot_act.send_goal(7.6, 6.73, 0.0, 0.0)
        

        while not rospy.is_shutdown():

            
            if robot_act.Is_gg_Reached:
                finish_travel_time = rospy.get_time()
                t = finish_travel_time - begin_travel_time
                rospy.loginfo("Goal is reached. Travel time: %s s." % t)
                #rospy.sleep(3)

                goals_reached += 1

                if goals_reached == len(partial_goals):
                    rospy.loginfo("All goals have been reached.")
                    break

                # Reset the gg reached flag
                robot_act.Is_gg_Reached = False

                robot_act.send_goal(partial_goals[goals_reached][0], partial_goals[goals_reached][1], 0.0, 0.0)

                # Sleep to allow the robot to reach the goal
                rospy.sleep(1)

            # wait for msgs of goal, AMCL and ob
            if robot_act.Is_lg_Received and robot_act.IsAMCLReceived and robot_act.IsObReceived:

                # travel time
                if not begin_travel:
                    begin_travel_time = rospy.get_time()
                    begin_travel = True

                # update local goal (gx,gy)
                robot_act.gx = robot_act.received_gx
                robot_act.gy = robot_act.received_gy
                robot_act.num_lg += 1
                robot_act.visualize_goal()
                robot_act.planner()
                #print("Local goal: ", robot_act.gx, robot_act.gy)
                finish_travel_time = rospy.get_time()
                t = finish_travel_time - begin_travel_time
                if t > TIME_LIMIT:
                    rospy.loginfo("Timeout. Travel time: %s s." % t)
                    break
            

            rate.sleep()

    except rospy.ROSInterruptException, e:
        raise e