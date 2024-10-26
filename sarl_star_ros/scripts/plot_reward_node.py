#!/usr/bin/python2.7
import rospy 
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np

class RealTimePlotter:
    def __init__(self):
        rospy.init_node('reward_plotter', anonymous=True)
        self.reward_data = []
        self.times = []
        self.start_time = rospy.get_time()

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')

        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-5, 10)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Reward')
        self.ax.set_title('Real-Time Reward Plot')

        rospy.Subscriber('calculated_reward', Float32, self.callback)
        #plt.ion()
        #plt.show()

    def callback(self, data):
        current_time = rospy.get_time() - self.start_time
        self.reward_data.append(data.data)
        self.times.append(current_time)

        if current_time > self.ax.get_xlim()[1]:
            self.ax.set_xlim(0, current_time + 10)
        
        self.line.set_data(self.times, self.reward_data)
        self.ax.relim()
        self.ax.autoscale_view()
        #plt.draw()
        #plt.pause(0.01)

if __name__ == '__main__':
    plotter = RealTimePlotter()
    print('Plotting reward...')
    rospy.spin()