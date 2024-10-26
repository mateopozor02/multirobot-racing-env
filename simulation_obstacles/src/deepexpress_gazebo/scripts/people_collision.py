#! /usr/bin/env python

from tkinter.messagebox import NO
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
from random import Random

VELOCITY = 1.0
ACCELERATION = 0.1
i=0.01
j=0.02
k=0.03
l=0.04

class MoveObstacle():
    number = 1
    
    x1,x2,x3,x4,x5,x6 = 0,0,0,0,0,0
    y1,y2,y3,y4,y5,y6 = 0,0,0,0,0,0
    vx1,vx2,vx3,vx4,vx5,vx6 = 0,0,0,0,0,0
    vy1,vy2,vy3,vy4,vy5,vy6 = 0,0,0,0,0,0

    def __init__(self):
        # publisher 
        self.move_legs1_pub = rospy.Publisher('/legs1/command_velocity',Twist, queue_size=1)
        self.move_legs2_pub = rospy.Publisher('/legs2/command_velocity',Twist, queue_size=1)
        self.move_legs3_pub = rospy.Publisher('/legs3/command_velocity',Twist, queue_size=1)
        self.move_legs4_pub = rospy.Publisher('/legs4/command_velocity',Twist, queue_size=1)
        self.move_legs5_pub = rospy.Publisher('/legs5/command_velocity',Twist, queue_size=1)
        self.move_legs6_pub = rospy.Publisher('/legs6/command_velocity',Twist, queue_size=1)
    
        # subscriber
        rospy.Subscriber('/legs1/odom',Odometry,self.b1odom_callback)
        rospy.Subscriber('/legs2/odom',Odometry,self.b2odom_callback)
        rospy.Subscriber('/legs3/odom',Odometry,self.b3odom_callback)
        rospy.Subscriber('/legs4/odom',Odometry,self.b4odom_callback)
        rospy.Subscriber('/legs5/odom',Odometry,self.b5odom_callback)
        rospy.Subscriber('/legs6/odom',Odometry,self.b6odom_callback)

        self.is_initial_1 = False
        self.is_initial_2 = False
        self.is_initial_3 = False
        self.is_initial_4 = False
        self.is_initial_5 = False
        self.is_initial_6 = False

# Call the odometry for each legs
    def b1odom_callback(self,msg1):
        self.x1 = msg1.pose.pose.position.x
        self.y1 = msg1.pose.pose.position.y
        self.vx1 = msg1.twist.twist.linear.x
        self.vy1 = msg1.twist.twist.linear.y

    def b2odom_callback(self,msg2):
        self.x2 = msg2.pose.pose.position.x
        self.y2 = msg2.pose.pose.position.y
        self.vx2 = msg2.twist.twist.linear.x
        self.vy2 = msg2.twist.twist.linear.y

    def b3odom_callback(self,msg3):
        self.x3 = msg3.pose.pose.position.x
        self.y3 = msg3.pose.pose.position.y
        self.vx3 = msg3.twist.twist.linear.x
        self.vy3 = msg3.twist.twist.linear.y

    def b4odom_callback(self,msg4):
        self.x4 = msg4.pose.pose.position.x
        self.y4 = msg4.pose.pose.position.y
        self.vx4 = msg4.twist.twist.linear.x
        self.vy4 = msg4.twist.twist.linear.y

    def b5odom_callback(self,msg5):
        self.x5 = msg5.pose.pose.position.x
        self.y5 = msg5.pose.pose.position.y
        self.vx5 = msg5.twist.twist.linear.x
        self.vy5 = msg5.twist.twist.linear.y

    def b6odom_callback(self,msg6):
        self.x6 = msg6.pose.pose.position.x
        self.y6 = msg6.pose.pose.position.y
        self.vx6 = msg6.twist.twist.linear.x
        self.vy6 = msg6.twist.twist.linear.y
#Move each legs with different velocity
#Move each legs with different velocity
    def setting_legs1_velocity(self):
        if self.is_initial_1:
            if self.y1 >= 1.5:
                self.move1 = Twist()
                self.move1.linear.x = 0
                self.move1.linear.y = -VELOCITY
                self.move1.angular.z = 0
            elif self.y1 <= -1.5:
                self.move1 = Twist()
                self.move1.linear.x = 0
                self.move1.linear.y = VELOCITY
                self.move1.angular.z = 0

        else:
            self.move1 = Twist()
            self.move1.linear.x = 0
            self.move1.linear.y = -VELOCITY
            self.move1.angular.z = 0
            self.is_initial_1 = True
    
    def setting_legs2_velocity(self):
        if self.is_initial_2:
            if self.y2 >= 1.5:
                self.move2 = Twist()
                self.move2.linear.x = 0
                self.move2.linear.y = -VELOCITY
                self.move2.angular.z = 0
            elif self.y2 <= -1.5:
                self.move2 = Twist()
                self.move2.linear.x = 0
                self.move2.linear.y = VELOCITY
                self.move2.angular.z = 0
        else:
            self.move2 = Twist()
            self.move2.linear.x = 0
            self.move2.linear.y = VELOCITY
            self.move2.angular.z = 0
            self.is_initial_2 = True
        
    def setting_legs3_velocity(self):
        if self.is_initial_3:
            if self.y3 >= 1.5:
                self.move3 = Twist()
                self.move3.linear.x = 0
                self.move3.linear.y = -0.5
                self.move3.angular.z = 0
            elif self.y3 <= -1.5:
                self.move3 = Twist()
                self.move3.linear.x = 0
                self.move3.linear.y = VELOCITY
                self.move3.angular.z = 0
        else:
            self.move3 = Twist()
            self.move3.linear.x = 0
            self.move3.linear.y = -VELOCITY
            self.move3.angular.z = 0
            self.is_initial_3 = True

    def setting_legs4_velocity(self):
        if self.is_initial_4:
            if self.y4 >= 1.5:
                self.move4 = Twist()
                self.move4.linear.x = 0
                self.move4.linear.y = -VELOCITY
                self.move4.angular.z = 0
            elif self.y4 <= -1.5:
                self.move4 = Twist()
                self.move4.linear.x = 0
                self.move4.linear.y = VELOCITY
                self.move4.angular.z = 0
        else:
            self.move4 = Twist()
            self.move4.linear.x = 0
            self.move4.linear.y = VELOCITY
            self.move4.angular.z = 0
            self.is_initial_4 = True

    def setting_legs5_velocity(self):
        if self.is_initial_5:
            if self.y5 > 1.5:
                self.move5 = Twist()
                self.move5.linear.x = 0
                self.move5.linear.y = -0.5
                self.move5.angular.z = 0
            elif self.y5 < -1.5:
                self.move5 = Twist()
                self.move5.linear.x = 0
                self.move5.linear.y = VELOCITY
                self.move5.angular.z = 0
        else:
            self.move5 = Twist()
            self.move5.linear.x = 0
            self.move5.linear.y = -VELOCITY
            self.move5.angular.z = 0
            self.is_initial_5 = True

    def setting_legs6_velocity(self):
        if self.is_initial_6:
            if self.y6 > 1.5:
                self.move6 = Twist()
                self.move6.linear.x = 0
                self.move6.linear.y = -VELOCITY
                self.move6.angular.z = 0
            elif self.y6 < -1.5:
                self.move6 = Twist()
                self.move6.linear.x = 0
                self.move6.linear.y = VELOCITY
                self.move6.angular.z = 0
        else:
            self.move6 = Twist()
            self.move6.linear.x = 0
            self.move6.linear.y = VELOCITY
            self.move6.angular.z = 0
            self.is_initial_6 = True
#print linear velocity for each legs

    def move_legs(self):
        #print("linear velocity of legs 1 : " + str(i))
        #print("linear_velocity legs 2 : " + str(j))
        #print("linear_velocity of legs 3 : " + str(k))
        #print("linear_velocity of legs 4 : " + str(l))  
####################################################################
        self.move_legs1_pub.publish(self.move1)
        self.move_legs2_pub.publish(self.move2)
        self.move_legs3_pub.publish(self.move3)
        self.move_legs4_pub.publish(self.move4)
        self.move_legs5_pub.publish(self.move5)
        self.move_legs6_pub.publish(self.move6)
#Print the odometry for the legses
    def print_odom(self):
        #print
        print('')
        print(" legs1: Odom: x:"+ str(self.x1)+" Odom: y:"+ str(self.y1)+ " Geo: vx : "+ str(self.vx1)+ " Geo: vy : "+ str(self.vy1))
        print(" legs2: Odom: x:"+ str(self.x2)+" Odom: y:"+ str(self.y2)+ " Geo: vx : "+ str(self.vx2)+ " Geo: vy : "+ str(self.vy2))
        print(" legs3: Odom: x:"+ str(self.x3)+" Odom: y:"+ str(self.y3)+ " Geo: vx : "+ str(self.vx3)+ " Geo: vy : "+ str(self.vy3))
        print(" legs4: Odom: x:"+ str(self.x4)+" Odom: y:"+ str(self.y4)+ " Geo: vx : "+ str(self.vx4)+ " Geo: vy : "+ str(self.vy4))
        

if __name__ == "__main__":

    rospy.init_node('move_obstacle')

    MO = MoveObstacle() 

    rate = rospy.Rate(10)
    
    i=0.5
    j=0.002
    k=1
    l=0.04
    while not rospy.is_shutdown():
        MO.setting_legs1_velocity()
        MO.setting_legs2_velocity()
        MO.setting_legs3_velocity()
        MO.setting_legs4_velocity()
        MO.setting_legs5_velocity()
        MO.setting_legs6_velocity()
        MO.move_legs()
        #MO.print_odom()
        i+=ACCELERATION
        j+=ACCELERATION
        k+=ACCELERATION
        l+=ACCELERATION
        #rate.sleep()

