#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64


def wheels_effort_controllers_JointVelocityController(velocity): 
    #Define publishers for each joint effort controller commands.
    

    
    
    #keti_control
    pub1 = rospy.Publisher('/deepexpress/FLwheel_Velocity_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/deepexpress/FRwheel_Velocity_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/deepexpress/MLwheel_Velocity_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/deepexpress/MRwheel_Velocity_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/deepexpress/RLwheel_Velocity_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/deepexpress/RRwheel_Velocity_controller/command', Float64, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        rospy.loginfo(velocity)
        
        pub1.publish(velocity)
        pub2.publish(velocity)
        pub3.publish(velocity)
        pub4.publish(velocity)
        pub5.publish(velocity)
        pub6.publish(velocity)
        rate.sleep()  #sleep for rest of rospy.Rate(10)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    
    #Initiate node for controlling velocitys in the wheeel
    rospy.init_node('keit_wheels', anonymous=True)
    try:
        wheels_effort_controllers_JointVelocityController(1.2) #rad/s INPUT the velocity
        
    except rospy.ROSInterruptException:
        pass
