#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
import os
import random

class mytarget:
    def __init__(self):
        rospy.init_node('mytarget', anonymous=True)
        x = random.randint(0, 11)   
        y = random.randint(0, 11)
        ang = random.randint(0,31)
        cmd = "rosservice call /spawn " + str(x) + " " +  str(y) + " " + str(ang/10) + " 'mytarget'"
        os.system(cmd)
        self.pose = Pose()
        self.rate = rospy.Rate(10) # 10hz
        self.pose.x = x
        self.pose.y = y
        pub = rospy.Publisher('target', Pose, queue_size=10)
        while not rospy.is_shutdown():
            pub.publish(self.pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        x=mytarget()
    except rospy.ROSInterruptException:
        pass

