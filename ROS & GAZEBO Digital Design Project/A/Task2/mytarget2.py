#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import os
import random
from math import pow, atan2, sqrt 
import time

class mytarget:
    def __init__(self):
        rospy.init_node('mytarget_controller', anonymous=True)
        x = random.randint(0, 11)   
        y = random.randint(0, 11)
        ang = random.randint(0,31)
        cmd = "rosservice call /spawn " + str(x) + " " +  str(y) + " " + str(ang/10) + " 'mytarget'"
        os.system(cmd)
        self.velocity_publisher = rospy.Publisher('/mytarget/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/mytarget/pose', Pose, self.update_pose)
        self.robot1pose_subscriber = rospy.Subscriber('robot1/pose', Pose, self.update_robot1pose)
        self.pose = Pose()
        self.robot1_pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def update_robot1pose(self, data):
        self.robot1_pose = data
        self.robot1_pose.x = round(self.robot1_pose.x, 4)
        self.robot1_pose.y = round(self.robot1_pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))
    def linear_vel(self, goal_pose, constant=0.5):
        #return constant * self.euclidean_distance(goal_pose)
        return 2
    def steering_angle(self, goal_pose):
        return atan2(-(goal_pose.y - self.pose.y), -(goal_pose.x - self.pose.x))
    def angular_vel(self, goal_pose, constant=2):
        return constant* (self.steering_angle(goal_pose) - self.pose.theta)


    def run_away(self):
        """Moves the turtle to the goal."""
        distance_tolerance = 0.05
        vel_msg = Twist()
        time.sleep(1)
        while self.euclidean_distance(self.robot1_pose) >= float(distance_tolerance):
            vel_msg.linear.x = self.linear_vel(self.robot1_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(self.robot1_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin()


if __name__ == '__main__':
    try:
        x=mytarget()
        time.sleep(1)   #important, without this goal position doesn't update in time
        x.run_away()
    except rospy.ROSInterruptException:
        pass


