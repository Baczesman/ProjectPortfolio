#!/usr/bin/env python3
import rospy
import os
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import random
import time


class TurtleBot:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.pose_subscriber = rospy.Subscriber('/robot1/pose', Pose, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.ontarget = 0

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))
    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    def angular_vel(self, goal_pose, constant=6):
        return constant* (self.steering_angle(goal_pose) - self.pose.theta)

    def find_target(self):
        while self.ontarget == 0:
            self.target_subscriber = rospy.Subscriber('target', Pose, self.move2goal)


    def move2goal(self, target):
        """Moves the turtle to the goal."""
        goal_pose = Pose()
        goal_pose.x = target.x
        goal_pose.y = target.y
        distance_tolerance = 0.1
        vel_msg = Twist()
        while self.euclidean_distance(goal_pose) >= float(distance_tolerance):
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.ontarget = 1



if __name__ == '__main__':
    try:
        x = random.randint(0, 11)   
        y = random.randint(0, 11)
        ang = random.randint(0,31)
        cmd = "rosservice call /kill 'turtle1'" #kills the default turtle
        os.system(cmd)
        cmd = "rosservice call /spawn " + str(x) + " " +  str(y) + " " + str(ang/10) + " 'robot1'"
        os.system(cmd)

        x = TurtleBot()
        time.sleep(1)   #important, without this goal position doesn't update in time
        x.find_target()
        cmd = "rosnode kill 'mytarget'" #terminates the process
        os.system(cmd)
        cmd = "rosnode kill 'sim'" #terminates the process
        os.system(cmd)
    except rospy.ROSInterruptException:
        pass

# sometimes the robots "bounce" which is due to the angle reaching a weird value and the
#  chasing/escaping algorythm not being sure what to do
#solution - change steering algorythm 