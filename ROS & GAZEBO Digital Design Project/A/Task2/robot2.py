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
        self.target_subscriber = rospy.Subscriber('/mytarget/pose', Pose, self.update_targetpose)
        self.self_pose = Pose()
        self.target_pose = Pose()
        self.rate = rospy.Rate(10)
        self.ontarget = 0

    def update_pose(self, data):
        self.self_pose = data
        #print(data)
        self.self_pose.x = round(self.self_pose.x, 4)
        self.self_pose.y = round(self.self_pose.y, 4)

    def update_targetpose(self, data):
        self.target_pose = data
        #print(data)
        self.target_pose.x = round(self.target_pose.x, 4)
        self.target_pose.y = round(self.target_pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.self_pose.x), 2) +
                    pow((goal_pose.y - self.self_pose.y), 2))
    def linear_vel(self, goal_pose, constant=0.5):
        #return constant * self.euclidean_distance(goal_pose)
        return 2
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.self_pose.y, goal_pose.x - self.self_pose.x)
    def angular_vel(self, goal_pose, constant=2):
        return constant* (self.steering_angle(goal_pose) - self.self_pose.theta)


    def move2goal(self):
        """Moves the turtle to the goal."""
        distance_tolerance = 0.1
        vel_msg = Twist()
        while self.euclidean_distance(self.target_pose) >= float(distance_tolerance):
            vel_msg.linear.x = self.linear_vel(self.target_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(self.target_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.ontarget = 1
        cmd = "rosservice call /kill 'mytarget'" #move to launch file
        os.system(cmd)
        cmd = "rosnode kill 'sim'" #terminates the process
        os.system(cmd)
        cmd = "rosnode kill 'mytarget'" #terminates the process
        os.system(cmd)

if __name__ == '__main__':
    try:
        x = random.randint(0, 11)   
        y = random.randint(0, 11)
        ang = random.randint(0,31)
        cmd = "rosservice call /kill 'turtle1'"
        os.system(cmd)
        cmd = "rosservice call /spawn " + str(x) + " " +  str(y) + " " + str(ang/10) + " 'robot1'"
        os.system(cmd)

        x = TurtleBot()
        time.sleep(1)   #important, without this goal position doesn't update in time
        while x.ontarget == 0:
            x.move2goal()
    except rospy.ROSInterruptException:
        pass

#robot will bounce off walls sometimes, 
#when it tries to turn for the target, gets away from it, tries turning back and loop
# also just bounce when the target is directly in line, 
#but the angle is weird
#target also seems to bounce always at the same time?
#