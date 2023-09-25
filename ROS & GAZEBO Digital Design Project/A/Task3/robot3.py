#!/usr/bin/env python3
import rospy
import os
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import random
import time
from xml.dom import minidom
import xml.etree.cElementTree as ET

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
        self.end_x = 0
        self.end_y = 0
        self.spawn_x = random.randint(0, 11)   
        self.spawn_y = random.randint(0, 11)

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
        cmd = "rosservice call /kill 'mytarget'"
        os.system(cmd)
        cmd = "rosnode kill 'sim'" #terminates the process
        os.system(cmd)
        cmd = "rosnode kill 'mytarget'" #terminates the process
        os.system(cmd)

    def parseXML(self, xmlfile):
        xmldoc = minidom.parse(xmlfile)
        robot_pos = xmldoc.getElementsByTagName("robot_init_pos")[0]
        print(robot_pos.firstChild.data)

        target_pos = xmldoc.getElementsByTagName("target_init_pos")[0]
        print(target_pos.firstChild.data)

        moves = xmldoc.getElementsByTagName("move")
        for m in moves:
            m_id = m.getAttribute("number")
            robot = m.getElementsByTagName("robot")[0]
            target = m.getElementsByTagName("target")[0]
            print("Move:%s, Robot:%s, Target:%s" % (m_id, robot.firstChild.data, target.firstChild.data))

    def saveXML(self, xmlfile):
        root = ET.Element("Simulation")
        ET.SubElement(root, "robot_init_pos").text = "1,1"
        ET.SubElement(root, "target_init_pos").text = "9,9"
        doc = ET.SubElement(root, "trial")

        ET.SubElement(doc, "number").text = "1"
        ET.SubElement(doc, "move", name = "number").text = "1"
        ET.SubElement(root, "capture_pos").text = str(self.end_x) + "," + str(self.end_y)

        tree = ET.ElementTree(root)
        tree.write(xmlfile)


if __name__ == '__main__':
    x = TurtleBot()
    try:
        x.parseXML('/home/S638237/catkin_ws/src/ZbigniewBarczyk_638237_ACW1/A/Task3/trial.xml')
        
    except:
        print("no previous data")
    cmd = "rosservice call /kill 'turtle1'"
    os.system(cmd)

    cmd = "rosservice call /spawn " + str(x.spawn_x) + " " +  str(x.spawn_y) + " 0 'robot1'"
    os.system(cmd)

    time.sleep(1)   #important, without this goal position doesn't update in time
    while x.ontarget == 0:
        x.move2goal()
    x.end_x = x.self_pose.x
    x.end_y = x.self_pose.y
    x.saveXML('/home/S638237/catkin_ws/src/ZbigniewBarczyk_638237_ACW1/A/Task3/output.xml')

