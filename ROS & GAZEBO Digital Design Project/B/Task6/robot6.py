#!/usr/bin/env python3
from inspect import _void
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import pow, atan2, sqrt

range_left = Float32()
range_right = Float32()
range_centre = Float32()
range_centre_left = Float32()
range_centre_right = Float32()
range_centre_left_left = Float32()
range_centre_centre_left = Float32()

regions = {
    'range_right': 0,
    'range_centre_right': 0,
    'range_centre': 0,
    'range_centre_left': 0,
    'range_centre_centre_left': 0,
    'range_centre_left_left': 0,
    'range_left': 0,
}


def distance_info(msg):
    range_right.data = msg.ranges[0]
    range_centre_right.data = msg.ranges[45]
    range_centre.data = msg.ranges[90]
    range_centre_centre_left.data = msg.ranges[112]
    range_centre_left.data = msg.ranges[135]
    range_centre_left_left.data = msg.ranges[157]
    range_left.data = msg.ranges[179]



def main():
    rospy.init_node("controller", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, distance_info)
    ang_vel = 0
    lin_vel = 0.05

    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0


    #publishes velocity and angle (after checking it)
    def update_vel(lin_vel, ang_vel):   #pos ang is left, pos vel is forward
        maxvel = 0.1
        maxang = 0.15
        if abs(lin_vel) > maxvel:
            if lin_vel > maxvel:
                lin_vel = maxvel
            else:
                lin_vel = -maxvel
        if abs(ang_vel) > maxang:
            if ang_vel > maxang:
                ang_vel = maxang
            else:
                ang_vel = -maxang
        #print(ang_vel)
        #print(-lin_vel)

        cmd_vel = Twist()
        cmd_vel.linear.x = -lin_vel
        cmd_vel.angular.z = ang_vel
        pub.publish(cmd_vel)


    while not rospy.is_shutdown():
        lin_vel = 0.06  #default speed
        ang_vel = 0
        ang_vel = -(range_left.data*sqrt(2) - range_centre_left.data)*10
            #angle for following the wall in regular circumstances


        #if range_centre_left.data > 0.4:  #too far from left
            #ang_vel = (range_left.data - 0.15)/5
            #lin_vel = 0.05

        #if range_centre_left_left.data > 0.3:
            #ang_vel = (range_left.data - 0.15)/4
            #lin_vel = 0.04

        if range_left.data > 0.25:  #too far from left
            ang_vel = (range_left.data - 0.15)/1.5
            lin_vel = 0.06



        if range_right.data < 0.14: #too close to right
            ang_vel = (0.017/(range_right.data+0.0000001))
            lin_vel = 0.05

        if range_centre_right.data < 0.17:
            ang_vel = (0.017/(range_centre_right.data+0.0000001))
            lin_vel = 0.05




        if range_centre_centre_left.data < 0.2:  #too close to left
            ang_vel = (-0.014/(range_centre_centre_left.data+0.0000001))
            lin_vel = 0.03

        if range_centre_left.data < 0.18:
            ang_vel = (-0.013/(range_centre_left.data+0.0000001))
            lin_vel = 0.05

        if range_centre_left_left.data < 0.14: 
            ang_vel = (-0.011/(range_centre_left_left.data+0.0000001))
            lin_vel = 0.05

        if range_left.data < 0.1:
            ang_vel = (-0.005/(range_left.data+0.0000001))
            lin_vel = 0.03



        if range_centre.data < 0.23: #too close ahead
            lin_vel = 0.01
            ang_vel = -1

        update_vel(lin_vel, ang_vel)

if __name__ == '__main__':
    main()

# solving maze:.