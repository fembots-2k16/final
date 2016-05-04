#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.msg import Odometry
import sys

started_broadcasting = False
robot_num = 0

def sendTransform(msg, base_link_frame_name):
    #this seems silly, but for whatever reason amcl is not broadcasting transform from base_link to map
    #so since the pose of the robot in simulation matches the map frame,
    #let's just force publish a tf!!
    br = tf.TransformBroadcaster()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion((ori.x, -ori.y, -ori.z, ori.w))
    theta = euler[2]
    x = (-math.cos(theta)*pos.x) + (math.sin(theta)*pos.y)
    y = (-math.sin(theta)*pos.x) + (-math.cos(theta)*pos.y)
    br.sendTransform((x, y, pos.z),
                     #tf.transformations.quaternion_from_euler(0, 0, math.radians(180)),
                     (ori.x, -ori.y, -ori.z, ori.w),
                     rospy.Time.now(),
                     "/map",
                     base_link_frame_name)

def robotPoseHandler(msg):
    global started_broadcasting, robot_num

    sendTransform(msg, "robot_"+str(robot_num)+"/base_link")

    if not started_broadcasting:
        started_broadcasting = True
        print "\tamcl fake broadcast for robot", robot_num, "started!"

if __name__ == '__main__':
    if len(sys.argv) == 2:
        robot_num = int(sys.argv[1])

    rospy.init_node('fake_amcl_'+str(robot_num))
    rospy.Subscriber('/robot_'+str(robot_num)+'/odom', Odometry, robotPoseHandler)
    rospy.spin()
