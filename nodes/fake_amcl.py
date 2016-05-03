#!/usr/bin/env python
import rospy

import tf
import math
from nav_msgs.msg import Odometry

def odometryHandler(msg):
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
                     "map",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('fake_amcl')
    rospy.Subscriber('/pose', Odometry, odometryHandler)
    rospy.spin()
