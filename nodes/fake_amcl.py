#!/usr/bin/env python
import rospy

import tf
from nav_msgs.msg import Odometry

def odometryHandler(msg):
    #this seems silly, but for whatever reason amcl is not broadcasting transform from base_link to map
    #so since the pose of the robot in simulation matches the map frame,
    #let's just force publish a tf!!
    br = tf.TransformBroadcaster()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    br.sendTransform((pos.x, pos.y, pos.z),
                     (ori.x, ori.y, ori.z, ori.w),
                     rospy.Time.now(),
                     "map",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('fake_amcl')
    rospy.Subscriber('/pose', Odometry, odometryHandler)
    rospy.spin()
