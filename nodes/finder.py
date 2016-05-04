#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from apriltags_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal, ExploreTaskActionResult
from p2os_msgs.msg import MotorState
import math

seq_id = 0
rate = None
goal_client = None
exploration_client = None
interrupt_exploration = False
found_ids = {}
odom_pose = None
robot_pose = None
initial_pose = None
initial_time = None

def tagDetectionsHandler(data):
    global found_ids, odom_pose, rate, interrupt_exploration
    global exploration_client, initial_pose, initial_time

    if initial_pose == None: return
    if exploration_client == None: return

    detections = data.detections

    for detection in detections:
        id = detection.id
        if id not in found_ids:
            print "we found an april tag!"
            print "tag id:", id
            size = detection.size
            #tag detections pose
            april_pose = detection.pose.pose

            interrupt_exploration = True
            print "interrupt exploration from april tags handler"
            exploration_client.cancel_all_goals()

            #MATH for finding global point from april tag
            #relative tag posiiton values
            april_x = april_pose.position.x
            april_z = april_pose.position.z
            #calculate the angle as the arctan of z offset / x offset
            #so this angle is relative from viewer, with 90 deg being head on?
            april_theta = math.atan(april_z / april_x)

            #nothing fancy here, just getting current robot position
            ori = robot_pose.orientation
            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y
            robot_theta = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))

            # if robot is  0 degrees, rel x position is april.z
            #                        rel y position is april.x (negative)
            # if robot is 90 degrees, rel x position is april x
            #                        rel y position is april z
            # if robot is 180 degees, rel x position is april z (negative)
            #                        rel y position is april x
            # if robot is 270 degees, rel x position is april x (negative)
            #                        rel y position is april z (negative)
            # i figured this out by drawing a coordinate system picture and a truth table thing
            x_offset = (math.cos(robot_theta)*april_z) + (math.sin(robot_theta)*april_x)
            y_offset = (math.sin(robot_theta)*april_z) + (-math.cos(robot_theta)*april_x)

            # additionally, put some buffer room (0.5 meters) in between robot and apriltag
            x_buffer = (math.cos(robot_theta)*(-0.5))
            y_buffer = (math.sin(robot_theta)*(-0.5))

            #finalize the goal coordinates
            goal_x = robot_x + x_offset + x_buffer
            goal_y = robot_y + y_offset + y_buffer
            goal_theta = robot_theta
            goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, goal_theta)

            #create the goal!
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.get_rostime()

            #set the goal values
            goal.target_pose.pose.position.x = goal_x
            goal.target_pose.pose.position.y = goal_y
            goal.target_pose.pose.orientation.x = goal_quaternion.x
            goal.target_pose.pose.orientation.y = goal_quaternion.y
            goal.target_pose.pose.orientation.z = goal_quaternion.z
            goal.target_pose.pose.orientation.w = goal_quaternion.w

            if goal_client == None:
                goal_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
                goal_client.wait_for_server()

            goal_client.send_goal(goal)
            print "sending goal and waiting"

            #wait for the goal to finish!!!
            goal_client.wait_for_result()

            curr_time = int(round(time.time() * 1000))
            print "found ya!"
            print "time:", curr_time - initial_time

def initialPoseHandler(data):
    global initial_pose
    initial_pose = data.pose.pose

def odometryHandler(data):
    global odom_pose, robot_pose, initial_pose
    odom_pose = data.pose.pose

    if initial_pose != None:
        robot_pose = odom_pose
        robot_pose.position.x += initial_pose.position.x
        robot_pose.position.y += initial_pose.position.y
        robot_pose.position.z += initial_pose.position.z

        #tbh why are quarternions used and not just roll pitch yaw :((((((
        #tbh honestly i'm crying cute cat
        robot_pose.orientation.x += initial_pose.orientation.x
        robot_pose.orientation.y += initial_pose.orientation.y
        robot_pose.orientation.z += initial_pose.orientation.z
        robot_pose.orientation.w += initial_pose.orientation.w


def startExploration():
    global exploration_client, rate, interrupt_exploration, seq_id
    if interrupt_exploration:
        return

    exploration_client = actionlib.SimpleActionClient("explore_server", ExploreTaskAction)
    print "waiting for the exploration server..."
    exploration_client.wait_for_server()

    exploration_goal = ExploreTaskGoal()
    seq_id += 1
    exploration_goal.explore_boundary.header.seq = seq_id
    exploration_goal.explore_boundary.header.frame_id = "map"
    exploration_goal.explore_center.point.x = 1
    exploration_goal.explore_center.point.y = -5
    exploration_goal.explore_center.point.z = 0
    print "explore boundary "
    print exploration_goal.explore_boundary.polygon.points
    print len(exploration_goal.explore_boundary.polygon.points)

    exploration_client.send_goal(exploration_goal)
    print "sent the exploration goal... waiting..."
    exploration_client.wait_for_result()
    print "exploration goal 'complete'"


def main():
    global goal_client, exploration_client, rate, initial_time
    rospy.init_node('fembots_finder')
    rate = rospy.Rate(10)

    rospy.Subscriber("/pose", Odometry, odometryHandler)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseHandler)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tagDetectionsHandler)


    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)

    motor = MotorState()
    motor.state = 1

    print "starting motors..."
    for i in xrange(20):
        motPub.publish(motor)
        rate.sleep()

    print "\n--------------------------------------"
    print "set the initial pose in rviz ya fool!!!"
    print "--------------------------------------\n"
    while initial_pose == None:
        rate.sleep()

    initial_time = int(round(time.time() * 1000))

    #TODO
    # MAIN LOOP
    print "exploration!"
    startExploration()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
