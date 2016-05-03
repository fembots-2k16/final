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
initial_time

def tagDetectionsHandler(data):
    global found_ids, odom_pose, rate, interrupt_exploration
    global exploration_client, initial_pose

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

            #MATH for finding global point from april tag (shout out to ryan)
            #relative tag posiiton
            april_x = april_pose.position.x
            april_z = april_pose.position.z

            angle = math.asin(april_x / april_z)
            x_diff = april_z * math.cos(angle)
            y_diff = april_z * math.sin(angle)
            #global tag position
            april_x = robot_pose.position.x + x_diff
            april_y = robot_pose.position.y + y_diff

            goal_x = april_x + 0.5*math.cos(-angle)
            goal_y = april_y + 0.5*math.sin(-angle)
            goal_z_theta = -angle

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.get_rostime()

            goal.target_pose.pose.position.x = goal_x
            goal.target_pose.pose.position.y = goal_y
            goal.target_pose.pose.orientation.z = goal_z_theta

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
    global goal_client, exploration_client, rate
    rospy.init_node('fembots_finder_sim')
    rate = rospy.Rate(10)

    rospy.Subscriber("/pose", Odometry, odometryHandler)
    #rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseHandler)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tagDetectionsHandler)


    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)

    motor = MotorState()
    motor.state = 1

    print "starting motors..."
    for i in xrange(20):
        motPub.publish(motor)
        rate.sleep()

    #print "\n--------------------------------------"
    #print "set the initial pose in rviz ya fool!!!"
    #print "--------------------------------------\n"
    #dummy out an initial pose because in simulation, pose returns relative to map coordinates!
    initial_pose = PoseWithCovarianceStamped().pose.pose
    #while initial_pose == None:
    #    rate.sleep()

    #TODO
    initial_time = int(round(time.time() * 1000))
    # MAIN LOOP
    print "exploration!"
    startExploration()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
