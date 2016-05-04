#!/usr/bin/env python

import rospy
import actionlib
import time
import sys
import threading
import math
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal, ExploreTaskActionResult
from p2os_msgs.msg import MotorState
from random import randint
from lib.Fembot import Fembot
import time

#note, this will be ignored with a command line argument specifying hide time
#and our bash script to run the hider sends this command line argument (1)!!!
#so modify the value there instead of here!!!
DEFAULT_HIDE_TIME = 60 # 1 minutes

#note, this is also overriden by command line argument (2)!!!!
#so modify in the run_hider script!!!
EXPLORATION_TYPE = "frontier_exploration"
find_radius = 1.0

seq_id = 0
rate = None
exploration_client = None
interrupt_exploration = False
velPub = None
initial_time = 0

robot = Fembot()

def hiderOdometryHandler(data):
    global robot, interrupt_exploration, find_radius, initial_time
    pose = data.pose.pose

    if robot.pose == None:
        print "waiting for robot pose..."
        return

    x1 = pose.position.x
    y1 = pose.position.y
    x2 = robot.pose.position.x
    y2 = robot.pose.position.y

    #only "found" the hider if we are within a certain radius
    #e.g. we hear or see the hideR???
    if robot.dist(x1, y1, x2, y2) <= find_radius:
        #we found yoU!!!
        stopExploration()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()

        goal.target_pose.pose.position.x = x1
        goal.target_pose.pose.position.y =  y1

        goal.target_pose.pose.orientation.x = pose.orientation.x
        goal.target_pose.pose.orientation.y = pose.orientation.y
        goal.target_pose.pose.orientation.z = pose.orientation.z
        goal.target_pose.pose.orientation.w = pose.orientation.w

        print "found her! navigating..."
        curr_time = int(round(time.time() * 1000))
        print "time taken to find:", curr_time - initial_time, "milliseconds"

        goal_client = actionlib.SimpleActionClient("robot_0/move_base", MoveBaseAction)
        goal_client.wait_for_server()
        goal_client.send_goal_and_wait(goal)

        curr_time = int(round(time.time() * 1000))
        print "time taken to NAVg:", curr_time - initial_time, "milliseconds"


def odometryHandler(data):
    global robot
    robot.setPose(data.pose.pose)

# checks the laser scan values and updates the values in the dictionary
# containing potential hidings spots
def scanHandler(data):
    global robot

    robot.laser = data

def isFrontierExploration():
    return EXPLORATION_TYPE == "frontier_exploration"

def isWallFollowBreaking():
    return EXPLORATION_TYPE == "wall_follow_break"

def startExploration():
    global goal_status, exploration_client, rate, interrupt_exploration, seq_id
    if interrupt_exploration: return

    if isFrontierExploration():
        exploration_client = actionlib.SimpleActionClient("robot_0/explore_server", ExploreTaskAction)
        print "waiting for the exploration server..."
        exploration_client.wait_for_server()

        exploration_goal = ExploreTaskGoal()
        seq_id += 1
        exploration_goal.explore_boundary.header.seq = seq_id
        exploration_goal.explore_boundary.header.frame_id = "map"
        exploration_goal.explore_center.point.x = 0
        exploration_goal.explore_center.point.y = 0
        exploration_goal.explore_center.point.z = 0

        exploration_client.send_goal(exploration_goal)
        print "sent the exploration goal... waiting..."
        exploration_client.wait_for_result()
        print "exploration goal 'complete'"

    if isWallFollowBreaking():
        pass

    initial_time = int(round(time.time() * 1000))

def continueExploration():
    global robot, velPub
    if isFrontierExploration(): return
    if isWallFollowBreaking():
        twist = Twist()
        twist = robot.navigate(twist)
        velPub.publish(twist)

def stopExploration():
    global exploration_client, interrupt_exploration
    interrupt_exploration = True
    if isFrontierExploration():
        exploration_client.cancel_all_goals()

    if isWallFollowBreaking():
        pass

def main(args):
    global rate, interrupt_exploration, velPub, EXPLORATION_TYPE

    #note using args[0] because we already spliced out the program name
    if len(args) > 0:
        EXPLORATION_TYPE = args[0]
    rospy.init_node('fembots_finder_sim')
    rate = rospy.Rate(10)

    rospy.Subscriber("/robot_0/odom", Odometry, hiderOdometryHandler)
    rospy.Subscriber("/robot_1/odom", Odometry, odometryHandler)
    rospy.Subscriber("/robot_1/base_scan", LaserScan, scanHandler)

    velPub = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size=10)

    print "Explore Mode:", EXPLORATION_TYPE

    # MAIN LOOP
    startExploration()
    while not rospy.is_shutdown() and not interrupt_exploration:
        continueExploration()
        rate.sleep()


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
