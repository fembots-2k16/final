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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal, ExploreTaskActionResult
from p2os_msgs.msg import MotorState
from random import randint

seq_id = 0
rate = None
goal_client = None
exploration_client = None
explore_status = None
goal_status = 0
interrupt_exploration = False
odom_pose = None
robot_pose = None
initial_pose = None

hidingDistance = 0.5
hidingMap = {1:[], 2:[], 3:[]}
chosenHidingSpot = None

def moveBaseActionResultHandler(data):
    global goal_status
    goal_status = data.status.status
    #print "status: ", status
    #print data.result
    #http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html

def exploreTaskActionResultHandler(data):
    global explore_status
    explore_status = data.status.status


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

def scanHandler(data):
    ranges = data.ranges
    left = ranges[len(ranges)-1]
    middle = ranges[int(len(ranges)/2)]
    right = ranges[0]
    levelOfHiding = 0

    if left < hidingDistance:
        levelOfHiding += 1
    if middle < hidingDistance:
        levelOfHiding += 1
    if right < hidingDistance:
        levelOfHiding += 1

    if levelOfHiding > 0:
        global robot_pose, hidingMap
        hidingMap[levelOfHiding].append(robot_pose)


def startExploration():
    global goal_status, exploration_client, rate, interrupt_exploration, seq_id
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

    exploration_client.send_goal(exploration_goal)
    print "sent the exploration goal... waiting..."
    exploration_client.wait_for_result()
    print "exploration goal 'complete'"

def chooseHidingSpot():
    global hidingMap, chosenHidingSpot
    hidingSpots = []
    if(len(hidingMap[3]) > 0):
        hidingSpots = hidingMap[3]
    elif(len(hidingMap[2]) > 0):
        hidingSpots = hidingMap[2]
    elif(len(hidingMap[1]) > 0):
        hidingSpots = hidingMap[1]
    else
        # choose some random bs spot i guess? this should pretty much never happen

    index = randint(0, len(hidingSpots)-1)
    chosenHidingSpot = hidingSpots[index]

def main():
    global goal_client, exploration_client, rate, status
    rospy.init_node('homework5_navigator')
    rate = rospy.Rate(10)

    rospy.Subscriber("/pose", Odometry, odometryHandler)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseHandler)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, moveBaseActionResultHandler)
    rospy.Subscriber("/explore_server/result", ExploreTaskActionResult, exploreTaskActionResultHandler)
    rospy.Subscriber("/scan", LaserScan, scanHandler)


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

    #TODO
    # after some fixed time kill exploration server
    # choose a hiding spot and navigate there
    # see chooseHidingSpot

    # MAIN LOOP
    print "exploration!"
    startExploration()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
