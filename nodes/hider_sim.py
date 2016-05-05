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

#note, this will be ignored with a command line argument specifying hide time
#and our bash script to run the hider sends this command line argument (1)!!!
#so modify the value there instead of here!!!
DEFAULT_HIDE_TIME = 60 # 1 minutes

#note, this is also overriden by command line argument (2)!!!!
#so modify in the run_hider script!!!
EXPLORATION_TYPE = "frontier_exploration"

seq_id = 0
rate = None
exploration_client = None
interrupt_exploration = False
farthest_position = [None, None]
velPub = None

hidingDistance = 0.5
hidingMap = {1:None, 2:None, 3:None}
chosenHidingSpot = None
robot = Fembot()

def odometryHandler(data):
    global robot
    robot.setPose(data.pose.pose)

# checks the laser scan values and updates the values in the dictionary
# containing potential hidings spots
def scanHandler(data):
    global chosenHidingSpot, farthest_position, robot, hidingMap

    robot.laser = data
    if robot.initial_pose == None:
        print "waiting for initial pose..."
        return

    ranges = data.ranges
    left = ranges[len(ranges)-1]
    middle = ranges[int(len(ranges)/2)]
    right = ranges[0]
    levelOfHiding = 0
    distance = robot.getDistanceFromInitial()

    if distance > farthest_position[1]:
        farthest_position = [robot.pose, distance]

    #left wall!
    if left < hidingDistance:
        levelOfHiding += 1
    #wall in front of!
    if middle < hidingDistance:
        levelOfHiding += 1
    #right wall!
    if right < hidingDistance:
        levelOfHiding += 1

    if levelOfHiding > 0:
        #print "distance ", distance
        if hidingMap[levelOfHiding] != None: # if a spot was previously saved
            # check if the current spot is farther from the previously saved spot
            if (distance > hidingMap[levelOfHiding][1]):
                hidingMap[levelOfHiding] = [robot.pose, distance]
                chosenHidingSpot = [robot.pose, distance]
        else:
            hidingMap[levelOfHiding] = [robot.pose, distance]
            chosenHidingSpot = [robot.pose, distance]

def isFrontierExploration():
    return EXPLORATION_TYPE == "frontier_exploration"

def isWallFollowBreaking():
    return EXPLORATION_TYPE == "wall_follow_break"

def isWallFollowing():
    return EXPLORATION_TYPE == "wall_follow"

def startExploration():
    global exploration_client, rate, interrupt_exploration, seq_id
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

    if isWallFollowing():
        pass
    if isWallFollowBreaking():
        pass

def continueExploration():
    global robot, velPub
    if isFrontierExploration(): return
    if isWallFollowing():
        twist = Twist()
        twist = robot.navigateWallFollow(twist)
        velPub.publish(twist)
    if isWallFollowBreaking():
        twist = Twist()
        twist = robot.navigateWallFollowBreak(twist)
        velPub.publish(twist)

def stopExploration():
    global exploration_client, interrupt_exploration
    interrupt_exploration = True
    if isFrontierExploration():
        exploration_client.cancel_all_goals()

    if isWallFollowing():
        pass

    if isWallFollowBreaking():
        pass

def chooseHidingSpot():
    global hidingMap, chosenHidingSpot, farthest_position
    hidingSpots = []
    how_many_walls = 0
    if hidingMap[3] != None: # 3 walls
        chosenHidingSpot = hidingMap[3]
        how_many_walls = 3
    elif hidingMap[2] != None: # 2 walls
        chosenHidingSpot = hidingMap[2]
        how_many_walls = 2
    elif hidingMap[1] != None: # 1 wall
        chosenHidingSpot = hidingMap[1]
        how_many_walls = 1
    # if no hiding spots are found, the chosenHidingSpot variable won't get set
    # in this case the robot will just stay in its last position

    if how_many_walls == 1 and chosenHidingSpot[1] < farthest_position[1]:
        chosenHidingSpot = farthest_position
    elif how_many_walls == 0:
        chosenHidingSpot = farthest_position

    #index = randint(0, len(hidingSpots)-1)
    #chosenHidingSpot = hidingSpots[index]

def setHidingTargetPose():
    global chosenHidingSpot
    goal_client = actionlib.SimpleActionClient("robot_0/move_base", MoveBaseAction)
    goal_client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.get_rostime()

    if (chosenHidingSpot != None and chosenHidingSpot[0] != None):
        goal.target_pose.pose.position.x = chosenHidingSpot[0].position.x
        goal.target_pose.pose.position.y =  chosenHidingSpot[0].position.y
        goal.target_pose.pose.orientation.w = 1.0

        print "sending hiding goal! you'll never catch me now..."
        print "hiding at position:", chosenHidingSpot[0].position.x, chosenHidingSpot[0].position.y
        print "\tdistance from initial:", chosenHidingSpot[1]
        goal_client.send_goal_and_wait(goal)

        print "Result:", goal_client.get_result()


def waitForHideTime(hide_time):
    time.sleep(hide_time)
    #for i in range(0, hide_time):
        #if (i%10 == 0): #print sometimes just so we know the loop is working
        #    print 'hiding, time:', i
    #    time.sleep(1)
    print 'time\'s up! got to hide...'

    stopExploration()

    print 'going to hide'
    chooseHidingSpot()
    setHidingTargetPose()

def main(args):
    global rate, DEFAULT_HIDE_TIME, interrupt_exploration, velPub, EXPLORATION_TYPE

    hide_time = DEFAULT_HIDE_TIME

    #note using args[0] and args[1] because we already spliced out the program name
    if len(args) > 0:
        hide_time = int(args[0])
        if len(args) > 1: EXPLORATION_TYPE = args[1]
    print 'set hide time to ' ,hide_time
    rospy.init_node('fembots_hider_sim')
    rate = rospy.Rate(10)

    rospy.Subscriber("/robot_0/odom", Odometry, odometryHandler)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, scanHandler)

    velPub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=10)

    print "Explore Mode:", EXPLORATION_TYPE

    # MAIN LOOP
    try:
        t = threading.Thread(target = waitForHideTime,args=(hide_time,))
        t.start()
    except:
        print 'Error: couldn\'t create thread'
    print "exploration!"
    startExploration()
    while not rospy.is_shutdown() and not interrupt_exploration:
        continueExploration()
        rate.sleep()


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
