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
hidingMap = {1:None, 2:None, 3:None}
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

# calculate the distance between the initial pose and the current pose
# uses x and y values and the distance formula
def getDistance(current_pose):
    global initial_pose
    init_x = initial_pose.position.x
    init_y = initial_pose.position.y
    current_x = current_pose.position.x
    current_y = current_pose.position.y
    diff1 = math.pow(current_x - init_x, 2)
    diff2 = math.pow(current_y - init_y, 2)
    return math.sqrt(diff1 + diff2) # return the distance

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

# checks the laser scan values and updates the values in the dictionary
# containing potential hidings spots
def scanHandler(data):
    global chosenHidingSpot
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
        distance = getDistance(robot_pose)
        print "distance ", distance
        if (hidingMap[levelOfHiding] != None): # if a spot was previously saved
            # check if the current spot is farther from the previously saved spot
            if (distance > hidingMap[levelOfHiding][1]):
                hidingMap[levelOfHiding] = [robot_pose, distance]
        else:
            hidingMap[levelOfHiding] = [robot_pose, distance]
        chosenHidingSpot = robot_pose    


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
    if(hidingMap[3] != None): # 3 walls
        chosenHidingSpot = hidingMap[3]
    elif(hidingMap[2] != None): # 2 walls
        chosenHidingSpot = hidingMap[2] 
    elif(hidingMap[1] != None): # 1 wall
        chosenHidingSpot = hidingMap[1] 
    #if no hiding spots are found, the chosenHidingSpot variable won't get set
    # in this case the robot will just stay in its last position

    #index = randint(0, len(hidingSpots)-1)
    #chosenHidingSpot = hidingSpots[index]

def setHidingTargetPose():
    global chosenHidingSpot
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.get_rostime()
    
    if (chosenHidingSpot != None):
        goal.target_pose.pose.position.x = chosenHidingSpot.position.x
        goal.target_pose.pose.position.y =  chosenHidingSpot.position.y
        goal.target_pose.pose.orientation.w = 1.0
            
        print "sending hiding goal! you'll never catch me now..."
        client.send_goal(goal)

        print "Result:", client.get_result()
    

def waitForHideTime(hide_time):
    global exploration_client
    for i in range(0, hide_time):
        if (i%10 == 0): #print sometimes just so we know the loop is working
            print 'hiding'
        time.sleep(1)
    print 'time\'s up! got to hide...' 
    exploration_client.cancel_all_goals()
    print 'going to hide'  
    chooseHidingSpot()  
    setHidingTargetPose()

def main(args):
    global goal_client, exploration_client, rate, status
    
    hide_time = 60 # default hide time 60 seconds
    if (len(args) > 0):
        hide_time = int(args[0])
    print 'set hide time to ' ,hide_time
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
    try:
        t = threading.Thread(target = waitForHideTime,args=(hide_time,))
        t.start()
    except:
        print 'Error: couldn\'t create thread'
    print "exploration!"
    startExploration()
    

if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
