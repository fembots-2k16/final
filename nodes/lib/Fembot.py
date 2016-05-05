import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Fembot:
    def __init__(self):
        self.pose = None
        self.prev_pose = None
        self.laser = None
        self.initial_pose = None
        self.angle_sensitivity = 40
        self.proximity_sensitivity = 0.4
        self.movement_amt = 7
        self.is_wall_searching = False
        self.is_spinning = 0
        self.is_stuck = 0
        self.was_i_turning_left = False
        self.get_off_the_wall = False
        self.getting_off_the_wall = 0
        self.non_wall_following_turn_direction = "left"
        self.moving_forward = 0
        self.queue = []
        self.prev_queue = []
        self.angle = 90
        self.count = 0
        self.reverse_flag = False
        self.flip_flag = False
        self.just_started_turning = False
        self.looking_for_right = False
        self.total_dist = 0

    def setPose(self, pose):
        if self.initial_pose == None:
            self.initial_pose = pose

        if self.prev_pose != None:
            dist = self.getDistanceTraveledDelta()
            if dist < 0.001:
                self.is_stuck += 1
            else: self.is_stuck = 0

        self.prev_pose = self.pose
        self.pose = pose

    def switchTurnDirection(self):
        if self.non_wall_following_turn_direction == "left":
            self.non_wall_following_turn_direction = "right"
        else: self.non_wall_following_turn_direction = "left"

    def getDistanceFromInitial(self):
        x1 = self.initial_pose.position.x
        y1 = self.initial_pose.position.y

        x2 = self.pose.position.x
        y2 = self.pose.position.y
        return self.dist(x1, y1, x2, y2)

    def getDistanceTraveled(self):
        x1 = self.initial_pose.position.x
        y1 = self.initial_pose.position.y

        x2 = self.pose.position.x
        y2 = self.pose.position.y

        return self.dist(x1, y1, x2, y2)

    def getDistanceTraveledDelta(self):
        x1 = self.prev_pose.position.x
        y1 = self.prev_pose.position.y

        x2 = self.pose.position.x
        y2 = self.pose.position.y

        #print "x1:",x1,",y1:",y1,",x2:",x2,",y2:",y2

        dist = self.dist(x1, y1, x2, y2)
        self.total_dist += dist
        return dist

    def dist(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))

    def hasFrontObstacle(self):
        return self.hasObstacle(90, self.angle_sensitivity, self.proximity_sensitivity)
    def hasLeftObstacle(self):
        return self.hasObstacle(180, 45, self.proximity_sensitivity)
    def hasRightObstacle(self):
        return self.hasObstacle(0, 45, self.proximity_sensitivity)
    def hasObstacle(self, angle, offset, s):
        has_obstacle = False
        i = angle - offset
        while i < angle + offset:
            if i < 0:
                i += 1
                continue
            if i >= len(self.laser.ranges): break
            has_obstacle = has_obstacle or (self.laser.ranges[i] < s)
            i += 1
        return has_obstacle

    def printQueue(self):
        if len(self.queue) != len(self.prev_queue):
            print self.queue
        self.prev_queue = self.queue[0:]

    def orientationToAngle(self, orient):
        angel = 2*math.degrees(math.asin(orient))
        if angel < 0:
            angel += 360
        return angel

    def angleToOrientation(self, angle):
        #angle = angle % 360
        mult = 1
        if angle > 180: mult = -1
        orient = math.sin(math.radians(angle/2))
        if mult < 0:
            orient = mult*orient
        return orient

    def round(self, x):
        f = math.floor(x)
        c = math.ceil(x)
        fdelta = abs(x-f)
        cdelta = abs(x-c)
        if fdelta < cdelta: return f
        return c

    def turn(self, amt, dir):
        self.just_started_turning = True
        #print "turn,",amt,self.angle,self.angle+amt
        return ["turn", False, self.angle+amt, dir]
    def leftTurn(self):
        return self.turn(90, 1)
    def rightTurn(self):
        return self.turn(-90, -1)
    def properDiffAngle(self, curr_angle, desired_angle, dir):
        #angels
        if dir > 0:
            if self.count < 0:
                if curr_angle > 270:
                    curr_angle -= 360
                else:
                    self.count = 0
            elif curr_angle < 0 or (self.count > 0 and curr_angle < desired_angle):
                curr_angle += 360
        else:
            if (curr_angle > 330 or self.count > 0):
                curr_angle -= 360
                if self.count >= 0:
                    self.count += 1

        return curr_angle - desired_angle


    def moveForward(self, dist, amt):
        return ["move", False, dist, amt, self.pose.position.x, self.pose.position.y]
    def properDiffDist(self, x1, y1, x2, y2):
        if self.angle == 0:
            return x1 - x2
        if self.angle == 90:
            return y1 - y2
        if self.angle == 180:
            return x2 - x1
        if self.angle == 270:
            return y2 - y1

    def tryBreakWall(self):
        return ["tryBreakWall", True]

    def tryStartWall(self):
        return ["tryStartWall", True]

    def handleQueue(self, twist):
        item = self.queue[0]
        command = item[0]

        #print "command:",command
        if command == "tryBreakWall":
            if self.angle == 90:
                self.is_wall_searching = False
            print "bROKEN?", (not self.is_wall_searching)

        if command == "tryStartWall":
            self.is_wall_searching = True
            print "STARTED?", self.is_wall_searching

        if command == "move":
            if item[2] != 0:
                x1 = self.pose.position.x
                y1 = self.pose.position.y

                x2 = item[4]
                if self.angle == 0: x2 += item[2]
                elif self.angle == 180: x2 -= item[2]
                y2 = item[5]
                if self.angle == 90: y2 += item[2]
                elif self.angle == 270: y2 -= item[2]

                diff = self.properDiffDist(x1, y1, x2, y2)

                #print x1, y1, x2, y2, "diff", diff

                twist.linear.x = 1.0

                if diff > 0 or (item[2] > 0 and self.hasFrontObstacle()):
                    twist.linear.x = 0
                    self.queue[0][1] = True
            else:
                twist.linear.x = 1.0
                item[3] -= 1
                if item[3] <= 0:
                    twist.linear.x = 0
                    self.queue[0][1] = True


        if command == "turn":
            sensitivity = 0.9
            dr = self.queue[0][3]
            twist.angular.z = dr*0.9
            curr_angle = self.orientationToAngle(self.pose.orientation.z)
            if curr_angle > 270:
                if self.count >= 0:
                    self.flip_flag = True
                if self.just_started_turning and dr > 0:
                    self.flip_flag = False
                    self.count = -1
            if curr_angle < 45 and self.flip_flag:
                self.count += 1
            desired_angle = self.queue[0][2]
            diff = self.properDiffAngle(curr_angle, desired_angle, dr)
            if self.reverse_flag and dr > 0: self.count += 1
            if dr > 0:
                if diff > sensitivity:
                    self.reverse_flag = True
                    twist.angular.z = -0.1
                elif diff > 0:
                    twist.angular.z = 0
                    self.queue[0][1] = True
                    self.angle = desired_angle % 360
            if dr < 0:
                #print curr_angle, desired_angle, diff, self.count
                if diff < -1*sensitivity:
                    self.reverse_flag = True
                    twist.angular.z = 0.1
                elif diff < 0:
                    twist.angular.z = 0
                    self.queue[0][1] = True
                    self.angle = desired_angle % 360
            self.just_started_turning = False

        if self.queue[0][1]:
            self.queue = self.queue[1:]
            self.count = 0
            self.reverse_flag = False
            self.flip_flag = False
            self.just_started_turning = False
            #print "UNQUEUE"
        self.printQueue()
        return twist

    def navigateWallFollow(self, twist):
        if self.laser == None:
            print "waiting for laser scan..."
            return twist
        #print self.pose.orientation.z
        #return twist
        if len(self.queue) > 0:
            return self.handleQueue(twist)

        if not self.is_wall_searching:
            if not self.hasFrontObstacle():
                twist.linear.x = 0.5
            else:
                self.queue.append(self.leftTurn())
                self.queue.append(self.tryStartWall())

        #WALL SEARCHING
        else:
            if self.hasRightObstacle():
                self.looking_for_right = False
                if not self.hasFrontObstacle():
                    twist.linear.x = 1.0
                else:
                    print "QUEUE LEFT TURN"
                    self.queue.append(self.leftTurn())
            else:
                if self.looking_for_right:
                    if not self.hasFrontObstacle():
                        twist.linear.x = 1.0
                    else:
                        print "QUEUE LEFT TURN"
                        self.queue.append(self.leftTurn())
                        if self.is_stuck < 120:
                            self.looking_for_right = False
                else:
                    self.queue.append(self.moveForward(0, self.movement_amt))
                    self.queue.append(self.rightTurn())
                    #self.queue.append(self.tryBreakWall())
                    self.looking_for_right = True



        if len(self.queue) > 0:
            return self.handleQueue(twist)
        else: return twist

    def navigateWallFollowBreak(self, twist):
        if self.laser == None:
            print "waiting for laser scan..."
            return twist
        #print self.pose.orientation.z
        #return twist
        if len(self.queue) > 0:
            return self.handleQueue(twist)

        if not self.is_wall_searching:
            if not self.hasFrontObstacle():
                twist.linear.x = 0.5
            else:
                self.queue.append(self.leftTurn())
                self.queue.append(self.tryStartWall())

        #WALL SEARCHING
        else:
            if self.hasRightObstacle():
                self.looking_for_right = False
                if not self.hasFrontObstacle():
                    twist.linear.x = 1.0
                else:
                    print "QUEUE LEFT TURN"
                    self.queue.append(self.leftTurn())
            else:
                if self.looking_for_right:
                    if not self.hasFrontObstacle():
                        twist.linear.x = 1.0
                    else:
                        print "QUEUE LEFT TURN"
                        self.queue.append(self.leftTurn())
                        if self.is_stuck < 120:
                            self.looking_for_right = False
                else:
                    self.queue.append(self.moveForward(0, self.movement_amt))
                    self.queue.append(self.rightTurn())
                    self.queue.append(self.tryBreakWall())
                    self.looking_for_right = True



        if len(self.queue) > 0:
            return self.handleQueue(twist)
        else: return twist
