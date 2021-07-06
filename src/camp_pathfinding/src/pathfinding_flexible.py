#!/usr/bin/env python
#----------------------------------------------------------------------------------------
# Original Author : Sean Carda
# Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
# Original Date   : 4/15/2021
# Description     : This code is adapted from an algorithm developed by Dr. Michael 
#                   McCourt. This code uses LiDAR and probabilistic map data to 
#                   generate waypoints for robotic pathfinding. 
#
#
#
# Change Log:
#
# 4/15/2021 - Sean Carda: Created script and initial methods.
#
# 5/5/2021  - Sean Carda: Finalized the code. It is entirely possible that more
#                         edits will be necessary.
#
# 6/25/2021 - Tyler Pigott: Tried to fix the map extention error
#
#
#----------------------------------------------------------------------------------------


# FOR DEVELOPMENT
# ---------------
# This code will subscribe to multiple Turtlebot topics such as its odometry and map.
# This code will use odometry to localize itself within the map. It will use map
# data to determine where to scan and where to generate waypoints.
# The current functionality outline of this code is as follows (this is subject to change):
# 1. Initialize a set of waypoints in one direction.
# 2. Check lidar distances.
# 3a. If no objects are detected to impede movement, travel to waypoint, shift waypoints, and create new waypoint.
# 3b. If an object is detected, reset the waypoint list. 

# Package imports.
import math
from typing import type_check_only
from geometry_msgs import msg
import roslib
import numpy as np
import rospy
import cv2
roslib.load_manifest('rospy')

# Message imports for object aquisition and use.
from geometry_msgs.msg import Vector3, Point, PointStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from camp_goto.msg import Cmd

class Waypoint:
    def __init__(self,num,x=0,y=0,theta=0):
        self.point = Point(x,y,0)
        self.heading = theta
        self.viz_publisher = rospy.Publisher('waypoint_'+str(num), PointStamped, queue_size = 10)
        
    def publish(self):
        # Result as a PointStamp.
        result_viz = PointStamped()
        result_viz.point = self.point
        result_viz.header.stamp = rospy.Time()
        result_viz.header.frame_id = "map"
        self.viz_publisher.publish(result_viz)

    def auto_adjust(self,map):
        # convert point from meters to grid squares (i don't int(round()) this because i do that in the l/r check)
        x = (self.point.x - map.info.origin.position.x)/map.info.resolution
        y = (self.point.y - map.info.origin.position.y)/map.info.resolution
        # set a unit vector normal to the heading
        unit_x = math.cos(self.theta + math.pi/2)
        unit_y = math.sin(self.theta + math.pi/2)
        # initialize left and right at 1m (20 squares)
        left = 20
        right = 20
        # if there is a wall closer than 1m, write the number of grid squares to left or right.
        # TODO: handle edge of map cases?
        for i in range(0,20):
            if left == 20 and map.data[int(round(x + i * unit_x)) + (map.info.width * int(round(y + i * unit_y)))] > 90:
                left = i
            if right == 20 and map.data[int(round(x - i * unit_x)) + (map.info.width * int(round(y - i * unit_y)))] > 90:
                right = i
        # find out how many meters we need to move the point to get it 0.5m from the wall
        adjust = (left - right)/2 * map.info.resolution
        # create adjustment vectors in meters for the x and y directions
        adjust_x = (unit_x * adjust)
        adjust_y = (unit_y * adjust)
        # change the point to the adjusted point
        self.point.x = self.point.x + adjust_x
        self.point.y = self.point.y + adjust_y
        print("points are " + str(left) + " and " + str(right) + " squares away on the left and right respectively")
        print("adjusted the point by "+str(adjust_x)+" x and "+str(adjust_y)+" y.")
        return [adjust_x,adjust_y]
    
    def quick_adjust(self,adjust):
        self.point.x = self.point.x + adjust[0]
        self.point.y = self.point.y + adjust[1]
        print("executed quick adjust")



class Pathfinding_Node:

    #--------------------------------------------------------------------------------------------------------------
    # Initialization of ROS attributes and global variables.
    #--------------------------------------------------------------------------------------------------------------
    def __init__(self): 
        # number of directions to look when deciding on a direction
        self.direction_count = 12
        #number of waypoints to generate
        self.waypoint_count = 4

        # Have ROS initialize this script as a node in rqt.
        rospy.init_node('pathfinding', anonymous = False)

        # Subscribe to map.
        rospy.Subscriber('/map', OccupancyGrid, self.updateMap)      
        
        # Subscribe to LiDAR.
        rospy.Subscriber('/scan', LaserScan, self.updateLidarScan)

        # Subscribe to odometry.
        rospy.Subscriber('/odom', Odometry, self.updateOdom)

        # Subscribe to backup state.
        rospy.Subscriber('/backup_state', Bool, self.updateBackup)

        # This will publish the computed waypoint information.
        self.point_publisher = rospy.Publisher('go_cmd', Cmd, queue_size = 10)
        
        # Instantiate publlishers for displaying the waypoints in rviz. This will be invaluable for debugging.
        # these moved to the new waypoint class
#         self.viz_publisher_1 = rospy.Publisher('point_viz_1', PointStamped, queue_size = 10)
#         self.viz_publisher_2 = rospy.Publisher('point_viz_2', PointStamped, queue_size = 10)
#         self.viz_publisher_3 = rospy.Publisher('point_viz_3', PointStamped, queue_size = 10)
#         self.viz_publisher_4 = rospy.Publisher('point_viz_4', PointStamped, queue_size = 10)
        self.region_publisher = rospy.Publisher('region', PointStamped, queue_size = 10)
        self.robot_publisher = rospy.Publisher('robot_publisher', PointStamped, queue_size = 10)
        

        # make waypoint_count number of waypoints
        self.waypoints = []
        self.waypoints = [Waypoint(i) for i in range(0,self.waypoint_count)]

        # Initialize important data types. For this script, we need access to the OccupancyGrid produced
        # by SLAM. We need the Lidar for obstacle detection. We need the odometry for positional data. 
        self.mapActual = OccupancyGrid() 
        self.lidar = LaserScan()                         
        self.odom = Odometry()

        # This is a check to prevent the robot from conducting too many reset calculations at once.
        self.reset = False
        
        # Track number of fails during a new waypoint calculation.
        self.fails = 0

        # Property which holds on to the satisfactory distance for when a new point should be generated.
        self.satisDist = 0.15 #meters

        self.entropyVector = [0, 0, 0, 0, 0, 0, 0, 0]

        self.backupOld = False

        self.backupNew = False


    #--------------------------------------------------------------------------------------------------------------
    # Subscription update methods.
    #--------------------------------------------------------------------------------------------------------------

    # This method will update the map data when new data is available. This methods grabs every paramater
    # from the generated map.
    def updateMap(self, data):
        self.mapActual = data

    # This method will update lidar data when new data will be available. This method grabs every parameter 
    # from the lidar node.
    def updateLidarScan(self, data):
        self.lidar = data

    # This method will grab information from the robot's odometry.
    def updateOdom(self, data):
        self.odom = data

    # Callback to update the backup state.
    def updateBackup(self, data):
        self.backupNew = data

    #--------------------------------------------------------------------------------------------------------------
    # Main Functionality of the Pathfinding algorithm
    #--------------------------------------------------------------------------------------------------------------
    def main(self):        
    
            # Method for obtaining the robot's position as a distance, in meters, relative to the SLAM-generated map.
        def getRoboMapPosition():
            # Use Odometry to get the robot's position.
            result = Vector3()
            result.x = self.odom.pose.pose.position.x
            result.y = self.odom.pose.pose.position.y

            # Return.
            return result


        # Method for obtaining the robot's position as a matrix position in the SLAM-generated map.
        def getMatrixPosition():
            x_map_offset = -1 * self.mapActual.info.origin.position.x # X origin of map object is -10. So, add 10 to robot X position.
            y_map_offset = -1 * self.mapActual.info.origin.position.y # Y origin of map object is -10. So, add 10 to robot Y position.
            
            # Get the robot position relative to the map
            robo_position = getRoboMapPosition()

            # Calculate the robot's position in meters from map object origin.
            position_in_meters = [x_map_offset + robo_position.x, 
                                  y_map_offset + robo_position.y]

            # Convert to matrix position. Used for map data traversing.
            position_in_units = [0, 0]
            while self.mapActual.info.resolution == 0:
                print('wait a sec...')
                rospy.sleep(0.2)
            for i in range(2):
                position_in_units[i] = round(position_in_meters[i] / self.mapActual.info.resolution, 0)
                position_in_units[i] = int(position_in_units[i])

            # Return.
            return position_in_units


        # Method for publishing waypoints to RQT. 
        def publishWaypoints():
            # removed this because waypoints are now stored in meters
            # # First get the waypoint, which is in units.
            # waypoint = self.waypoints.get(1)
            #
            # # Calculate the waypoint in meters. Cannot use getPointForPublish method as it returns
            # # a PointStamped. We want this published in a format that camp_goto to read: a Point.
            # x = (self.mapActual.info.resolution * waypoint.x) + self.mapActual.info.origin.position.x
            # y = (self.mapActual.info.resolution * waypoint.y) + self.mapActual.info.origin.position.y
            # goTo = Point(x, y, waypoint.z)

            command = Cmd()
            command.destination = self.waypoints[0].point
            command.stop = False
            command.is_relative = False
            command.is_deca = False

            command.speed = 0.43

            command.destination_stop_distance = 0
            command.emergency_stop_distance = 0.15
            command.emergency_stop_angle = 30

            self.point_publisher.publish(command)

            # Publish the points in rviz.
            #print("publishing...")
            for w in self.waypoints:
                w.publish()

            roboPosX = getRoboMapPosition().x
            roboPosY = getRoboMapPosition().y
            roboPos = PointStamped()
            roboPos.header.stamp = rospy.Time()
            roboPos.header.frame_id = "map"
            roboPos.point = Point(roboPosX, roboPosY, 1)

            self.robot_publisher.publish(roboPos)

        # This method replaces the resetWaypoints() function. It searches for a point of interest to travel to or uses the traditional reset function if a point of interest is already close by.
        def reevaluate():
            print("reevaluating...")
            # prep the data
            x = round(self.odom.pose.pose.position.x)
            y = round(self.odom.pose.pose.position.y)
            size = 1/self.mapActual.info.resolution
            # check the current meter square to see if there is still interesting data to be found. a normal reset is performed if this is the case.
            if poiCheck(x,y) > size:
                resetWaypoints()
            else:
                # in an increasing radius, check squares for poi potential. Give each square a value based on how often there are unknown and free areas touching and how far away it is. once a sufficiently valuable poi is found, stop.
                rad = 3
                max_poi = 0
                width = self.mapActual.info.width
                height = self.mapActual.info.height
                while max_poi < size/2 and rad < max(width,height)*1.8:
                    for m in range(-rad,rad+1):
                        for n in range(-rad,rad+1):
                            dist = m*m+n*n
                            if dist <= rad:
                                # this function makes it so an area with sixe or more edge points (out of size^2 pixels) has a value of size at distance 0 and 0.6*size at distance 10. 
                                poi = min(poiCheck(x+m,y+n),size) * (0.5 + 0.5 * math.exp(-dist*4/25))
                                if poi > max_poi:
                                    # take the highest score and set it as the target
                                    max_poi = poi
                                    max_x = x+m
                                    max_y = y+n
                    rad = rad + 2
                # now that a point of interest has been identified, we need to make waypoints leading to that POI.
                # for now, lets assume the point is on navigable terrain.
                # we will now pass the start and end info to our recursive pathfinding algorithm
                path = makePath([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,max_x,max_y],erode([self.mapActual.data,self.mapActual.info.width,self.mapActual.info.height],[[1 for i in range(9)],3,3]))
                self.waypoints = []
                lx = path[0]
                ly = path[1]
                for i in range(2,len(path/2)):
                    x = path[2*i]
                    y = path[2*i+1]
                    theta = math.atan2(y-ly,x-lx)
                    self.waypoints[i] = Waypoint(i,x,y,theta)
                    lx = x
                    ly = y

                    #TODO: make a bunch of waypoints, requires a review of waypoint infrastructure to allow for waypoint surges.
                # TODO:                
                #  if the target is unreachable, mark it as "explored"
                # when you get to the target, set it's square as explored.

        def makePath(path,map):
            size = 1/self.mapActual.info.resolution
            width = self.mapActual.info.width
            height = self.mapActual.info.height
            map = map[0]
            # get the data and convert to grid squares
            sx = path[0]
            sy = path[1]
            gx = path[2]
            gy = path[3]
            distance = math.sqrt((sx-gx)*(sx-gx)+(sy-gy)*(sy-gy))
            if distance < 0.2:
                # if endponts are already super close together, stop.
                return path

            # save a unit vector for ease of use later
            dx = (gx-sx)/distance
            dy = (gy-sy)/distance
            # search for non-free space in the path. 
            # Don't look too close to the POI because it could be in a wall or something
            wall = 0
            for i in range(0,round((distance-1)*size)):
                if map[(meter2grid(sx,'x')+dx*i) + (meter2grid(sy,'y')+dy*i)*width] > 90:
                    wall = 1
                    break
            if wall == 0:
                return path

            # expand each map until they overlap, 
            # then set the overlap as an intermediary waypoint and repeat on both sides
            startmap = []
            goalmap = []
            startmap = [0 for i in range(0,width*height)]
            goalmap = [0 for i in range(0,width*height)]
            startmap[meter2grid(sx,'x') + meter2grid(sy,'y')*width] = 1
            goalmap[meter2grid(gx,'x') + meter2grid(gy,'y')*width] = 1
            val = 0
            overlap = 0
            # build a circular kernel real quick
            kernel = []
            kernel = [0 for i in range(7*7)]
            for i in range(7*7):
                if (i%7-3)*(i%7-3) + (int(i/7)-3)*(int(i/7)-3) < 12:
                    kernel[i] = 1

            # keep looping until an overlap is found or the maps no longer change
            while overlap == 0:
                startmap = dialate(startmap,kernel)
                goalmap = dialate(goalmap,kernel)
                for m in range(width):
                    for n in range(height):
                        startmap[m+n*width] = startmap[m+n*width] and map[m+n*width]
                        goalmap[m+n*width] = goalmap[m+n*width] and map[m+n*width]
                        if (startmap[m+n*width] and goalmap[m+n*width]) == 1:
                            overlap = 1
                            mx = grid2meter(m,'x')
                            my = grid2meter(n,'y')
                            break
                    if overlap == 1:
                        break
                newval = sum(startmap) + sum(goalmap)
                if newval == val:
                    # something needs to happen here so the POI gets shrown out, but i don't know how yet. maybe errors or exceptions?
                    print("no path available")
                    return
            
            # handle recursion
            firsthalf = makePath([sx,sy,mx,my],map)
            secondhalf = makePath([mx,my,gx,gy],map)
            path = firsthalf + [secondhalf[i] for i in range(2,len(secondhalf))]
            return path

        
        # this function converts meters to grid squares
        def meter2grid(n,param):
            if param == 'x':
                return round((n - self.mapActual.info.origin.x) / self.mapActual.info.resolution)
            if param == 'y':
                return round((n - self.mapActual.info.origin.y) / self.mapActual.info.resolution)

        #this function does the opposite
        def grid2meter(n,param):
            if param == 'x':
                return self.mapActual.info.origin.x + n * self.mapActual.info.resolution
            if param == 'y':
                return self.mapActual.info.origin.y + n * self.mapActual.info.resolution

        # This function takes the square meter around a point on the map (in meters)
        # and determines how many free spaces there are bu unknown spaces and away from walls.
        def poiCheck(x,y):
            size = 1/self.mapActual.info.resolution
            x_origin =  self.mapActual.info.origin.x
            y_origin =  self.mapActual.info.origin.y
            width = self.mapActual.info.width
            height = self.mapActual.info.height

            xmin = round((x-0.5-x_origin)*size)
            xmax = round((x+0.5-x_origin)*size)
            ymin = round((y-0.5-y_origin)*size)
            ymax = round((y+0.5-y_origin)*size)

            # make two arrays, one of free space and one of occupied space.
            free = []
            free = [0 for i in range(0,size*size)]
            wall = []
            wall = [0 for i in range(0,size*size)]

            for m in range(max(xmin,0),min(xmax,width-1)):
                for n in range(max(ymin,0),min(ymax,width-1)):
                    data = self.mapActual.data[m + n * width]
                    if data < 10:
                        free[m - xmax + (n - ymax) * size] = 1
                    if data > 90:
                        wall[m - xmax + (n - ymax) * size] = 1
            
            bigfree = dialate([free,size,size],[[1 for i in range(0,3*3)],3,3])
            bigwall = dialate([wall,size,size],[[1 for i in range(0,7*7)],7,7])
            result = []
            result = [0 for i in range(size*size)]
            for m in range(0,size):
                for n in range(0,size):
                    if bigfree[0][m+n*size] and not(bigwall[0][m+n*size] or free[m+n*size]):
                        result[m+n*size] = 1
            return sum(result)

        # This function implements 2d convolution like in matlab. the map needs to be a list containing [data, width, height]. the submap function should return those values in that order
        def conv2(map,kernel):
            mapd = map[0]
            mapw = map[1]
            maph = map[2]
            kerd = kernel[0]
            kerw = kernel[1]
            kerh = kernel[2]
            outh = maph+kerh-1
            outw = mapw+kerw-1
            outd = []
            outd = [0 for i in range(0,outw*outh)]
            for m in range(0,mapw):
                for n in range(0,maph):
                    for i in range(0,kerw):
                        for j in range(0,kerh):
                            outd[m+i+(n+j)*outw] = outd[m+i+(n+j)*outw] + mapd[m+n*mapw] * kerd[i+j*kerw]
            return [outd,outw,outh]

        def erode(map,kernel):
            temp = conv2(map,kernel)
            startx = (kernel[1]-1)/2
            starty = (kernel[2]-1)/2
            endx = temp[1]-startx-1
            endy = temp[2]-starty-1
            temp = submap(temp,[startx,endx],[starty,endy])
            kernelsum = sum(kernel[0])
            outd = []
            outd = [0 for i in range(0,temp[1]*temp[2])]
            for m in range(0,temp[1]):
                for n in range(0,temp[2]):
                    if temp[0][m+n*temp[1]] == kernelsum:
                        outd[m+n*temp[1]] = 1
            return [outd,temp[1],temp[2]]

        def dialate(map,kernel):
            temp = conv2(map,kernel)
            startx = (kernel[1]-1)/2
            starty = (kernel[2]-1)/2
            endx = temp[1]-startx-1
            endy = temp[2]-starty-1
            temp = submap(temp,[startx,endx],[starty,endy])
            outd = []
            outd = [0 for i in range(0,temp[1]*temp[2])]
            for m in range(0,temp[1]):
                for n in range(0,temp[2]):
                    if temp[0][m+n*temp[1]] > 0:
                        outd[m+n*temp[1]] = 1
            return [outd,temp[1],temp[2]]


        # This function takes the whole map data and splits it into a smaller section containing only range x = [x1,x2] and y = [y1,y2] in gridsquares.
        # it returns [data,width,height] for use in conv2.
        def submap(map,x,y):
            mapd = map[0]
            mapw = map[1]
            width = max(x)-min(x)+1
            height = max(y)-min(y)+1
            dataout = []
            dataout = [0 for i in range(0,width * height)]
            for m in range(min(x),max(x)+1):
                for n in range(min(y),max(y)+1):
                    dataout((m - min(x)) + (n - max(y)) * width) = mapd[m+n*mapw]
            return [dataout,width,height]

        # This method resets the waypoints in the event of an obstacle preventing the traversal
        # to waypoint 1 or if all paths fail around waypoint 3.
        def resetWaypoints():
            print("reset")
            # TODO: modify to match waypoint generation (i didn't plan this one out yet)
            # Get position of robot as a matrix value in the map.
            #x_pos = int(getMatrixPosition()[0])
            #y_pos = int(getMatrixPosition()[1])

            x_pos = getMatrixPosition()[0]
            y_pos = getMatrixPosition()[1]

            dist = 0.3

            entropyDirections = []
            entropyDirections = [0 for i in range(0,self.direction_count)]
            foundWalls = []
            foundWalls = [False for i in range(0,self.direction_count)]

            rayLimit = 40
            offset = 30
            power = 2

            if x_pos > self.mapActual.info.origin.position.x and y_pos > self.mapActual.info.origin.position.y:
                print("xpos>xmin,ypos>ymin")
                # Calculate entropy sums.
                for k in range(1, rayLimit):
                    #print("k = " + str(k))
                    # Check each direction
                    for d in range(0,self.direction_count):
                        x = (x_pos + k*math.cos(2*math.pi/self.direction_count * d))
                        y = (y_pos + k*math.sin(2*math.pi/self.direction_count * d))
                        if x > 0 and x < self.mapActual.info.width and y > 0 and y < self.mapActual.info.height and foundWalls[d] is False:
                            #print(str(x)+" , "+str(y))
                            mapdata = map(int(round(x)),int(round(y)))
                            if mapdata > 90:
                                foundWalls[d] = True
                            entropyDirections[d] = entropyDirections[d] + pow(mapdata - offset, power)/k
                        else:
                            entropyDirections[d] = entropyDirections[d] + 9999
                
            # Find the direction of minimum entropy.
            direction = None
            maxVal = 1000
            minVal = 500
            self.entropyVector = entropyDirections
            #for value in entropyDirections:
            #    if value < maxVal and value > minVal:
            #        direction = entropyDirections.index(value)
            #        break
            
            #if direction is None:
            direction = entropyDirections.index(min(entropyDirections))

            # Establish path as a series of new waypoints.
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            for i in range(0,self.waypoint_count):
                x = x + dist*math.cos(2*math.pi/self.direction_count * direction)
                y = y + dist*math.sin(2*math.pi/self.direction_count * direction)
                self.waypoints[i].point = Point(x,y,0)
                self.waypoints[i].theta = 2*math.pi/self.direction_count * direction


        # This method will create a new waypoint once the robot is within a certain distance
        # to waypoint 1.
        def createNewWaypoint():
            print("new waypoint!")
            # Get the position of the last waypoint.            
            end = self.waypoints[self.waypoint_count-1].point

            # number of directions
            #self.direction_count = 6
            # distance from last point in meters
            dist = 0.4
            # height and width of square to test in meters
            size = 0.5
            biggerboxmultiplier = 3

            entropyDirections = []
            entropyDirections = [0 for i in range(0,self.direction_count)]
            for i in range(0,self.direction_count):
                print(i)
                entropyDirections[i] = grabEntropySquare(end.x + math.cos(2*math.pi/self.direction_count * i)*dist - size/2, end.x + math.cos(2*math.pi/self.direction_count * i)*dist + size/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist - size/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist + size/2)
                + grabEntropySquare(end.x + math.cos(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier - size*biggerboxmultiplier/2, end.x + math.cos(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier + size*biggerboxmultiplier/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier - size*biggerboxmultiplier/2, end.y + math.sin(2*math.pi/self.direction_count * i)*dist*biggerboxmultiplier + size*biggerboxmultiplier/2)/9*0.1
                #TODO: add more boxes
            # Initialize a parameter to check if there is an obstacle between the 3rd waypoint and the generated waypoint.
            # It is assumed to be true that there is an obstacle between the points.
            isObstacle = 1

            while isObstacle == 1:
                inc = 0
                #test = 0
                for x in entropyDirections:
                    rospy.loginfo(str(inc)+": " +str(x))
                    inc = inc + 1

                # Find the direction to place a new waypoint. The square with the highest entropy is chosen.
                # The entropy tells the robot where the "highest reward" is.
                # TODO: implement tiebreaker?
                direction = entropyDirections.index(max(entropyDirections))   

                rospy.loginfo("try:"+str(direction))           

                # Get the number of squares to increase in either direction depending on the calculated direction.
                dx = dist*math.cos(2*math.pi/self.direction_count * direction)
                dy = dist*math.sin(2*math.pi/self.direction_count * direction)

                # Get the last waypoint for ease of calculations.
                end = self.waypoints[self.waypoint_count-1].point

                # Check for duplicate points.
                # TODO: think more like nearby points than exact matches?
                duplicates = 0
                for i in range(0, self.waypoint_count):
                    #rospy.loginfo("Point " + str(i) +"\nNew point: ("+str(end.x + dx)+","+str(end.y + dy)+")\nOld point: ("+str(self.waypoints.get(i).x)+","+str(self.waypoints.get(i).y)+")") 
                    if end.x + dx == self.waypoints[i].point.x and end.y + dy == self.waypoints[i].point.y:
                        rospy.loginfo("Match")     
                        duplicates = duplicates + 1
                    else:
                        rospy.loginfo("No Match")

                # Connect the 3rd point and the theoretical last point.
                yMin = min([end.y, end.y + dy])
                yMax = max([end.y, end.y + dy])
                xMin = min([end.x, end.x + dx])
                xMax = max([end.x, end.x + dx])

                # Check for bounds errors or if any duplicates exist. If so, restart the sweek by setting the entropy
                # sum in that direction to 0. This will prevent that direction from being searched again since the 
                # algorithm checks for the maximum entropy value.
                if yMin < self.mapActual.info.origin.position.y + 1 or yMax >= self.mapActual.info.origin.position.y + self.mapActual.info.height*self.mapActual.info.resolution -1 or xMin < self.mapActual.info.origin.position.x + 1 or xMax >= self.mapActual.info.origin.position.x + self.mapActual.info.width*self.mapActual.info.resolution -1 or duplicates > 0:
                    if duplicates > 0:
                        rospy.loginfo("duplicates > 0 (" + str(duplicates) + ")")
                    entropyDirections[direction] = 0
                else:
                    maximum = 0
                    for i in range(int(round((xMin - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution) - 1), int(round((xMax - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution) + 2)):
                        for j in range(int(round((yMin - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution) - 1), int(round((yMax - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution) + 2)):
                            regionX = (xMax + xMin)/2
                            regionY = (yMax + yMin)/2
                            regionPos = PointStamped()
                            regionPos.header.stamp = rospy.Time()
                            regionPos.header.frame_id = "map"
                            regionPos.point = Point(regionX, regionY, 1)
                            self.region_publisher.publish(regionPos)
                            if map(i, j) > maximum:
                                maximum = map(i, j)

                    if maximum > 70:
                        entropyDirections[direction] = 0
                        rospy.loginfo("maximum > 0.7 (" + str(maximum) + ")")
                    else:
                        for i in range(0,self.waypoint_count-1):
                            self.waypoints[i].point = self.waypoints[i+1].point
                            self.waypoints[i].theta = self.waypoints[i+1].theta
                        self.waypoints[self.waypoint_count-1].point = Point(end.x + dx, end.y + dy, 0)
                        self.waypoints[self.waypoint_count-1].theta = 2*math.pi/self.direction_count * direction
                        isObstacle = 0
                        self.fails = 0

                # Track number of fails.
                self.fails = self.fails + 1
                rospy.loginfo("Fails: " + str(self.fails))

                if self.fails > 7:
                    print("fail reset")
                    resetWaypoints()
                    self.fails = 0
                    isObstacle = 0



        # Method to calculate an entire region of entropy. Reduces the necessary lines of code to write.
        def grabEntropySquare(range_x_1, range_x_2, range_y_1, range_y_2):
            # Initialize the result of the scan.
            result = 0

            # convert all inputs to grid domain
            range_x_1 = int(round((range_x_1 - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution))
            range_x_2 = int(round((range_x_2 - self.mapActual.info.origin.position.x)/self.mapActual.info.resolution))
            range_y_1 = int(round((range_y_1 - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution))
            range_y_2 = int(round((range_y_2 - self.mapActual.info.origin.position.y)/self.mapActual.info.resolution))

            # On the bounds of the given entropy region, calculate the total entropy.
            # added some extra brain to make sure we go biggest to smallest and to make the last value inclusive.
            for i in range(min(range_x_1, range_x_2),max(range_x_1, range_x_2) + 1):
                for j in range(min(range_y_1, range_y_2),max(range_y_1, range_y_2) + 1):
                    result = result + entropy(i, j)

            # Return.
            return result
                

        # This method calculates and returns the entropy data at a given matrix coordinate.
        def map(x, y):
            # If the value at a given index is -1, return 100. This is to keep the robot from travrsing
            # to regions that have not been explored.
            # rospy.loginfo(len(self.mapActual.data))
            #print("checking ("+str(x)+","+str(y)+")")
            if self.mapActual.data[x + (self.mapActual.info.width * y)] < 0:
                return 50
            # Return.
            else:
                return self.mapActual.data[x + (self.mapActual.info.width * y)]

        # Method to calculate the entropy at a given map index.
        def entropy(x, y):
            #print("entropy("+str(x)+","+str(y)+")")
            # First grab probability. Divide by 102 such that 100 becomes approximately 0.99.
            p = self.mapActual.data[x + (self.mapActual.info.width * y)]
            
            # If the value of the probability at the given index is negative, replace it with 0.5.
            # Note: this does not replace the value of the probability value in the OccupancyMap.
            if p < 0:
                p = 0.5
            
            p = p / 102.0

            # Quick calculation to ensure that the probability is between 0.01 and 0.99.
            p = (p * 0.98) + 0.01

            # Return.
            return ((-p * math.log(p, 2)) - ((1 - p) * math.log(1 - p, 2)))


        # This method checks for obstacles between the robot and waypoint 1. Taken from camp_goto_node.
        def obstacleCheck():
            # Instantiate a closest object integer.
            closestFrontObject = 0

            # Obtain lidar distance data.
            ranges = self.lidar.ranges
            if len(ranges) == 360:
                if ranges[0] != 0:
                    closestFrontObject = ranges[0]
                else:
                    closestFrontObject = 500

                # Increment the second value in this loop to sweep over a larger angle.
                for i in range(1, 17):
                    if ranges[i] < closestFrontObject and ranges[i] != 0:
                        closestFrontObject = self.lidar.ranges[i]
                    if ranges[360 - i] < closestFrontObject and ranges[360 - i] != 0:
                        closestFrontObject = ranges[360 - i]
                
                # Threshold distance. *This double is in meters.
                if closestFrontObject < 0.3:
                    print("object "+str(closestFrontObject)+" meters away!")
                    return True
                else:
                    return False
            else:
                print("no lidar data!")
                return True
        
        # Main functionality of the pathfinding code. 
        self.newPoint = False  # For debug. Prints if the algorithm is currently calculating a new point.
        self.reset = False
        # First, check for obstacles. If an obstacle is found between the robot and it's target, reset the path.
        # If an object is not found between the robot and it's target, and the path is valid, calculate a new waypoint.
        dx = self.odom.pose.pose.position.x - self.waypoints[0].point.x
        dy = self.odom.pose.pose.position.y - self.waypoints[0].point.y
        diff = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        if obstacleCheck():
            print("obstacle reset")
            resetWaypoints()
            self.reset = True
        elif diff > 5:
            print("diff > 5 reset")
            print(diff)
            resetWaypoints()
        else:
            if diff < self.satisDist:
                # when close enough to waypoint 0
                # adjust the next waypoint
                adjustment = self.waypoints[1].auto_adjust(self.mapActual)
                # do the adjustment on all the other waypoints after
                for i in range(2,self.waypoint_count):
                    self.waypoints[i].quick_adjust(adjustment)
                # create a new waypoint after adjustment
                if len(self.waypoints) == self.waypoint_count:
                    # if we are in explore mode, make a new waypoint at the end
                    createNewWaypoint()
                else:
                    # otherwise, just delete the first one and push the rest down
                    self.waypoints = [self.waypoints[i] for i in range(1,len(self.waypoints))]

                self.newPoint = True
            if self.newPoint == True and diff > 0.3:
                self.newPoint = False 
            if self.backupOld is True and self.backupNew is False:
                print("backup reset")
                resetWaypoints()
        
        publishWaypoints()
        # ROS info for debugging. Prints the waypoints and boolean information regarding the algorithm's status.
        #rospy.loginfo("\nRobot:" +  
        #              "\nReset        : " + str(self.reset) + 
        #              "\nCalculate    : " + str(newPoint) + 
        #              "\nFails        : " + str(self.fails) + 
        #              "\nEntropy Array: " + str(self.entropyVector))

        # Publish the waypoints to rqt for other scripts to use.


# Trigger functionality. Run this script until the keyboardInterrupt is triggered.
if __name__ == '__main__':
    path = Pathfinding_Node()
    while not rospy.is_shutdown():
        path.main()

        # Run at 10 Hz.
        rospy.sleep(0.1)

