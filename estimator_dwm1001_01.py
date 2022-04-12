#!/usr/bin/env python 
#from geometry_msgs import msg
import numpy
import rospy
import roslib
import math

from cv2 import sqrt
#mport tf2_py
from geometry_msgs.msg import Vector3
#from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, Float32, String, Float32MultiArray
#from nav_msgs.msg import Odometry
from localizer_dwm1001.msg import Tag
from math import pow, acos
#import tf2_geometry_msgs
#mport tf2_msgs
#import tf2_ros
#from tf import transformations

class Estimator:
    def getRedScore(self, data):
        self.redScore = data.data

    #def getPosition(self, data):
    #    self.odom_pos.x = data.pose.pose.position.x
    #    self.odom_pos.y = data.pose.pose.position.y

    def getPosition(self, data):
        #print("\ndatax is: ")
        #print(data.x)
        #print("\ndatay is: ")
        #print(data.y)
        self.pose.x = data.x
        self.pose.y = data.y

    def __init__(self):
        rospy.init_node('estimator_listener', anonymous=False)
        rospy.Subscriber('img_rec_line', Float32, self.getRedScore)
        #rospy.Subscriber('odom', Odometry, self.getPosition)
        rospy.Subscriber('/dwm1001/tag', Tag, self.getPosition) # subscribe tag message
        self.pub = rospy.Publisher('estimated_state', Float32MultiArray, queue_size=5)

        self.distanceThreshold = 0.1  # The linear travel distance that constitutes a move to another grid
        self.redThreshold = 0.01  # the threshold to determine if the grid is on fire
        self.FPR = 0.1  # false positive rate
        self.FNR = 0.1  # false negative rate
        self.num_locations = 10;
        self.estimated_state = [0.5] * self.num_locations  # list of 100 locations with initial probability of 0.5

        self.index_position = 0  # linear index position
        #self.odom_pos = Vector3()
        #self.odom_pos.z = 1
        #self.last_odom_position = {'x': 0, 'y':0}
        #self.last_position = 0  # the previous position of the robot until distanceThreshold is exceeded
        self.view_width = 0 # the number of indexes to the left and right of the position that are updated
        self.redScore = 0

        self.pose = Tag() # dwm1001/tag1
        self.last_pose = Vector3() # difference between current and last position
        self.delta_pose = Vector3() # difference between current and last position
        self.last_pose.x = self.pose.x #last position = current position
        self.last_pose.y = self.pose.y

       # def getPosition(location): #get position from DecaWave topic Distance from starting position
        #    coordinates = location.data
         #   position = ((coordinates[0]-startX)^2 + (coordinates[1] - startY)^2)**(0.5);
          #  return position; 

    def decayBelief(self):
        for idx, belief in enumerate(self.estimated_state):
            if belief > 0.5:
                self.estimated_state[idx] = 0.998 * belief  # magic number: reduce current belief by 0.2% every spin to tend to 0.5
            else:
                self.estimated_state[idx] = max(0.01, 1.01 * belief)  # magic number: increased belief by 1% every spin to tend to 0.5

    def main(self):
        # check if robot has moved to the next grid
        # get the position of the robot. if the difference between the new and old positions exceed
        #  the distance threshold, update the old position to the current position and update the
        #  linear index for the belief matrix
        #dif = math.sqrt(math.pow(self.last_odom_position['x'] - self.odom_pos.x, 2 ) + math.pow(self.last_odom_position['y'] -self.odom_pos.y, 2))

        self.delta_pose.x = self.pose.x - self.last_pose.x
        self.delta_pose.y = self.pose.y - self.last_pose.y

        dist = math.sqrt(math.pow(self.delta_pose.x, 2 ) + math.pow(self.delta_pose.y, 2))

        #angle between last and current position
        if dist > 0:
            theta1 = acos(((self.delta_pose.x)/(dist)))

        #if dif >= self.distanceThreshold:
            #self.last_odom_position['x'] = self.odom_pos.x
            #self.last_odom_position['y'] = self.odom_pos.y

        if dist >= self.distanceThreshold:
            self.last_pose.x = self.pose.x
            self.last_pose.x = self.pose.y
            self.index_position += 1

        # set dynamic bounds to the left and right of the central position
        leftBound = self.index_position - self.view_width
        if leftBound < 0:
            leftBound = 0
        rightBound = self.index_position + self.view_width
        if rightBound > (self.num_locations - 1):
            rightBound = self.num_locations - 1
        viewArea = range(leftBound, rightBound + 1)

        # get the redscore and check against the threshold. If above threshold, update belief that there is
        #  a fire at the current location in the belief matrix. Otherwise, update that there is no fire.
        if self.redScore > self.redThreshold:
            for index in viewArea:
                self.estimated_state[index] = ((1 - self.FNR) * self.estimated_state[index]) / \
                                                  ((1 - self.FNR) * self.estimated_state[index] + self.FPR * (
                                                              1 - self.estimated_state[index]))
        else:
            for index in viewArea:
                self.estimated_state[index] = (self.FNR * self.estimated_state[index]) / \
                                                  ((1 - self.FPR) * (1 - self.estimated_state[index]) + self.FPR * (
                                                  self.estimated_state[index]))

        # the belief at each index in the belief matrix tends towards 0.5 (maximum uncertainty)
        self.decayBelief()
        pubdata = Float32MultiArray()
        pubdata.data = self.estimated_state
        self.pub.publish(pubdata)
        #print(self.estimated_state).


if __name__ == "__main__":
    estimator = Estimator();
    while not rospy.is_shutdown():
        #print('est state: ', estimator.estimated_state)
        print("last position x,y: {}, {}".format(estimator.last_pose.x, estimator.last_pose.y))
        print("current position x,y: {}, {}".format(estimator.pose.x, estimator.pose.y))
        estimator.main()
        rospy.sleep(1)
