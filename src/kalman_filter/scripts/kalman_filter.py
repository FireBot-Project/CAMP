#!/usr/bin/env python
#----------------------------------------------------------------------------------------
# Original Author : Sean Carda
# Project         : Coordinated Autonomous Movement & Pathfinding (CAMP)
# Original Date   : 2/2/2021
# Description     : This python script filters positional data from the
#                   Decawave TOF sensors.  
#
#
#
# Change Log:
#
# 2/2/2021 - Sean Carda: Created script and initial methods.
#
#
#
#----------------------------------------------------------------------------------------


# Import functions for matrix operaions.
import roslib
import csv
roslib.load_manifest('rospy')
import numpy;
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from localizer_dwm1001.msg import Tag
from std_msgs.msg import String, Float64

class KalmanFilter:

    def __init__(self):

        with open('positions.csv', 'w') as file:
            self.writer = csv.writer(file)
            self.writer.writerow(['input x', 'input y', 'filtered x', 'filtered y'])

        # Initialize node.
        rospy.init_node('kalman_filter', anonymous = False)

        self.rate = rospy.Rate(1)

        # Publish x, y, and heading.
        self.info_publisher = rospy.Publisher('filtered', Twist, queue_size = 10)

        # Subscribe to TOF Tag position, velocity, and heading.
        rospy.Subscriber('/dwm1001/tag1', Tag, self.tag_update)
        rospy.Subscriber('/odom', Odometry, self.velocity_update)
        rospy.Subscriber('/imu', Imu, self.heading_update)

        self.position = Tag()
        self.heading = float()
        self.u = float()


        # State dynamics matrix. This will be a 3 x 3 matrix.
        self.A = numpy.array([[1.0, 1.0, 1.0], 
                              [1.0, 1.0, 1.0], 
                              [1.0, 1.0, 1.0]])

        # Input array. This will be a 3 x 1 array.
        self.B = numpy.array([[1.0], 
                              [1.0], 
                              [1.0]])

        self.C = numpy.array([[1.0, 0, 0],
                             [0, 1.0, 0],
                             [0, 0, 1.0]])

        # Process noise covariance matrix Q. This will be a 3 x 3 matrix.
        self.Q = numpy.array([[5, 0, 0], 
                              [0, 10, 0], 
                              [0, 0, 80]])

        # Measurement noise covariance matrix R. This will be a 3 x 3 matrix.
        self.R = numpy.array([[0.2, 0, 0], 
                              [0, 0.5, 0], 
                              [0, 0, 0.7]])

        # Uncertainty matrix. This will be a 3 x 3 matrix.
        self.P = numpy.array([[0, 0, 0], 
                              [0, 0, 0], 
                              [0, 0, 0]])

        # Finally, we will start with an initial prediction of state X = [0 0 0].
        # We will also start with a corrected state equal to the predicted state.
        self.X_hat = numpy.array([[20.0], 
                                  [20.0],
                                  [20.0]])
        self.X_corrected = self.X_hat

    # Update Tag data.
    def tag_update(self, data):
        self.position.x = data.x
        self.position.y = data.y
    
    # Update velocity data.
    def velocity_update(self, data):
        self.u = data.twist.twist.linear.x


    # Update heading data.
    def heading_update(self, data):
        self.heading = data.orientation.z


    def main(self):
        
        # Prediction Step:
        self.X_hat = numpy.dot(self.A, self.X_corrected) + (self.B * self.u)
        self.P = numpy.dot(self.A, numpy.dot(self.P, numpy.transpose(self.A))) + self.Q

        # Measurement Y:
        Y = numpy.array([[self.position.x], 
                         [self.position.y], 
                         [self.heading]])

        # Correction Step:
        K1 = numpy.dot(self.P, self.C)
        K2 = numpy.linalg.inv(numpy.dot(numpy.dot(self.C, self.P), numpy.transpose(self.C)) + self.R)
        K = numpy.dot(K1, K2)
        self.X_corrected = self.X_hat + numpy.dot(K, Y - numpy.dot(self.C, self.X_hat))
        self.P = self.P - numpy.dot(numpy.dot(K, self.C), self.P)

        result = Twist()
        result.linear.x = self.X_corrected[0]
        result.linear.y = self.X_corrected[1]
        result.angular.x = self.X_corrected[2]

        print("\nvelocity: " + str(self.u) +
              "\nkalman gain: " + str(K) + 
              "\ncovariance: " + str(self.P) +
              "\nx_i: " + str(self.position.x) + "  y_i: " + str(self.position.y) + " heading_i: " + str(self.heading) +
              "\nx_o: " + str(result.linear.x) + "  y_o: " + str(result.linear.y) + " heading_o: " + str(result.angular.x) +
              "\n")

        with open('positions.csv', 'w') as file:
            self.writer = csv.writer(file)
            self.writer.writerow([Y[0], Y[1], result.linear.x.data, result.linear.y.data])

        self.info_publisher.publish(result)

if __name__ == '__main__':
    filter = KalmanFilter()
    while not rospy.is_shutdown():
        filter.main()
        rospy.sleep(1)
    


    









