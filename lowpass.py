#!/usr/bin/env python 
import numpy
import rospy
import roslib
import math
#import time

#from cv2 import sqrt
from geometry_msgs.msg import Vector3
#from std_msgs.msg import Bool, Float32, String, Float32MultiArray
#from nav_msgs.msg import Odometry
from localizer_dwm1001.msg import Tag

class LowPassFilter:

    #def current_milli_time():
    #    return round(time.time() * 1000)

    #Odometry coordinates are commented out
    #getting positions from decawave
    def getPosition(self, data):
        #print("\ndatax is: ")
		#print(data.x)
		#print("\ndatay is: ")
		#print(data.y)
        self.pose.x = data.x
        self.pose.y = data.y      
    
    def __init__(self):
        rospy.init_node('lowpass_listener', anonymous=False)
        #rospy.Subscriber('odom', Odometry, self.getPosition)
        rospy.Subscriber('/dwm1001/tag1', Tag, self.getPosition) # subscribe tag message
        self.pub = rospy.Publisher('filtered_pos', Vector3, queue_size=5)

        # Filter coefficients 
        self.low_pass_filter = [-0.0053,-0.0028,0.0435,0.1695,0.2951,0.2951,0.1695,0.0435,-0.0028,-0.0053]; # low pass filter to time domain

        self.prev_pose_x = [0,0,0,0,0,0,0,0,0,0]; # ten previous x coodinates
        #self.prev_deriv_x = [0,0,0,0,0,0,0,0,0,0]; # ten x derivatives
        
        self.prev_pose_y = [0,0,0,0,0,0,0,0,0,0]; # ten previous y coodinates
        #self.prev_deriv_y = [0,0,0,0,0,0,0,0,0,0]; # ten y derivatives
        
        self.pose = Tag() # dwm1001/tag1
        self.filtered_pose = Vector3() # difference between current and last position
		
        self.filtered_pose.x = 0
		self.filtered_pose.y = 0
        self.filtered_pose.z = 1
        
        #self.error = 0;
        #self.prevError = 0; 
        #self.derError = 0; 
        #self.intError = 0;
        #self.MAX_INT = 10;
        
        #self.ts0 =  int(time() * 1000);
        #self.ts1 = self.ts0; 

    def main(self):
        # Shift sensor measurement array and add new value 
        # void *memcpy(void *dest, const void * src, size_t n)
        # copy prevDistVals[1] to prevDistVals[0]
        # memcpy(prevDistVals, &prevDistVals[1], sizeof(prevDistVals) - sizeof(float)); //2)
        self.prev_pose_x = self.prev_pose_x[1:] # remove the first element or del a[0], pop is not inefficient as it returns the item
        self.prev_pose_x.append(self.pose.x)

        self.prev_pose_y = self.prev_pose_y[1:]
        self.prev_pose_y.append(self.pose.y)
        
        # Multiply by filter coefficients to calculate filtered motor position 
        
        filtered_pose_x = 0;
        filtered_pose_y = 0;
        
        for i in range(0, 9)
            filtered_pose_x += self.prev_pose_x[i] * self.low_pass_filter[i];
            filtered_pose_y += self.prev_pose_y[i] * self.low_pass_filter[i];

        self.filtered_pose.x = filtered_pose_x / len(self.low_pass_filter)
		self.filtered_pose.y = filtered_pose_y / len(self.low_pass_filter)   

        self.pub.publish(self.filtered_pose)

        # Each loop, calculate the time elapsed 
        #ts2 =  int(time() * 1000);
        #stepTime = ts2 - self.ts1; 
        #self.ts1 = ts2;

        # Calculate error, integrated error, and derivative of error 
        # Convert error to meters and timing to seconds 
  
        #self.prevError = self.error; 
  
        # Shift derivative array and add new value prevDeriv
        # memcpy(prevDeriv, &prevDeriv[1], sizeof(prevDeriv) - sizeof(float)); 
        # prevDeriv[9] = derError; 
        #self.prev_deriv_x = self.prev_deriv_x[1:] # remove the first element or del a[0], pop is not inefficient as it returns the item
        #self.prev_deriv_x.append(self.derError)

        #self.prev_deriv_y = self.prev_deriv_y[1:]
        #self.prev_deriv_y.append(self.derError)        

        # Multiply by filter coefficients to calculate filtered motor position 
        #filt_deriv_x = 0;
        #filt_deriv_y = 0;
        #for i in range(0, 9)
        #    filt_deriv_x += self.prev_deriv_x[i] * self.low_pass_filter[i];
        #   filt_deriv_y += self.prev_deriv_y[i] * self.low_pass_filter[i];
  
        # Limit max/min integrated error 
        #if(self.intError > self.MAX_INT)
        #    self.intError = self.MAX_INT;
        #if(self.intError<-self.MAX_INT)
        #   self.intError = -self.MAX_INT;     

        # the belief at each index in the belief matrix tends towards 0.5 (maximum uncertainty)

if __name__ == "__main__":
    lowpassfilter = LowPassFilter();
    while not rospy.is_shutdown():
        print('filtered: ', lowpassfilter.filtered_pose)
        lowpassfilter.main()
	rospy.sleep(1)