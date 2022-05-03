#!/usr/bin/env python 
import numpy
import rospy
import roslib
import math

from cv2 import sqrt
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32, String, Float32MultiArray
#from nav_msgs.msg import Odometry
from localizer_dwm1001.msg import Tag

class PathPlan:    
    def getState(self, data):
        #print("\ndatax is: ")
		#print(data.x)
		#print("\ndatay is: ")
		#print(data.y)
        self.estate = data 

    def getPosition(self, data):
        #print("\ndatax is: ")
		#print(data.x)
		#print("\ndatay is: ")
		#print(data.y)
        self.pose.x = data.x
        self.pose.y = data.y      
    
    def __init__(self):
        rospy.init_node('GoToPos', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz  slows this down
        #rospy.Subscriber('odom', Odometry, self.getPosition)
        rospy.Subscriber('estimated_state', estimated_state, self.getState) # subscribe tag message
        rospy.Subscriber('filtered_pos', Tag, self.getPosition) # subscribe tag message

        self.pub = rospy.Publisher('desired_position', Vector3, queue_size=10)
        
        self.des_pos = Vector3()
        self.offset = 0
        self.slice_size = 150
        self.led_length = 4.9022
        self.index_distance = self.led_length / self.slice_size
        self.entropy = [0] * slice_size
        self.desired_index = 0

        self.pose = Tag() #filltered position 
		
        #initialize desired postion = current position
		self.des_pos.z = self.pose.z
		self.des_pos.x = self.pose.x
		self.des_pos.y = self.pose.y

    def main(self):
        #Binary entropy calculation
        #rob_index is pose.x 
        #N = 150 <= index size
        
        #entropy = -est_state.*log2(est_state)-(1-est_state).*log2(1-est_state); 
        for i in range(0, 149)
            entropy[i] = -estate[i]*math.log2(estate[i])-(1-estate[i])*math.log2(1-estate[i])

        #Robot motion based on entropy - look 30 steps away (1 meter) 
        
        rob_index = min(math.floor(pose.x / self.index_distance), 149) #prevent out of bound
        #ind1 = max(1,rob_index-30);
        ind1 = max(0,rob_index-30)
        #ind2 = min(N,rob_index+30);
        ind2 = min(slice_size-1,rob_index+30)
        entropy_left = 0
        entropy_right = 0
        #entropy_left = sum(entropy(ind1:(rob_index-1)));
        for i in range(ind1, rob_index-1)
            entropy_left = entropy_left + entropy[i]
        #entropy_right = sum(entropy((rob_index+1):ind2)); 
        for i in range(rob_index+1, ind2)
            entropy_right = entropy_right + entropy[i]
        
        #Note: Added bias to move towards center of map (this takes effect 
        #when entropy values are similar)      
        
        if rob_index > slice_size/2
            if (entropy_left+3)>entropy_right
                desired_index = max(0,rob_index-1)
            else
                desired_index = min(slice_size-1,rob_index+1) 
        else
            if entropy_left>(entropy_right+1.5) 
                desired_index = max(0,rob_index-1)
            else
                desired_index = min(slice_size-1,rob_index+1) 
        
		self.des_pos.z = 0
		self.des_pos.x = self.desired_index * self.index_distance
		self.des_pos.y = self.pose.y
        
        #send position and print it
        #rospy.loginfo(self.des_pos)
        pub.publish(self.des_pos)
        rate.sleep()        

if __name__ == "__main__":
    pathplan = PathPlan();
    while not rospy.is_shutdown():
        print('desired position: ', self.des_pos)
        pathplan.main()
	rospy.sleep(1)