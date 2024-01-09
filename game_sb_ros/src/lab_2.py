#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
#from moveit_msgs.msg import DisplayTrajectory

from geometry_msgs.msg import TwistStamped

# Python 2/3 compatibility imports
import sys
import copy
import rospy
import geometry_msgs.msg
#import moveit_commander

import cv2
import numpy as np
import time

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String


class GenericBehavior(object):
    """
    Generic behavior class.
    """
    def __init__(self):
        self.pub = rospy.Publisher("/sb_cmd_state", TwistStamped, queue_size = 20)
        
        self.prev = np.array([])
        self.prev_joint = []
        self.prev_center = 0
        self.num = 0
        self.frames = 0
        self.audio = 0
        
        self.audio_sub = rospy.Subscriber("/audio", Float32MultiArray, callback=self.callback_1, queue_size=20)
        self.camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=self.callback_2, queue_size=20)
        rospy.loginfo("Node started.")


    def callback_1(self, data):
    	alert = False
    	if max(data.data) > 0.5:
    		alert = True
    	if min(data.data) < (-0.5):
    		alert = True
    	if self.num%10 == 0:
    		self.audio = 0
    	
    	if alert and self.audio == 0:
    		
    		print("Noise Detected.")
    		
    		joint_goal = [0, 0, 0, 0]
    		joint_goal[0] = -20#-0.5
    		joint_goal[1] = -20 #-0.5
    		joint_goal[2] = 20 #0.5
    		joint_goal[3] = 0
    		

    		twist = TwistStamped()
    		
    		twist.twist.linear.x = joint_goal[0]
    		twist.twist.linear.y = joint_goal[1]
    		twist.twist.linear.z = joint_goal[2]
    		twist.twist.angular.x = joint_goal[3]
    		
    		
    		self.pub.publish(twist)
    		
    		
    		time.sleep(2)
    		
    		joint_goal[0] = 0
    		joint_goal[1] = 0
    		joint_goal[2] = 0
    		joint_goal[3] = 0
    		
    		twist.twist.linear.x = joint_goal[0]
    		twist.twist.linear.y = joint_goal[1]
    		twist.twist.linear.z = joint_goal[2]
    		twist.twist.angular.x = joint_goal[3]
    		
    		self.prev_joint = joint_goal
    		
    		print("Sending joint positions: ", joint_goal)
    		
    		self.pub.publish(twist)
    		
    		time.sleep(2)
    		
    		self.audio = 1
    	


    def callback_2(self, data):
    	twist = TwistStamped()
    	joint_goal = [0, 0, 0, 0]
    	if self.num == 0 or self.num %5 == 0:
    		arr_np = np.fromstring(data.data, np.uint8)
    		image_np = cv2.imdecode(arr_np, cv2.IMREAD_COLOR)
    		image_rotate = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
    		face = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    		gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
    		#rotated = cv2.rotate(gray, cv2.ROTATE_90_CLOCKWISE)
    		faces = face.detectMultiScale(gray, 1.3, 5)
    		
    		#image_np = rotated
    		if self.num == 0:
    			joint_goal = [0, 0, 0, 0]
    			self.prev_joint = joint_goal
    		else:
    			joint_goal = self.prev_joint
    		#joint_goal = [0, 0, 0, 0]
    		if len(faces) == 0:
    			print("No face detected.")
    			self.frames = self.frames + 1
    			joint_goal[2] = 20 #0.5
    			
    			if self.frames == 10:
    				joint_goal[0] = 0
    				joint_goal[1] = 0
    				joint_goal[2] = 0
    				joint_goal[3] = 0
    				self.frames = 0
    				#time.sleep(2)
    		
    			twist.twist.linear.x = joint_goal[0]
    			twist.twist.linear.y = joint_goal[1]
    			twist.twist.linear.z = joint_goal[2]
    			twist.twist.angular.x = joint_goal[3]
    			
    		for (x,y,w,h) in faces:
    			
    			
    			self.frames = 0
    			cv2.rectangle(image_np, (x,y), (x+w, y+h), (255, 255, 0), 2)
    	
    			center_x = x + w/2
    			center_y = y + h/2
    	
    			height, width, channels = image_np.shape
    		
    			area = height*width
    			
    			quarter = width/2
    		
    			left = 0
    			right = 0
    			
    				
    			if center_x == width/2 or (center_x < (width/2 + 40) and center_x > (width/2-40)):
    				joint_goal[1] = 0#self.prev_joint[1]
    				print("center")
    				
    				
    			elif center_x < quarter:
    			#else:	
    				dist = width/2 - center_x
    				
    				dist = (width/2-center_x)/(width/2)
    				#print(dist)
    				
    				#move = dist*width/2
    				print("left", 90*dist)
    				
    				angle = 90*dist
    				if angle < 90 and angle > -90:
    					joint_goal[1] = -angle
    				
    				#if (self.prev_joint[1] - 10) <= 90:
    				#	joint_goal[1] = -85
    				#else:
    				#	joint_goal[1] = self.prev_joint[1] - 10
    			
    				self.prev_joint = joint_goal
    				
    				

    				#twist.twist.linear.x = 0#joint_goal[0]
    				twist.twist.linear.y = joint_goal[1]
    				#twist.twist.linear.z = 0#joint_goal[2]
    				#twist.twist.angular.x = 0#joint_goal[3]
    				
    				#self.pub.publish(twist)
    				#time.sleep(2)
    			else:	
    				dist = width/2 - center_x
    				dist = (width/2-center_x)/(width/2)
    				
    				move = dist*width/2
    				print("right", 90*dist)
    				
    				angle = 90*dist
    				if angle < 90 and angle > -90:
    					joint_goal[1] = -angle 
    				
    				#if (self.prev_joint[1] + 10) >= 90:
    				#	joint_goal[1] = 85
    				#else:
    				#	joint_goal[1] = self.prev_joint[1] + 10
    					
    				self.prev_joint = joint_goal
    					
    				#twist = TwistStamped()

    				#twist.twist.linear.x = 0#joint_goal[0]
    				twist.twist.linear.y = joint_goal[1]
    				#twist.twist.linear.z = 0#joint_goal[2]
    				#twist.twist.angular.x = 0#joint_goal[3]
    				
    				#self.pub.publish(twist)
    				#time.sleep(2)
    		
    			
    			
    			area_face = h*w
    			area_prev = 0
    			if len(self.prev) > 0:
    				area_prev = self.prev[2] * self.prev[3]
    		
    			percentage = area_face/area
    			if percentage > 0.45:
    				print("CLOSE")
    				joint_goal[0] = 20
    				joint_goal[3] = 20
    				
    				#twist = TwistStamped()
    				twist.twist.linear.x = joint_goal[0]
    				#twist.twist.linear.y = joint_goal[1]
    				#twist.twist.linear.z = 0#joint_goal[2]
    				twist.twist.angular.x = joint_goal[3]
    			else:
    				if percentage > 0.30:
    					joint_goal[0] = -20
    					joint_goal[3] = -20
    					twist.twist.linear.x = joint_goal[0]
    					twist.twist.angular.x = joint_goal[3]
    				else:
    					twist.twist.linear.x = 0
    					twist.twist.angular.x = 0
    		
    			#if area_prev > 0 and False:	
    			#	movement = area_face/area_prev
    				
    				#if movement < 1.1 and movement > 0.9:
    				#	print("No movement")
    				#if movement > 1.1 and movement < 1.3:
    			#	if percentage> 0.35:
    			#		joint_goal[0] = 20
    			#		joint_goal[3] = 10#0.5
    					#print("Some movement")
    			#		if percentage > 0.45:
    			#			joint_goal[0] = -20#-0.5
    			#			joint_goal[3] = -10#0.5
    			#			print("TOO CLOSE!")
    			#	else:
    			#		joint_goal[0] = 0
    			#		joint_goal[3] = 0
    			#	if movement > 1.3:
    			#		print("Fast movement")
    			#		joint_goal[3] = -20#-0.5
    			#		joint_goal[2] = 20#0.5
    				#if movement < 0.9:
    					#joint_goal[3] = 0
    					#print("Move back")
    					
    			#	prev_center_x = self.prev[0] + self.prev[2]/2
    			#	prev_right = 0
    			#	prev_left = 0
    			self.prev = [x,y,w,h]
    			twist.twist.linear.z = 0
    		
    	if self.num %5 == 0 or self.num == 0:
    		cv2.imshow("Image", image_np)
    		cv2.waitKey(2)
    		#time.sleep(2)
    		
    		#twist = TwistStamped()

    		#twist.twist.linear.x = joint_goal[0]
    		#twist.twist.linear.y = joint_goal[1]
    		#twist.twist.linear.z = joint_goal[2]
    		#twist.twist.angular.x = joint_goal[3]
    		
    		self.prev_joint = joint_goal
    		#print("Sending joint positions: ", joint_goal)
    	
    	
    		self.pub.publish(twist)
    		
    		#time.sleep(2)
    	self.num = self.num +1
  
if __name__ == '__main__':
    rospy.init_node("lab_1_node")
    
    
    ##############################
    # YOUR CODE HERE             #
    # Call the behavior(s)       #
    ##############################
    
    behave = GenericBehavior()
    
    
    rospy.spin()
