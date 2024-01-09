#!/usr/bin/env python3
import sys
import copy
import time
import cv2
import numpy as np
from math import pi, tau, dist, fabs, cos
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image


class Attention(object):
    """
    Attention class.
    """
    def __init__(self):
        
        self.rate = rospy.Rate(50)
        self.audio_pub = rospy.Publisher("/audio", String, queue_size= 20)
        self.sub_response = rospy.Subscriber("/chatter", String, callback=self.response_callback, queue_size=1)
        self.sub_gaze = rospy.Subscriber("/gaze", String, callback=self.gaze_callback, queue_size=1)

        self.number_look = 0
        self.prev_time = None
        self.interval_time = 0.
        self.interval_thresh = 10
        self.gaze = None
        self.gaze_size = 50
        self.gaze_ind = 0
        self.gaze_hist = np.zeros(self.gaze_size)
        self.gaze_dict = {"no contact": 0, "eye contact": 1, "tablet contact": 2, "no gaze": 3}
        rospy.loginfo("Node started.")


    def response_callback(self, data):
        if data.data == "start":
            self.prev_time = time.time()
        self.interval_time = time.time() - self.prev_time
        rospy.logdebug("interval_time: ", str(self.prev_time))


    def gaze_callback(self, data):
        self.gaze_hist[self.gaze_ind] = self.gaze_dict[data.data]
        self.gaze_ind += 1
        if self.gaze_ind == self.gaze_size:
            self.gaze_ind = 0
        self.gaze = np.median(self.gaze_hist)

    def game_response(self):
        while not rospy.is_shutdown():
            if self.gaze == 3: # no gaze detected
                # voice: i don't see you
                rospy.loginfo("I don't see you")
                self.audio_pub.publish("3")
                # behavior: sweep
                #pass
            elif self.prev_time is not None:
                if (time.time() - self.prev_time) > self.interval_thresh: # intervene now
                    if self.gaze == 0: # looking somewhere else / no contact
                    # voice: it's your turn now
                        rospy.loginfo("It's your turn now")
                        self.audio_pub.publish("0")
                    elif self.gaze == 1: # eye contact
                        # voice: do you want to continue?
                        rospy.loginfo("Do you want to continue")
                        self.audio_pub.publish("1a")
                    elif self.gaze == 2: # tablet contact
                        # voice: do you need any help?
                        rospy.loginfo("Do you need any help?")
                        self.audio_pub.publish("2")
            elif self.prev_time is None:
                if self.gaze == 1: # eye contact
                    # voice: do you want to play a game?
                    rospy.loginfo("Do you want to play a game?")
                    self.audio_pub.publish("1b")
           
           
           # the original code part
            # elif (time.time() - self.prev_time) > self.interval_thresh: # intervene now
            #     if self.gaze == 0: # looking somewhere else / no contact
            #         # voice: it's your turn now
            #         rospy.loginfo("It's your turn now")
            #         #pass
            #     elif self.gaze == 1: # eye contact
            #         if self.prev_time is None:
            #             # voice: do you want to play a game?
            #             rospy.loginfo("Do you want to play a game?")
            #             #pass
            #         else:
            #             # voice: do you want to continue?
            #             rospy.loginfo("Do you want to continue")
            #             #pass
            #     elif self.gaze == 2: # tablet contact
            #         # voice: do you need any help?
            #         rospy.loginfo("Do you need any help?")
            #         #pass
            self.rate.sleep()


    # def execute_game_attention(self):
    #     #url = "https://drive.google.com/file/d/15Uq-M6FkqNgI_PX4M3134jWD8Abg73gN/view?usp=share_link"
    #     value = "1"
    #     self.audio_pub.publish(value)
    #     print("You are Distracted.")
    #     return


    # def execute_attention_seek(self,data):
    #     if data.data == "no eye contact":
    #         if self.number_look == 0 or self.number_look % 50 == 0:
    #             value = "2"
    #             self.audio_pub.publish(value)
    #             print("Turn to look at the robot.")
    #             self.number_look += 1
    #     else:
    #         self.number_look = 0
    #     return


if __name__ == '__main__':
    rospy.init_node("attention_node")
    attention = Attention()
    attention.game_response()

    #rospy.spin()
