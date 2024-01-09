#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from math import pi, tau, dist, fabs, cos, atan2, degrees
import numpy as np
import cv2
import sys
import copy

from gaze_api import GazeAPI

import time

# parameters
RATE = 50                      # ROS update rate
THRESH_EYES = 0.4               # Threshold distance for gaze to eyes
THRESH_TABLET = 0.8             # Threshold distance for gaze to tablet
TABLET_POS = [0, 0.45, 0.37]    # X, Y and Z offset of the tablet w.r.t camera
SB_NO = 2                       # Serial number of the Survivor Buddy used

class SurvivorBuddy:
    def __init__(self) -> None:
        """
        Inititialize the SurvivorBuddy object
        Args:
            None
        Return:
            None
        """
        self.gaze_api = GazeAPI()

        self.sub_camera = rospy.Subscriber(
            "/camera/image/compressed", CompressedImage, 
            callback=self.callback_image, queue_size=1
        )

        self.pub_sb = rospy.Publisher("/sb_" + str(SB_NO) + "_cmd_state", 
            TwistStamped, queue_size=10)
        self.pub_gaze = rospy.Publisher("/gaze", String,
            queue_size=10)
        self.pub_head = rospy.Publisher("/head_loc", String, queue_size=10)

        self.rate = rospy.Rate(RATE)

        self.thresh_eyes = THRESH_EYES
        self.thresh_tablet = THRESH_TABLET
        self.tablet_pos = TABLET_POS
        self.image_data = None
        self.twist = TwistStamped()
        self.img_h = None
        self.img_w = None

        self.joint_pos = np.zeros(4)
        self.twist.twist.linear.x = -self.joint_pos[0]
        self.twist.twist.linear.y = -self.joint_pos[1]
        self.twist.twist.linear.z = self.joint_pos[2]
        self.twist.twist.angular.x = -self.joint_pos[3]
        rospy.logdebug("Sending joint positions: " + str(self.joint_pos))
        self.pub_sb.publish(self.twist)
        self.rate.sleep()


    def publish_goal(self, joint_goal, wait=None):
        """
        Publish joint goal to SurvivorBuddy
        Args:
            joint_goal : np.array
                joint angle goal for the robot (radians)
            wait : float
                time to wait after publishing joint goal
        Return:
            None
        """    
        joint_goal_deg = np.clip(np.rad2deg(joint_goal), a_min=-45, a_max=45)
        self.twist.twist.linear.x = -joint_goal_deg[0]
        self.twist.twist.linear.y = -joint_goal_deg[1]
        self.twist.twist.linear.z = joint_goal_deg[2]
        self.twist.twist.angular.x = -joint_goal_deg[3]
        rospy.logdebug("Sending joint positions: " + str(joint_goal_deg))
        self.pub_sb.publish(self.twist)
        self.joint_pos = joint_goal
        if wait:
            rospy.sleep(wait)
        else:
            self.rate.sleep()


    def callback_image(self, data):
        """
        Callback for image.
        Args:
            data : CompressedImage
                compressed image data from camera
        Return:
            None
        """
        camera_data = data.data
        compressed_arr = np.fromstring(camera_data, np.uint8)
        self.image_data = cv2.imdecode(compressed_arr, cv2.IMREAD_COLOR)
        if self.img_h is None:
            self.img_h = self.image_data.shape[0]
            self.img_w = self.image_data.shape[1]


    def behavior_attention(self):
        """
        Behavior for robot to seek human attention.
        Args:
            None
        Return:
            None
        """

        # sweep to draw attention
        joint_goal = np.copy(self.joint_pos)
        for i in range(0, int(100*pi/4), 5):
            joint_goal[2] = i / 100.0
            self.publish_goal(joint_goal)
        for i in range(int(100*pi/4), int(-100*pi/4), -5):
            joint_goal[2] = i / 100.0
            self.publish_goal(joint_goal)
        for i in range(int(-100*pi/4), 0, 1):
            joint_goal[2] = i / 100.0
            self.publish_goal(joint_goal)
        self.face_v = 0.


    def behavior_confused(self):
        """
        Behavior when the robot doesn't detect a face.
        Args:
            None
        Return:
            None
        """

        # sweep to look for faces
        joint_goal = np.copy(self.joint_pos)
        for i in range(int(joint_goal[1]*100), int(100*pi/4), 1):
            joint_goal[1] = i / 100.0
            self.publish_goal(joint_goal)
            # alert_face = self.detect_face()
            # alert_audio = self.detect_noise()
            # if alert_face or alert_audio or rospy.is_shutdown():
            #     self.face_v = 0.
            #     return

        for i in range(int(joint_goal[1]*100), int(-100*pi/4), -1):
            joint_goal[1] = i / 100.0
            self.publish_goal(joint_goal)
            # alert_face = self.detect_face()
            # alert_audio = self.detect_noise()
            # if alert_face or alert_audio or rospy.is_shutdown():
            #     self.face_v = 0.
            #     return

        for i in range(int(joint_goal[1]*100), 0, 1):
            joint_goal[1] = i / 100.0
            self.publish_goal(joint_goal)
            # alert_face = self.detect_face()
            # alert_audio = self.detect_noise()
            # if alert_face or alert_audio or rospy.is_shutdown():
            #     self.face_v = 0.
            #     return

        self.face_v = 0.
        return


    def behavior_mutual_gaze(self, head_pos):
        """
        Behavior for the robot to follow a face and establish mutual gaze.
        Args:
            head_pos: np.array
                3D position of the head detected in the image
        Return:
            None
        """
        joint_goal = np.copy(self.joint_pos)
        e_w = head_pos[0]
        e_h = head_pos[1]
        joint_goal[1] = e_w * 0.7
        joint_goal[3] = e_h * -1 * 0.8

        self.publish_goal(joint_goal)


    def percept_gaze(self, head_pos, gaze_vec):
        """
        Determine where the gaze vector is directed at the tablet.
        Args:
            head_pos: np.array
                3D position of the head detected in the image
            gaze_vec: np.array
                eye gaze vector
        Return:
            gaze_category: int
                category of gaze detected. 1 - eye, 2 - tablet, 0 - none
        """

        dir_eye = head_pos / np.linalg.norm(head_pos)
        proj_eye = (np.dot(gaze_vec, dir_eye) * dir_eye
                    / np.dot(dir_eye, dir_eye))
        dist_eye = np.linalg.norm(gaze_vec - proj_eye)

        dir_tablet = self.tablet_pos - head_pos
        dir_tablet = dir_tablet / np.linalg.norm(dir_tablet)
        proj_tablet = ((np.dot(gaze_vec, dir_tablet) *  dir_tablet 
                        / np.dot(dir_tablet, dir_tablet))
                        + self.tablet_pos)
        dist_tablet = np.linalg.norm(gaze_vec - proj_tablet)

        if dist_eye <= self.thresh_eyes:
            gaze_category = 1
        elif dist_tablet <= self.thresh_tablet:
            gaze_category = 2
        else:
            gaze_category = 0
        return gaze_category


    def percept_head(self, bbox):
        """
        Determine which region of the image the head lies in.
        Args:
            bbox: np.array
                bounding box of the head detected in the image
        Return:
            None
        """
        center_x = (bbox[0][0] + bbox[1][0])//2
        center_y = (bbox[0][1] + bbox[1][1])//2
        width_one_third = self.img_w // 3
        height_one_third = self.img_h // 3

        if center_x <= width_one_third:
            if center_y <= height_one_third:
                location_grid = 0
            elif center_y <= (height_one_third*2):
                location_grid = 1
            else:
                location_grid = 2
        elif center_x <= (width_one_third * 2):
            if center_y <= height_one_third:
                location_grid = 3
            elif center_y <= (height_one_third*2):
                location_grid = 4
            else:
                location_grid = 5
        else:
            if center_y <= height_one_third:
                location_grid = 6
            elif center_y <= (height_one_third*2):
                location_grid = 7
            else:
                location_grid = 8
        self.pub_head.publish(str(location_grid))


    def gaze(self):
        """
        The perceptual schema for gaze. Also implements mutual gaze.
        Args:
            None
        Return:
            None
        """
        while self.img_h is None and not rospy.is_shutdown():
            rospy.logwarn("no image received.")
            self.rate.sleep()

        while not rospy.is_shutdown():
            gaze_faces = self.gaze_api.run(self.image_data)
            if len(gaze_faces) == 0:
                rospy.loginfo("no gaze detected.")
                self.pub_gaze.publish("no gaze")
            if len(gaze_faces) >= 1:
                face = gaze_faces[0] # first face used
                gaze_category = self.percept_gaze(face["head_pos"], face["gaze_vector"])
                if gaze_category == 1:
                    rospy.loginfo("eye contact.")
                    self.pub_gaze.publish("eye contact")
                elif gaze_category == 2:
                    rospy.loginfo("tablet contact.")
                    self.pub_gaze.publish("tablet contact")
                else:
                    rospy.loginfo("no contact.")
                    self.pub_gaze.publish("no contact")

                self.percept_head(face["bbox"])
                self.behavior_mutual_gaze(face["head_pos"])
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("percept_node", anonymous=False)
    rospy.loginfo("Perception node started.")
    schema_sb = SurvivorBuddy()
    schema_sb.gaze()