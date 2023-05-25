#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert

import rospy
import numpy as np
import math
import os
import cv2
from enum import Enum
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class DetectSign():
    def __init__(self):
        self.fnPreproc()

        self.sub_image_type = "raw" # you can choose image type "compressed", "raw"
        self.pub_image_type = "compressed" # you can choose image type "compressed", "raw"

        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.cbFindTrafficSign, queue_size = 1)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.cbFindTrafficSign, queue_size = 1)

        self.pub_traffic_sign = rospy.Publisher('/detect/traffic_sign', UInt8, queue_size=1)

        if self.pub_image_type == "compressed":
            # publishes traffic sign image in compressed type 
            self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
        elif self.pub_image_type == "raw":
            # publishes traffic sign image in raw type
            self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output', Image, queue_size = 1)

        self.cvBridge = CvBridge()

        self.TrafficSign = Enum('TrafficSign', 'divide stop parking tunnel left right no_entry construction')

        self.counter = 1

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('turtlebot3_autorace_construction_detect/nodes', 'turtlebot3_autorace_construction_detect/')
        dir_path += 'file/detect_sign/'

        self.img_construction  = cv2.imread(dir_path + 'construction.png',0)

        self.kp_construction, self.des_construction  = self.sift.detectAndCompute(self.img_construction, None)
      
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err

    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        MIN_MATCH_COUNT = 8 #9
        MIN_MATCH_COUNT_P = 15
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_construction = self.flann.knnMatch(des1,self.des_construction,k=2)
      
        image_out_num = 1

        good_construction = []
        for m,n in matches_construction:
            if m.distance < 0.7*n.distance:
                good_construction.append(m)
        if len(good_construction)>MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_construction ]).reshape(-1,1,2)
            dst_pts = np.float32([self.kp_construction[m.trainIdx].pt for m in good_construction]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matches_construction = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.construction.value

                self.pub_traffic_sign.publish(msg_sign)
                rospy.loginfo("construction")

                image_out_num = 9
        else:
            matches_construction = None
      
        if image_out_num == 1:
            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(cv_image_input, "bgr8"))

            elif image_out_num == 2:
                draw_params_construction = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches_construction, # draw only inliers
                            flags = 2)

                final_construction = cv2.drawMatches(cv_image_input,kp1,self.img_construction,self.kp_construction,good_construction,None,**draw_params_construction)

                if self.pub_image_type == "compressed":
                    # publishes traffic sign image in compressed type
                    self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final_construction, "jpg"))

                elif self.pub_image_type == "raw":
                    # publishes traffic sign image in raw type
                    self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final_construction, "bgr8"))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_sign')
    node = DetectSign()
    node.main()



















