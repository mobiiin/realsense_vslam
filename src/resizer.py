#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
import sys, time
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

VERBOSE = True

class image_resizer:

    def __init__(self):
        '''
        Initialize ros publisher, ros subscriber
        '''
        # self.image = None
        self.depth = None
        self.rate = rospy.Rate(30) # 30hz
        self.cvbridge = CvBridge()

        # topic where we publish
        # self.image_pub = rospy.Publisher("image_compressed", Image, queue_size = 30)
        self.depth_pub = rospy.Publisher("depth_compressed",  Image, queue_size = 30)

        # subscribed Topic
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)

        # print("subscribed to /camera/color/image_raw")

        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw/compressed", CompressedImage, self.callback_d)

        print("subscribed to /camera/depth/image_rect_raw/compressed")
        

    def callback(self, frame):
        self.image = self.cvbridge.imgmsg_to_cv2(frame,"bgr8")
        
    def callback_d(self, frame):
        self.depth = self.cvbridge.compressed_imgmsg_to_cv2(frame)

    def resize(self, frame):
        '''
        cropping the depth image
        '''
        return frame[0:480, 100:740]
    
    def start(self):
        # rospy.spin()

        while not rospy.is_shutdown():
            if self.depth is not None:

                # if self.image is not None:
                #     self.image_pub.publish(self.cvbridge.cv2_to_imgmsg(self.image, encoding = "bgr8"))

                self.depth = self.resize(self.depth)
                self.depth_pub.publish(self.cvbridge.cv2_to_imgmsg(self.depth, encoding="8UC1"))

                rospy.loginfo('publishing image')

            self.rate.sleep()

def main(args):
    '''Initializes and cleans up ros node'''
    rospy.init_node('image_resizer', anonymous=True)
    node = image_resizer()
    node.start()
    

if __name__ == '__main__':
    main(sys.argv)