#!/usr/bin/env python

import rospy
import std_msgs.msg
import sys, time
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import message_filters
from cv_bridge import CvBridge
import skimage
from PIL import Image as pilimage

class image_resizer:

    def __init__(self):
        '''
        Initialize ros publisher, ros subscriber
        '''
        self.image = None
        self.depth = None
        self.info = None
        self.rate = rospy.Rate(30) # 20hz
        self.cvbridge = CvBridge()
        
        self.msg_header = std_msgs.msg.Header()
        self.depth_msg_header = std_msgs.msg.Header()

        # topic where we publish
        self.image_pub = rospy.Publisher("image_compressed", Image, queue_size = 3)
        self.depth_pub = rospy.Publisher("depth_compressed",  Image, queue_size = 3)
        # self.info_pub = rospy.Publisher('camera_info_compressed', CameraInfo, queue_size= 3)

        # subscribed Topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw/compressed",CompressedImage, self.callback)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)
        print("subscribed to /camera/color/image_raw/compressed")

        # self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw/compressed", CompressedImage, self.callback_d)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback_d)
        print("subscribed to /camera/aligned_depth_to_color/image_raw")
        
        # self.info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.callback_cf)

    def callback(self, frame):
        self.msg_header = frame.header
        self.image = self.cvbridge.compressed_imgmsg_to_cv2(frame)
        # self.image = self.cvbridge.imgmsg_to_cv2(frame,"bgr8")
        
    def callback_d(self, frame_d):
        self.depth_msg_header = frame_d.header
        # self.depth = self.cvbridge.compressed_imgmsg_to_cv2(frame_d,"passthrough")
        self.depth = self.cvbridge.imgmsg_to_cv2(frame_d, "passthrough")
        # self.depth = cv.applyColorMap(cv.convertScaleAbs(self.depth, alpha=0.03), cv.COLORMAP_JET)
        # self.depth = np.array(self.depth, dtype=np.uint16)
        # self.depth *= 256

    def callback_cf(self, massage):
        self.info = massage

    # def compress(self, frame):
    #     self.compress = 
    def resize(self, frame):
        '''
        cropping the depth image
        '''
        return frame[0:480, 100:740]
    
    def start(self):
        # rospy.spin()

        while not rospy.is_shutdown():
            
            if self.image is not None:

                image = Image()
                # image = self.image.resize(480,640, pilimage.ANTIALIAS)
                # image = self.image.thumbnail(480,640)
                image = self.cvbridge.cv2_to_imgmsg(self.image,"bgr8")
                image.header = self.msg_header
                self.image_pub.publish(image)
                
            if self.depth is not None:
                
                # self.depth = skimage.img_as_float(self.depth)
                depth = Image()
                depth = self.cvbridge.cv2_to_imgmsg(self.depth,"16UC1")
                depth.header = self.depth_msg_header
                self.depth_pub.publish(depth)

            # if self.info is not None:
            #     self.info_pub.publish(self.info)

            # rospy.loginfo('publishing image')
            # if self.info is not None:
                # self.info_pub.publish(self.info)

            self.rate.sleep()

def main(args):
    '''Initializes and cleans up ros node'''
    rospy.init_node('image_resizer', anonymous=True)
    node = image_resizer()
    node.start()
    

if __name__ == '__main__':
    main(sys.argv)