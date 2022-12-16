#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 

def img_callback(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    cv2.circle(cur_frame,(150,150),5, (0, 0, 255), 4)
    cv2.imshow('feed',cur_frame)
    cv2.waitKey(1)

def recv():
    rospy.init_node('img_listener',anonymous=True)
    rospy.Subscriber('/webcam/image_raw',Image,img_callback)
    rospy.spin()
    cv2.destroyAllWindows()
    
    
if __name__=='__main__':
    recv()

