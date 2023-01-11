#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
from math import floor

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
def aruco_centre(corners):
    x = (corners[0][0][0][0] + corners[0][0][2][0])/2
    y = (corners[0][0][0][1] + corners[0][0][2][1])/2
    return [x,y]
    
def img_call(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    if corners:
        aruco.drawDetectedMarkers(cur_frame, corners)
        cntr=aruco_centre(corners)
        cv2.arrowedLine(cur_frame, (320, 240), (int(floor(cntr[0])), int(floor(cntr[1]))), (255, 0, 0), 3)

    cv2.imshow('image feed',cur_frame)
    cv2.waitKey(1)

def subs():
    rospy.init_node('img_listener',anonymous=False)
    rospy.Subscriber('/webcam/image_raw',Image,img_call)
    rospy.spin()
    cv2.destroyAllWindows()
    
if __name__=='__main__':
    subs()
