#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
from py_gnc import *
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from term_colors import *
from geometry_msgs.msg import TwistStamped,PoseStamped
import time

times=0
drone = gnc_api()
frame_centre = [320.,240.]
pixel_threshold = 10
reached_marker_0 = 0
reached_marker_4 = 0

#    Marker Coordinates

#    x: -1.58653998375
#    y: 0.0905971601605

mp = {}

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

def aruco_centre(corners):
    x = (corners[0][0][0][0] + corners[0][0][2][0])/2
    y = (corners[0][0][0][1] + corners[0][0][2][1])/2
    return [x,y]

def reqd_velo(corners,param):
    lst=aruco_centre(corners)
    v_x=lst[0]-frame_centre[0]
    v_y=lst[1]-frame_centre[1]
    p=(v_x**2+v_y**2)**(0.5)
    v_x,v_y=v_x/p,v_y/p
    return param*v_x, param*v_y

def thres(corners):
    lst1=aruco_centre(corners)
    
    mag=((lst1[0]-frame_centre[0])**2+(lst1[1]-frame_centre[1])**2)**(0.5)
    if(mag>10):
        return mag
    return 0

def img_callback():
    global times
    global reached_marker_0
    global reached_marker_4


    cap=cv2.VideoCapture(2)
    ret,cur_frame=cap.read()
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    aruco.drawDetectedMarkers(cur_frame, corners)

    mp.clear()

    if ids is not None:
        for i in range(len(ids)):
            mp[ids[i][0]]=[[corners[i][0]]]
    
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    vel = TwistStamped()


    if (times == 0) and (0 not in mp.keys()):
        reached_marker_0 = 1
        # cv2.imshow('liveimg',cur_frame)
        # cv2.waitKey(1)

        vel.twist.linear.x = 0.2
        vel.twist.linear.y = 0
        vel.twist.linear.z = 0
        velocity_pub.publish(vel)
        return


    if (reached_marker_0 == 1):
        vel.twist.linear.x = 0
        vel.twist.linear.y = 0
        vel.twist.linear.z = 0
        
        times += 1
        velocity_pub.publish(vel)
        reached_marker_0 = 0


    if (0 in mp.keys()) and (times == 1): ## Phase 1
        if not (thres(mp[0])):
            hovering_phase()
            return

        aruco.drawDetectedMarkers(cur_frame, corners)
        
        v_x,v_y = reqd_velo(mp[0],0.2)

        vel.twist.linear.x = -v_x
        vel.twist.linear.y = -v_y
        
        velocity_pub.publish(vel)
        return

        

def hovering_phase():
    velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    vel = TwistStamped()
    
    duration = 5
    start = time.time()
    while time.time()-start < duration :
        vel.twist.linear.x=0
        vel.twist.linear.y=0
        vel.twist.linear.z=-0.2
        velocity_pub.publish(vel)

    global times
    times += 1
    return

def rtl():
    drone.set_mode("RTL")
    global times
    times += 1

def takeoff_drone(): 
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.takeoff(2)
    rate = rospy.Rate(3)
    rospy.loginfo(CGREEN2 + "Takeoff Completed" + CEND)

def recv():
    #rospy.Subscriber('/webcam/image_raw',Image,img_callback)
    while(not rospy.is_shutdown()):
        img_callback()
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # print(k)
    rospy.init_node("drone_controller", anonymous=True)
    takeoff_drone()
    recv()