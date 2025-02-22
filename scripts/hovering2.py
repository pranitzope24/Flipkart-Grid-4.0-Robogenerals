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

times=0
drone = gnc_api()
frame_centre = [320.,240.]
pixel_threshold = 10

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

def img_callback(data):
    br=CvBridge()
    cur_frame=br.imgmsg_to_cv2(data)
    gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    # while ids is not None and corners is not None  and times == 0: ## Phase 1
    if (times == 0) and ids is not None and corners is not None :
        gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
        aruco.drawDetectedMarkers(cur_frame, corners)
        velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
        vel = TwistStamped()

        if ids is None:
            pass

        global mp
        mp.clear()
    
        for i in range(len(ids)):
            mp[ids[i][0]]=[[corners[i][0]]]
        
        if 0 not in mp.keys():
            pass
        
        if corners==[]:
            pass

        if not (thres(mp[0])):
            hovering_phase()
            pass

        aruco.drawDetectedMarkers(cur_frame, corners)
        
        v_x,v_y = reqd_velo(mp[0],0.1)

        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
        
        velocity_pub.publish(vel)

    rospy.loginfo("sex")



    while ids is not None and corners is not None  and times == 1: ##### marker ka id is 0
        gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
        aruco.drawDetectedMarkers(cur_frame, corners)
        
        velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
        vel = TwistStamped()

        if ids is None:
            break

        global mp
        mp.clear()
        #one change here
        for i in range(len(ids)):
            mp[ids[i][0]]=[[corners[i][0]]]
        
        if 4 not in mp.keys():
            break

        if not (thres(mp[4])):
            hovering_phase()
            break

        aruco.drawDetectedMarkers(cur_frame, corners)
        
        v_x,v_y = reqd_velo(mp[4],0.1)

        vel.twist.linear.x = -v_y
        vel.twist.linear.y = -v_x
    
        velocity_pub.publish(vel)

    if(times > 1) :
        rtl()
        rospy.loginfo(CBOLD + CGREEN + "MISSION SUCCESSFUL, RETURNING TO BASE !" + CEND)
        

def hovering_phase():
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    poser=PoseStamped()
    cur_x = drone.current_pose_g.pose.pose.position.x
    cur_y = drone.current_pose_g.pose.pose.position.y

    poser.pose.position.x = cur_x
    poser.pose.position.y = cur_y
    poser.pose.position.z = 0.3
    pose_pub.publish(poser)
    rospy.loginfo("First pose was given")
    rospy.sleep(10)
    
    rospy.loginfo("Second pose was given")
    while (drone.current_pose_g.pose.pose.position.z < 1.95):
        poser.pose.position.z = 2
        pose_pub.publish(poser)

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
    rospy.Subscriber('/webcam/image_raw',Image,img_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # print(k)
    rospy.init_node("drone_controller", anonymous=True)
    takeoff_drone()
    recv()