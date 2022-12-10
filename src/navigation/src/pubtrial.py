#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
from py_gnc import *
from term_colors import *

def func():
    rospy.init_node("drone_controller", anonymous=True)
    drone = gnc_api()
    drone.initialize_local_frame()
    drone.wait4connect()
    drone.wait4start()

    drone.takeoff(5)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        # have to add waypoint_g = data sybscribed from setpoint topic
        a,b,c = drone.local_pos_pub.pose.pose.position.x,drone.local_pos_pub.pose.pose.position.y,drone.local_pos_pub.pose.pose.position.z
        drone.set_destination(a,b,c,0)
        rate.sleep()
        if(drone.check_waypoint_reached(0.5,0.1)):
            drone.land()
            break




if __name__ == '__main__':
    try:
        func()
    except KeyboardInterrupt:
        exit()