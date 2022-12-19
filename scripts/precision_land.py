#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

# Callback function to handle state updates
def state_callback(state_msg):
    global current_state
    current_state = state_msg

# Callback function to handle marker position updates
def marker_callback(marker_msg):
    global marker_pose
    marker_pose = marker_msg

# Function to arm the autopilot and switch to offboard mode
def set_guided_mode():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        arm_service(True)
    except rospy.ServiceException as e:
        print("Arming service call failed: %s" % e)

    rospy.wait_for_service('mavros/set_mode')
    try:
        mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
        mode_service(custom_mode='OFFBOARD')
    except rospy.ServiceException as e:
        print("Set mode service call failed: %s" % e)

# Function to control the drone's movement based on the marker position
def control_movement():
    # Set the desired position above the marker
    setpoint_pose = PoseStamped()
    setpoint_pose.pose.position.x = marker_pose.pose.position.x
    setpoint_pose.pose.position.y = marker_pose.pose.position.y
    setpoint_pose.pose.position.z = marker_pose.pose.position.z + 0.2
    setpoint_pose.header.stamp = rospy.Time.now()
    setpoint_pose.header.frame_id = "map"

    # Publish the setpoint position
    setpoint_pub.publish(setpoint_pose)

if __name__ == '__main__':
    # Initialize ROS node and subscribers
    rospy.init_node('precision_landing')
    state_sub = rospy.Subscriber('mavros/state', State, state_callback)
    marker_sub = rospy.Subscriber('marker_pose', PoseStamped, marker_callback)

    # Initialize publisher for setpoint position
    setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Set the desired position above the marker
    setpoint_pose = PoseStamped()
    setpoint_pose.pose.position.x = 0.0
    setpoint_pose.pose.position.y = 0.0
    setpoint_pose.pose.position.z = 2.0
    setpoint_pose.header.stamp = rospy.Time.now()
    setpoint_pose.header.frame_id = "map"

    # Set the rate at which setpoint positions will be published (in Hz)
    rate = rospy.Rate(10)

    # Arm the autopilot and switch to offboard mode
    set_guided_mode()

    # Wait until the autopilot is in offboard mode
    while not current_state.mode == "GUIDED":
        rate.sleep()

    # Main loop to control the drone's movement
    while not rospy.is_shutdown():
        # If the marker is detected, update the setpoint position based on its position
        if marker_pose is not None:
            control_movement()
        else:
            # If the marker is not detected, use the default setpoint position
            setpoint_pub.publish(setpoint_pose)

        rate.sleep()