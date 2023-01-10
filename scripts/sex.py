#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest, SetModeResponse
from mavros_msgs.srv import CommandBoolResponse
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix

current_state = State()
current_global_pos = NavSatFix()
offb_set_mode = SetModeRequest()
arm_cmd = CommandBoolRequest()
global_pos = GlobalPositionTarget()

def state_cb(state):
    global current_state
    current_state = state

def global_pos_cb(global_pos):
    global current_global_pos
    current_global_pos = global_pos

def main():
    rospy.init_node("offb_node", anonymous=True)

    # Subscribers
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/global_position/global", NavSatFix, global_pos_cb)

    # Publishers
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/global", GlobalPositionTarget, queue_size=10)

    # Services
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # The desired global position for the UAV
    global_pos.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
    global_pos.altitude = 2.0  # Takeoff to 2 meters

    # Wait for FCU connection
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.sleep(1)

    # Set the UAV into GUIDED mode and arm the motors
    offb_set_mode.custom_mode = "GUIDED"
    arm_cmd.value = True
    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        # if current_state.mode != "GUIDED" and rospy.Time.now() - last_request > rospy.Duration(5.0):
        set_mode_response = set_mode_client(offb_set_mode)
        if set_mode_response.mode_sent:
            rospy.loginfo("GUIDED enabled")
            # last_request = rospy.Time.now()
        # else:
        #     if not current_state.armed and rospy.Time.now() - last_request > rospy.Duration(5.0):
        arm_response = arming_client(arm_cmd)
        if arm_response.success:
            rospy.loginfo("Vehicle armed")
        # last_request = rospy.Time.now()
        
        # Set the current global position as the desired position
        global_pos.latitude = current_global_pos.latitude
        global_pos.longitude = current_global_pos.longitude
        global_pos.altitude = 1.0  # Hover at 1 meter

        local_pos_pub.publish(global_pos)
        rospy.sleep(1)


if __name__ == '__main__':
    main()