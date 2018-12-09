#!/usr/bin/env python
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
import rospy
import lcm
import math
import numpy as np
from acfrlcm import asv_torqeedo_motor_command_t
from std_msgs.msg import Float32
lc = lcm.LCM()
right_time = 0
left_time = 0

def callback_left(message):
    global left_time
    left_time = rospy.get_time()
    # port_channel  = "WAMV.PORT_MOTOR_CONTROL"
    port_channel = "WAMV.LEFT_ROS_CMD"
    LIMIT = 1000.
    MULTIPLIER = 1.
    port = message.data*MULTIPLIER
    if port > LIMIT:
        port = LIMIT
    elif port < -LIMIT:
        port = -LIMIT
    port_msg = asv_torqeedo_motor_command_t()
    utime = long(rospy.get_time())*1000000
    if math.isnan(port) or port == np.nan or port == None:
        port = 0
    port_msg.command_speed = int(port)
    port_msg.enabled = True
    port_msg.utime = utime
    rospy.logdebug("Publishing port")
    lc.publish(port_channel, port_msg.encode())

def callback_right(message):
    global right_time
    right_time = rospy.get_time()
    utime = long(rospy.get_time())*1000000
    # starboard_channel = "WAMV.STBD_MOTOR_CONTROL"  # This controls the motors directly
    starboard_channel = "WAMV.RIGHT_ROS_CMD"
    MULTIPLIER = 1.
    LIMIT = 1000.
    starboard = message.data*MULTIPLIER
    if starboard > LIMIT:
        starboard = LIMIT
    elif starboard < -LIMIT:
        starboard = -LIMIT
    if math.isnan(starboard) or starboard == np.nan or starboard == None:
        starboard = 0
    starboard_msg = asv_torqeedo_motor_command_t()
    starboard_msg.command_speed = int(starboard)
    starboard_msg.enabled = True
    starboard_msg.utime = utime
    rospy.logdebug("Publising starboard")
    lc.publish(starboard_channel, starboard_msg.encode())


def publishZero(channel):
    msg = asv_torqeedo_motor_command_t()
    msg.command_speed = 0
    msg.enabled=False
    msg.utime = long(rospy.get_time())*1000000
    lc.publish(channel,msg.encode())

if __name__ == "__main__":
    rospy.init_node("motor_lcm_driver")
    rospy.loginfo("Started lcm_driver")
    rospy.Subscriber("left_thrust_cmd", Float32, callback_left)
    rospy.Subscriber("right_thrust_cmd", Float32, callback_right)
    #rospy.Subscriber("cmd_drive", UsvDrive, callback)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        if (rospy.get_time() - right_time)>5:
            publishZero("WAMV.RIGHT_ROS_CMD")
            rospy.logwarn("TIMEOUT ON RIGHT now: %f, right: %f, diff %f",rospy.get_time() ,right_time, rospy.get_time()-right_time)

        if (rospy.get_time() - left_time)>5:
            publishZero("WAMV.LEFT_ROS_CMD")
            rospy.logwarn("TIMEOUT ON LEFT")
        r.sleep()
