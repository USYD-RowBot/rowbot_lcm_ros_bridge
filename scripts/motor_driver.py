#!/usr/bin/env python
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
import rospy
import lcm

from acfrlcm import asv_torqeedo_motor_command_t
from robotx_gazebo import UsvDrive
from std_msgs import Float64


def callback_left(message):
    port_channel  = "PORT_MOTOR_CONTROL"
    port = message.data
    port_msg = asv_torqeedo_motor_command_t()
    utime = long(rospy.get_time())*1000000

    port_msg.command_speed = port
    port_msg.enabled = True
    port_msg.utime = utime
    rospy.logdebug("Publishing port")
    lc.publish(port_channel, port_msg.encode())

def callback_right(message):
    utime = long(rospy.get_time())*1000000
    starboard_channel  = "STBD_MOTOR_CONTROL"
    starboard = message.data
    starboard_msg = asv_torqeedo_motor_command_t()
    starboard_msg.command_speed = starboard
    starboard_msg.enable = True
    starboard_msg.utime = utime
    rospy.logdebug("Publising starboard")
    lc.publish(starboard_channel, starboard_msg.encode())



if __name__ = "__main__":
    rospy.init("motor_lcm_driver")
    rospy.Subscriber("left_motor", Float64, callback_left)
    rospy.Subscriber("right_motor", Float64, callback_right)
    #rospy.Subscriber("cmd_drive", UsvDrive, callback)
    rospy.spin()
