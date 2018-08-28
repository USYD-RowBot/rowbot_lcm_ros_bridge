#!/usr/bin/env python
import rospy
import lcm

from acfrlcm import asv_torqeedo_motor_command_t
from robotx_gazebo import UsvDrive


def callback(message):
    starboard_channel  = "STBD_MOTOR_CONTROL"
    port_channel  = "PORT_MOTOR_CONTROL"
    port = message.left
    starboard = message.right
    port_msg = asv_torqeedo_motor_command_t()
    starboard_msg = asv_torqeedo_motor_command_t()
    utime = long(rospy.get_time())*1000000

    port_msg.command_speed = port
    starboard_msg.command_speed = starboard
    port_msg.enabled = True
    starboard_msg.enable = True
    port_msg.utime = utime
    starboard_msg.utime = utime
    lc.publish(starboard_channel, starboard_msg.encode())
    lc.publish(port_channel, port_msg.encode())


if __name__ = "__main__":
    rospy.init("motor_lcm_driver")
    rospy.Subscriber("cmd_drive", UsvDrive, callback)
    rospy.spin()
