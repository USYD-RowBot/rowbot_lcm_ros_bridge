#!/usr/bin/env python
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
import rospy
import lcm

from acfrlcm import asv_torqeedo_motor_command_t
from std_msgs.msg import Float32
lc = lcm.LCM()

def callback_left(message):
    port_channel  = "WAMV.PORT_MOTOR_CONTROL"
    port = message.data
    port_msg = asv_torqeedo_motor_command_t()
    utime = long(rospy.get_time())*1000000

    port_msg.command_speed = port*650
    port_msg.enabled = True
    port_msg.utime = utime
    rospy.logdebug("Publishing port")
    lc.publish(port_channel, port_msg.encode())

def callback_right(message):
    utime = long(rospy.get_time())*1000000
    starboard_channel  = "WAMV.STBD_MOTOR_CONTROL"
    starboard = message.data
    starboard_msg = asv_torqeedo_motor_command_t()
    starboard_msg.command_speed = starboard*650
    starboard_msg.enabled = True
    starboard_msg.utime = utime
    rospy.logdebug("Publising starboard")
    lc.publish(starboard_channel, starboard_msg.encode())



if __name__ == "__main__":
    rospy.init_node("motor_lcm_driver")
    rospy.loginfo("Started lcm_driver")
    rospy.Subscriber("left_thrust_cmd", Float32, callback_left)
    rospy.Subscriber("right_thrust_cmd", Float32, callback_right)
    #rospy.Subscriber("cmd_drive", UsvDrive, callback)
    rospy.spin()
