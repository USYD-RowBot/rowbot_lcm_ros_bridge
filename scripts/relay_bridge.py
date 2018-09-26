#!/usr/bin/env python
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
import lcm
import rospy
from acfrlcm import relay_command_t, relay_status_t
from rowbot_lcm_ros_bridge.srv import RelayCommand
from rowbot_lcm_ros_bridge.msg import RelayStatus
lc = lcm.LCM()
def sendRelayCommand(req):
    rospy.logdebug("Sending Relay Command")
    relay_msg_lcm = relay_command_t()
    relay_msg_lcm.utime  = long(rospy.get_time())*1000000
    relay_msg_lcm.relay_number = req.relay_number
    relay_msg_lcm.relay_request = req.relay_request
    relay_msg_lcm.relay_off_delay = req.relay_off_delay
    relay_msg_lcm.io_number = req.io_number
    relay_msg_lcm.io_request=req.io_request
    lc.publish("WAMV.RELAY_COMMAND",relay_msg_lcm.encode())
    return true

def recieveRelayStatus(channel,relay_status):
    rospy.logdebug("Recieved lcm relay status")
    relay_status_msg_lcm = relay_status_t.decode(relay_status)
    seq = 0
    relay_status_msg_ros = RelayStatus()
    relay_status_msg_ros.header.stamp  = rospy.Time.now()
    relay_status_msg_ros.header.seq = seq;
    for i in relay_status_msg_lcm.state_list:

    #TODO CONVERT ARRAY OF BYTES TO LIST OF BOOLS
    pub = rospy.Publisher('relay/status',RelayStatus,queue_size=10)
    pub.publish(relay_status_msg_ros);

if __name__ == "__main__":
    rospy.init_node("relay_command_bridge")
    s = rospy.Service('relay_command', RelayCommand, sendRelayCommand)
    rospy.logdebug("Initalized relay_command_service")
    subscription = lc.subscribe("WAMV.RELAY_STATUS",recieveRelayStatus)
    rospy.loginfo("Initalied relay_lcm_ros_bridge");
    while( not rospy.is_shutdown()):
        lc.handle()
