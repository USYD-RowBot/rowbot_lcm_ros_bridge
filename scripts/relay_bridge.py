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
    lc.publish("WAMV.RELAY_CONTROL",relay_msg_lcm.encode())
    return True

def recieveRelayStatus(channel,relay_status):
    rospy.logdebug("Recieved lcm relay status")
    relay_status_msg_lcm = relay_status_t.decode(relay_status)
    seq = 0
    relay_status_msg_ros = RelayStatus()
    relay_status_msg_ros.header.stamp  = rospy.Time.now()
    relay_status_msg_ros.header.seq = seq;
    binary_string = ""
    for x in relay_status_msg_lcm.state_list:
        binary = ord(x)
        binary_string = binary_string + format(binary,'08b')
    rospy.logdebug(binary_string)
    output_list = []
    for i in binary_string:
        if i == "1":
            output_list.append(True)
        else:
            output_list.append(False)
    relay_status_msg_ros.state_list = output_list
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
