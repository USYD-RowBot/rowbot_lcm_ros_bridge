#!/usr/bin/env python2
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
import lcm
from senlcm import novatel_t
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header
seq = 0



def my_handler(channel, data):
    global seq
    pub = rospy.Publisher('rowbot/fix', NavSatFix, queue_size=10)

    #Read the LCM Data
    msg = novatel_t.decode(data)
    utime = msg.utime
    status = msg.status
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = 0
    tag = "gps"
    #Write Data to ROS Message
    rosmsg = NavSatFix()
    rosmsg.latitude = latitude
    rosmsg.longitude = longitude
    rosmsg.altitude = altitude
    rosmsg.position_covariance = [0,0,0,0,0,0,0,0,0]
    rosmsg.position_covariance_type = 0

    headermsg = Header()
    seq = seq+1;
    headermsg.seq = seq
    headermsg.stamp = rospy.Time.now()
    headermsg.frame_id = tag
    rosmsg.header = headermsg
    
    print("Got GPS INFO")
    pub.publish(rosmsg)

lc = lcm.LCM()
rospy.init_node('lcmtoRosFix', anonymous=True)

subscription = lc.subscribe("NOVATEL", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
