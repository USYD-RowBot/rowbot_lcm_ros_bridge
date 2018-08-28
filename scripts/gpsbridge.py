#!/usr/bin/env python

import lcm
from senlcm import novatel_t
import rospy
from sensor_msgs import NavSatFix
from sensor_msgs import NavSatStatus
from std_msgs import Header
seq = 0



def my_handler(channel, data):
    pub = rospy.Publisher('rowbot/fix', String, queue_size=10)

    #Read the LCM Data
    msg = senlcm_gpsd3_t.decode(data)
    utime = msg.utime
    status = msg.online
    fix = gpsd3_fix_t.decode(msg.fix)
    latitude = fix.latitiude
    longitude = fix.longitude
    altitiude = fix.altitude
    speed = fix.speed
    climb = fix.climb
    tag = msg.tag

    #Write Data to ROS Message
    rosmsg = NavSatFix()
    rosmsg.latitude = latitiude
    rosmsg.longitude = longitude
    rosmsg.altitude = alititude
    rosmsg.position_covariance = [0,0,0,0,0,0,0,0,0]
    rosmsg.position_covariance_type = 0

    statusmsg = NavSatStat()
    if status == 0:
        statusmsg.status = -1
    else:
        statusmsg.status = 0
    #USING GPS
    statusmsg.service = 1

    rosmsg.status = statusmsg

    headermsg = Header()
    seq = seq+1;
    headermsg.seq = seq
    headermsg.stamp = utime
    headermsg.frame_id = tag

    pub.publish(msg)

lc = lcm.LCM()
rospy.init_node('lcmtoRosFix', anonymous=True)

subscription = lc.subscribe("WAMV.GPSD_CLIENT", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
