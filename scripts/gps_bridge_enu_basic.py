#!/usr/bin/env python2
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
import lcm
import math
import tf
from acfrlcm import auv_acfr_nav_t
import rospy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import numpy as np
seq = 0

def my_handler(channel, data):
    global seq
    fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
    odom_pub = rospy.Publisher('odom',Odometry,queue_size=10)
    imu_pub = rospy.Publisher('imu/data',Imu,queue_size=10)
    use_odom=True
    use_fix = True
    use_imu = True

    gps_frame = "novatel"
    headermsg = Header()
    headermsg.seq = seq +1
    headermsg.stamp = rospy.Time.now()
    headermsg.frame_id = gps_frame

    msg = auv_acfr_nav_t.decode(data)
    if use_fix== True:
        rospy.logdebug("Publising gps info")
        fixmsg = NavSatFix()
        fixmsg.header = headermsg
        fixmsg.latitude = math.radians(msg.latitude)
        fixmsg.longitude = math.radians(msg.longitude)
        fixmsg.altitude = msg.altitude
        fixmsg.position_covariance = [0,0,0,0,0,0,0,0,0]
        fixmsg.position_covariance_type = 0
        fix_pub.publish(fixmsg)

    if use_imu == True:
        rospy.logdebug("Publishing imu")
        imumsg = Imu()
        q = quaternion_from_euler(msg.pitch, msg.roll, -np.unwrap(msg.heading+np.pi/2)) # TODO check this
        imumsg.orientation.x = q[0]
        imumsg.orientation.y = q[1]
        imumsg.orientation.z = q[2]
        imumsg.orientation.w = q[3]

        imumsg.angular_velocity.x = msg.pitchRate
        imumsg.angular_velocity.y = msg.rollRate
        imumsg.angular_velocity.z = -msg.headingRate

        imu_pub.publish(imumsg)
    if use_odom == True:
        br = tf.TransformBroadcaster()
        q = quaternion_from_euler(msg.pitch, msg.roll, -np.unwrap(msg.heading+np.pi/2)) # TODO check this

        rospy.logdebug("Publishing odom")
        odommsg = Odometry()
        odommsg.header = headermsg
        odommsg.header.frame_id = "odom"

        #TODO, USE TRANSFORM
        odommsg.child_frame_id ="base_link"
        odommsg.pose.pose.position.x = msg.y
        odommsg.pose.pose.position.y = msg.x
        odommsg.pose.pose.position.z = -msg.altitude
        odommsg.pose.pose.orientation.x = q[0]
        odommsg.pose.pose.orientation.y = q[1]
        odommsg.pose.pose.orientation.z = q[2]
        odommsg.pose.pose.orientation.w = q[3]


        odommsg.twist.twist.linear.x = msg.vy
        odommsg.twist.twist.linear.y = msg.vx
        odommsg.twist.twist.linear.z = -msg.vz
        odommsg.twist.twist.angular.x = msg.pitchRate
        odommsg.twist.twist.angular.y = msg.rollRate
        odommsg.twist.twist.angular.z = -msg.headingRate

        odom_pub.publish(odommsg)
        br.sendTransform((msg.y, msg.x, 0),
                     tf.transformations.quaternion_from_euler(msg.pitch, msg.roll, -np.unwrap(msg.heading+np.pi/2)),
                     rospy.Time.now(),
                     "base_link",
                     "odom")



lc = lcm.LCM()
rospy.init_node('lcmtoRosFix', anonymous=True)

#TODO GET CHANNEL FROM PARAM
subscription = lc.subscribe("WAMV.ACFR_NAV", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
