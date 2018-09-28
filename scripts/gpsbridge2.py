#!/usr/bin/env python2
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')
import lcm
import math
from acfrlcm import auv_acfr_nav_t
import rospy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msgs import Imu
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
    if use_fix = True:
        rospy.lodebug("Publising gps info")
        fixmsg = NavSatFix()
        fixmsg.header = headermsg
        fixmsg.latitude = math.radians(msg.latitude)
        fixmsg.longitude = math.radians(msg.longitude)
        fixmsg.alitiude = msg.altitude
        fixmsg.position_covariance = [0,0,0,0,0,0,0,0,0]
        fixmsg.position_covariance_type = 0
        fix_pub.publish(fixmsg)

    if use_imu = True:
        rospy.logdebug("Publishing imu")
        imumsg = Imu()
        q = quaternion_from_euler(msg.roll, msg.pitch, msg.heading)
        imumsg.orientation = q

        imumsg.angular_velocity.x = msg.rollRate
        imumsg.angular_velocity.y = msg.pitchRate
        imumsg.angular_velocity.z = msg.headingRate

        imu_pub.publish(imumsg)
    if use_odom = True:
        rospy.logdebug("Publishing odom")
        odommsg = Odometry()
        odomsg.header = headermsg
        odomsg.header.frame_id = "odom"

        #TODO, USE TRANSFORM
        odomsg.child_frame_id = base_link
        odommsg.pose.pose.position.x = msg.x
        odommsg.pose.pose.position.y = msg.y
        odommsg.pose.pose.position.z = msg.altitude
        odom.pose.pose.orientation = q
        odommsg.twist.twist.linear.x = msg.vx
        odommsg.twist.twist.linear.y = msg.vy
        odommsg.twist.twist.linear.z = msg.vz
        odommsg.twist.twist.angular.x = msg.rollRate
        odommsg.twist.twist.angular.y = msg.pitchRate
        odommsg.twist.twist.angular.z = msg.headingRate

        odom_pub.publish(odommsg)



lc = lcm.LCM()
rospy.init_node('lcmtoRosFix', anonymous=True)

#TODO GET CHANNEL FROM PARAM
subscription = lc.subscribe("WAMV.ACFRNAV", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
