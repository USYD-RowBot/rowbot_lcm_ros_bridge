#include <stdio.h>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_relay_t.hpp"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


class NavBridge
{
    public:
        NavBridge()
        {
            odomPub_ = n_.advertise<nav_msgs::Odometry>("/odom", 1);
            fixPub_ = n_.advertise<sensor_msgs::NavSatFix>("/fix", 1);
            imuPub_ = n_.advertise<sensor_msgs::Imu>("/imu", 1);
            modePub_ = n_.advertise<std_msgs::String>("/vessel_mode", 1);
        }
        ~NavBridge() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const acfrlcm::auv_acfr_nav_t *nav)
        {
            if(ros::ok)
            {
                std_msgs::Header header_msg;
                header_msg.stamp =  ros::Time::now();
                header_msg.seq = seq_;
                seq_++; // Increment the sequence

                tf2::Quaternion quat;
                quat.setRPY(nav->pitch, nav->roll, -NavBridge::unwrap(nav->heading - M_PI_2)); //TODO convert to ENU
                quat.normalize();

                // Fill the Odometry message
                nav_msgs::Odometry odom_msg;
                odom_msg.header = header_msg;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id ="base_link";
                odom_msg.pose.pose.position.x = nav->y;
                odom_msg.pose.pose.position.y = nav->x;
                odom_msg.pose.pose.position.z = -nav->depth;
                odom_msg.pose.pose.orientation.x = quat[0];
                odom_msg.pose.pose.orientation.y = quat[1];
                odom_msg.pose.pose.orientation.z = quat[2];
                odom_msg.pose.pose.orientation.w = quat[3];

                odom_msg.twist.twist.linear.x = nav->vy;
                odom_msg.twist.twist.linear.y = nav->vx;
                odom_msg.twist.twist.linear.z = -nav->vz;
                odom_msg.twist.twist.angular.x = nav->pitchRate;
                odom_msg.twist.twist.angular.y = nav->rollRate;
                odom_msg.twist.twist.angular.z = -nav->headingRate;

                // Publish the odom message
                odomPub_.publish(odom_msg);

                // IMU
                sensor_msgs::Imu imu_msg;
                imu_msg.orientation.x = quat[0];
                imu_msg.orientation.y = quat[1];
                imu_msg.orientation.z = quat[2];
                imu_msg.orientation.w = quat[3];

                imu_msg.angular_velocity.x = nav->pitchRate;
                imu_msg.angular_velocity.y = nav->rollRate;
                imu_msg.angular_velocity.z = -nav->headingRate;

                imuPub_.publish(imu_msg);

                // Fix Publish
                sensor_msgs::NavSatFix fix_msg;
                fix_msg.header = header_msg;
                fix_msg.header.frame_id = "novatel";
                fix_msg.latitude = nav->latitude;
                fix_msg.longitude = nav->longitude;
                fix_msg.altitude = nav->altitude;
                fixPub_.publish(fix_msg);

                // Send the transform
                static tf2_ros::TransformBroadcaster br;
                geometry_msgs::TransformStamped trStamp;
                trStamp.header = header_msg;
                trStamp.header.frame_id = "odom";
                trStamp.child_frame_id = "base_link";
                trStamp.transform.translation.x = odom_msg.pose.pose.position.x;
                trStamp.transform.translation.y = odom_msg.pose.pose.position.y;
                trStamp.transform.translation.z = odom_msg.pose.pose.position.z;
                trStamp.transform.translation.z = odom_msg.pose.pose.position.z;
                trStamp.transform.rotation.x = quat.x();
                trStamp.transform.rotation.y = quat.y();
                trStamp.transform.rotation.z = quat.z();
                trStamp.transform.rotation.w = quat.w();

                br.sendTransform(trStamp);
            }

        }
        // Keeps the angle between -pi, pi
        double unwrap(double x)
        {
            x = fmod(x + M_PI,2*M_PI);
            if (x < 0)
                x += 2*M_PI;
            return x - M_PI;
        }


        void handleMode(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const acfrlcm::auv_relay_t *msg)
        {
            if(ros::ok)
            {
                std::string mode_colour;
                if(msg->state == 0)
                {
                    mode_colour = "red";
                }
                else if(msg->state == 1)
                {
                    mode_colour = "orange";
                }
                else if(msg->state == 2)
                {
                    mode_colour = "green";
                }
                std_msgs::String mode_msg;
                mode_msg.data = mode_colour;
                modePub_.publish(mode_msg);
            }
        }
         
    private:
        ros::NodeHandle n_;
        ros::Publisher odomPub_;
        ros::Publisher imuPub_;
        ros::Publisher fixPub_;
        ros::Publisher modePub_;
        int seq_;

};

int main(int argc, char** argv)
{
    lcm::LCM lcm;

    ros::init(argc, argv, "nav_bridge");

    if(!lcm.good())
        return 1;

    NavBridge nb;
    lcm.subscribe("WAMV.ACFR_NAV", &NavBridge::handleMessage, &nb);
    lcm.subscribe("WAMV.ACFR_NAV", &NavBridge::handleMode, &nb);

    while(0 == lcm.handle());

    return 0;
}
