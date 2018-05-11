//
// Created by alsaibie on 5/7/18.
//

#pragma once

#include "ros/ros.h"
#include <mavconn/interface.h>
/* ROS Messages */
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>

namespace px4
{

    class Mavlink
    {
    public:
        Mavlink(std::string mavlink_fcu_url);

        ~Mavlink() {}

    protected:
        mavconn::MAVConnInterface::Ptr _mavconnlink;
        ros::NodeHandle _nh;


        /* Subscriptions */
        ros::Subscriber _v_attitude_quaternion_ros_sub;

        /* Publishers */
        ros::Publisher _mixed_motor_cmd_ros_pub;


        /* ROS Callbacks */

        /**
         * Relays Attitude Information from UWSim to PX4
         * */
        void VehicleAttitudeCallback(const sensor_msgs::Imu::ConstPtr& att_q);


        /*MAV handles */

        /**
         * Handle incoming mavlink messages ant publish them to ROS ("Mavlink Receiver")
         *
         * */
        void handle_msg(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);

        /**
         * Handle ACTUATOR_OUTPUT
         *
         * */
        void handle_msg_actuator_output(const mavlink::mavlink_message_t *msg);

    };

}
