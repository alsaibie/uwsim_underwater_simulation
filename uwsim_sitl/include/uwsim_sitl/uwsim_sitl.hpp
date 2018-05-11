//
// Created by alsaibie on 5/7/18.
//

#pragma once

#include "ros/ros.h"
#include <mavconn/interface.h>

/* ROS Messages */
#include <std_msgs/Float64MultiArray.h>
#include <uwsim_msgs/hil_sensor.h>
#include <uwsim_msgs/hil_quaternion.h>
#include <uwsim_msgs/hil_battery.h>

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
        ros::Subscriber _v_hil_sensor_ros_sub;
        ros::Subscriber _v_hil_quaternion_ros_sub;

        /* Publishers */
        ros::Publisher _mixed_motor_cmd_ros_pub;

        /* ROS Callbacks */
        void vehicle_hil_sensor_callback(const uwsim_msgs::hil_sensor::ConstPtr &sen_msg);
        void vehicle_hil_quaternion_callback(const uwsim_msgs::hil_quaternion::ConstPtr &att_msg);

        /*MAV handles */
        void handle_msg(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);
        void handle_msg_actuator_output(const mavlink::mavlink_message_t *msg);

    };

}
