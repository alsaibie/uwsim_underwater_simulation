//
// Created by alsaibie on 5/7/18.
//

#pragma once

#include "ros/ros.h"
/* ROS Messages */
#include <uwsim_msgs/full_state.h>
#include <uwsim_msgs/hil_sensor.h>
#include <uwsim_msgs/hil_quaternion.h>
#include <uwsim_msgs/hil_battery.h>
#include <uwsim_msgs/power.h>

namespace uwsim
{
    class Sensor
    {
    public:
        Sensor();

        ~Sensor() {}

        /* Utilities */
        void start(void);

    protected:
        ros::NodeHandle _nh;

        /* Subscribers */
        ros::Subscriber _v_full_state_ros_sub;
        ros::Subscriber _v_power_ros_sub;

        /* Publishers */
        ros::Publisher _v_hil_sensor_ros_pub;
        ros::Publisher _v_hil_quaternion_ros_pub;
        ros::Publisher _v_hil_battery_ros_pub;

        /* ROS Callbacks */
        void VehicleStateCallback(const uwsim_msgs::full_state::ConstPtr& state22_msg);
        void VehiclePowerCallback(const uwsim_msgs::power::ConstPtr& power);

        /* msg keepers */
        uwsim_msgs::full_state     _state22;
        uwsim_msgs::power          _power;

        /* Outgoing */
        void send_hil_sensor_msg(void);
        void send_hil_quaternion_msg(void);
        void send_hil_batt_msg(void);

        /* Parameter List */
        std::string _vehicle_name;
        double _hil_sensor_period_sec;
        double _hil_quaternion_period_sec;
        double _hil_battery_period_sec;
        float _sen_accelerometer_std;
        float _sen_gyro_std;
        float _sen_mag_std;
        float _att_quaternion_std;
        float _att_omega_std;
        float _att_acceleration_std;

        /* time services */
        double _last_hil_sensor_sec;
        double _last_hil_quaternion_sec;
        double _last_hil_battery_sec;
    };
}
