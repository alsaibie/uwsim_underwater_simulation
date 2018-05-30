//
// Created by alsaibie on 5/7/18.
//

#pragma once

#include "ros/ros.h"
#include "Sensor.hpp"
/* ROS Messages */
#include <uwsim_msgs/full_state.h>
#include <uwsim_msgs/hil_sensor.h>
#include <uwsim_msgs/hil_quaternion.h>
#include <uwsim_msgs/hil_battery.h>
#include <uwsim_msgs/power.h>

namespace uwsim
{
    /* Rotation Operations */
    using namespace Eigen;

    class UWSim_Sensor
    {
    public:
        UWSim_Sensor();

        ~UWSim_Sensor() {}

        void start(void);

    protected:

        void set_parameters();

        ros::NodeHandle _nh;

        /** ROS Subscribers */
        ros::Subscriber _v_full_state_ros_sub;
        ros::Subscriber _v_power_ros_sub;

        /** ROS Publishers */
        ros::Publisher _v_hil_sensor_ros_pub;
        ros::Publisher _v_hil_quaternion_ros_pub;
        ros::Publisher _v_hil_battery_ros_pub;

        /* ROS Callbacks */
        void vehicle_state_callback(const uwsim_msgs::full_state::ConstPtr &state22_msg);
        void vehicle_power_callback(const uwsim_msgs::power::ConstPtr &power);

        /** ROS Outgoing */
        void send_hil_sensor_msg(void);
        void send_hil_quaternion_msg(void);
        void send_hil_batt_msg(void);

        /** ROS Parameter List */
        std::string _vehicle_name;
        double _hil_sensor_period_sec;
        double _hil_quaternion_period_sec;
        double _hil_battery_period_sec;
        float _sen_accelerometer_std;
        float _sen_acc_noise_density;
        float _sen_acc_bias_diffusion;
        float _sen_gyro_std;
        float _sen_gyro_noise_density;
        float _sen_gyro_bias_diffusion;
        float _sen_mag_std;
        float _sen_mag_inclination;
        float _sen_mag_declination;
        float _sen_pressure_ref;
        float _sen_pressure_std;
        float _sen_temp_ref;
        float _sen_temp_std;
        float _att_quaternion_std;
        float _att_omega_std;
        float _att_acceleration_std;

        SensorDynamics *_sensor_dyn;
    };
}
