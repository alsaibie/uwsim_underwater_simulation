//
// Created by alsaibie on 5/7/18.
//

#pragma once

#include "ros/ros.h"
#include <random>
#include <Eigen/Dense>
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
        void vehicle_state_callback(const uwsim_msgs::full_state::ConstPtr &state22_msg);
        void vehicle_power_callback(const uwsim_msgs::power::ConstPtr &power);

        /* msg keepers */
        uwsim_msgs::full_state     _state22;
        uwsim_msgs::power          _power;

        /* Outgoing */
        void send_hil_sensor_msg(double dt);
        void send_hil_quaternion_msg(double dt);
        void send_hil_batt_msg(double dt);

        /* Parameter List */
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

        /* rotation services */
        Quaterniond qmag_fu; // Magnetic inclination-declination - expressed in NED frame u
        Quaterniond Euler2Quaterniond(const Vector3d v);
        Vector3d Quaternion2eulerd(const Quaterniond q);
        Quaterniond QuaternionVectord(const Vector3d v);

        /* time services */
        double _last_hil_sensor_sec;
        double _last_hil_quaternion_sec;
        double _last_hil_battery_sec;

        /* Signal deprocessing */
        void acclerometer_real(Vector3d &acc, double dt);
        void gyroscope_real(Vector3d &gyro, double dt);
        void magnetometer_real(Vector3d &mag, double dt);

        /* stochastics */
        std::default_random_engine _rand_generator;
        std::normal_distribution<float> _sen_acc_dist;
        std::normal_distribution<float> _sen_gyro_dist;
        std::normal_distribution<float> _sen_mag_dist;
        std::normal_distribution<float> _sen_pressure_dist;
        std::normal_distribution<float> _sen_temp_dist;
    };
}
