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
#include <string>

using namespace std;

template <class T>
using param_pair = pair<string, T>;


namespace uwsim
{

    template<typename T>
    inline void get_param(ros::NodeHandle &nh, pair<string, T> &param_pair_) {
        nh.getParam(param_pair_.first, param_pair_.second);
    }

        /* Rotation Operations */
//    using namespace Eigen;

    class UWSim_Sensor
    {
    public:
        UWSim_Sensor();

        ~UWSim_Sensor() {}

    protected:

        void start(void);

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
        param_pair<double> _hil_sensor_period_sec{"hil_sensor_period", {}};
        param_pair<double> _hil_quaternion_period_sec{"hil_quaternion_period", {}};
        param_pair<double> _hil_battery_period_sec{"hil_battery_period", {}};

        param_pair<float> _sen_accelerometer_std{"sensor/accelerometer_std", {}};
        param_pair<float> _sen_acc_noise_density{"sensor/accelerometer_noise_density", {}};
        param_pair<float> _sen_acc_bias_diffusion{"sensor/accelerometer_bias_diffusion", {}};
        param_pair<float> _sen_gyro_std{"sensor/gyro_std", {}};
        param_pair<float> _sen_gyro_noise_density{"sensor/gryo_noise_density", {}};
        param_pair<float> _sen_gyro_bias_diffusion{"sensor/gyro_bias_diffusion", {}};
        param_pair<float> _sen_mag_std{"sensor/mag_std", {}};
        param_pair<float> _sen_mag_inclination{"sensor/mag_inclination", {}};
        param_pair<float> _sen_mag_declination{"sensor/mag_declination", {}};
        param_pair<float> _sen_pressure_ref{"sensor/pressure_ref", {}};
        param_pair<float> _sen_pressure_std{"sensor/pressure_std", {}};
        param_pair<float> _sen_temp_ref{"sensor/temp_ref", {}};
        param_pair<float> _sen_temp_std{"sensor/temp_std", {}};

        param_pair<float> _att_quaternion_std{"att/quaternion_std", {}};
        param_pair<float> _att_omega_std{"att/omega_std", {}};
        param_pair<float> _att_acceleration_std{"att/acceleration_std", {}};

        Sensor::SensorDynamics *_sensor_dyn;
    };
}

