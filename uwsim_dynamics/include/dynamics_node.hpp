//
// Created by alsaibie on 5/28/18.
//

#pragma once

#include "ros/ros.h"
#include "AUVDynamics.hpp"
#include <uwsim_msgs/full_state.h>
#include <uwsim_msgs/power.h>
#include <uwsim_msgs/dynamics_param.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>

namespace uwsim {

    class Dynamics_Node {
    public:
        Dynamics_Node();
        ~Dynamics_Node() {};

    private:
        void start(void);
        ros::NodeHandle _nh;

        /** ROS Subscribers */
        ros::Subscriber _v_thrusters_input_sub;

        /** ROS Publishers */
        ros::Publisher _v_uwsim_pose_ros_pub;
        ros::Publisher _v_full_state_ros_pub;
        ros::Publisher _v_power_ros_pub;

        /** ROS Callbacks */
        void thrusters_input_callback(const std_msgs::Float64MultiArray::ConstPtr &thruster_msg);

        /** Outgoing */
        void pub_vehicle_state(void);
        void pub_uwsim_pose_state(void);

        /** ROS Outgoing msgs */
        uwsim_msgs::full_state  _state24_msg;
        uwsim_msgs::power       _power_msg;
        geometry_msgs::Pose     _uwsim_pose_sp_msg;

        /** ROS Parameter List */
        std::string _vehicle_name;

        /* node specific param */
        double _diffq_period {};
        double _uwsim_period {};

        /* simulation specific param */
        /* inertia param */
        double _mass {};
        vector<double> _rG {};
        vector<double> _rB {};
        double _g {};
        vector<double> _radius {};
        vector<double> _tensor {};

        /* damping param */
        vector<double> _damping {};
        vector<double> _quadratic_damping {};

        double _dzv {};
        double _dv {};
        double _dh {};
        double _density {};

        /* actuator param */
        int _num_actuators {};
        vector<double> _kF_coefficients {};
        vector<double> _kM_coefficients {};
        vector<double> _kV_coefficients {};
        vector<double> _kI_coefficients {};
        vector<double> _kBd_coefficients {};
        double _actuators_r {};
        double _actuators_tconst {};
        double _actuators_maxsat {};
        double _actuators_minsat {};
        vector<int> _actuators_dir_inversion {};
        vector<int> _actuators_pwm_range {};

        /* battery param */
        double _bat_c_vmax {};
        double _bat_c_vcut {};
        double _bat_esr {};
        double _bat_cell_n {};
        int _bat_mAh {};

        /* initial state param */
        vector<double> _p_initial;
        vector<double> _v_initial;

        /** utilities */
        void set_parameters(void);
        void reset(void);

        /** Dynamics Handle */
        AUVDynamics *_auv_dyn;
    };
}