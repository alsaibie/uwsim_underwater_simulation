//
// Created by alsaibie on 5/28/18.
//

#pragma once

#include "ros/ros.h"
#include "AUVDynamics.hpp"
#include <uwsim_msgs/full_state.h>
#include <uwsim_msgs/power.h>
#include <uwsim_msgs/dynamics_param.h>
#include <uwsim_msgs/state_control.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <string>
using namespace std;

template <class T>
using param_pair = pair<string, T>;

namespace uwsim {

    template<typename T>
    inline void get_param(ros::NodeHandle &nh, pair<string, T> &param_pair_) {
            nh.getParam(param_pair_.first, param_pair_.second);
    }

    class Dynamics_Node {
    public:
        Dynamics_Node();
        ~Dynamics_Node() {};

    private:
        void start(void);

        ros::NodeHandle _nh;

        /** ROS Subscribers */
        ros::Subscriber _v_thrusters_input_sub;
        ros::Subscriber _state_control_sub;

        /** ROS Publishers */
        ros::Publisher _v_uwsim_pose_ros_pub;
        ros::Publisher _v_full_state_ros_pub;
        ros::Publisher _v_power_ros_pub;
        ros::Publisher _v_dyn_param_ros_pub;

        /** ROS Callbacks */
        void thrusters_input_callback(const std_msgs::Float64MultiArray::ConstPtr &thruster_msg);
        void state_control_callback(const uwsim_msgs::state_control::ConstPtr &control_msg);

        /** Outgoing */
        void pub_vehicle_state(void);
        void pub_uwsim_pose_state(void);
        void pub_uwsim_dyn_param(void);

        /** ROS Outgoing msgs */
        uwsim_msgs::full_state  _state24_msg;
        uwsim_msgs::power       _power_msg;
        geometry_msgs::Pose     _uwsim_pose_sp_msg;
        uwsim_msgs::dynamics_param         _uwsim_dyn_param_msg;

        /** ROS Parameter List */
        std::string _vehicle_name;

        /* simulation specific param */
        /* node specific param */

        /** */
        param_pair<double> _diffq_period{"dynamics/diffq_period", {}};
        param_pair<double> _pub_period{"dynamics/pub_period", {}};

        /* simulation specific param */
        /* inertia param */
        param_pair<double> _mass{"dynamics/mass", {}};
        param_pair<vector<double>> _rG{"dynamics/gravity_center", {{}}};
        param_pair<vector<double>> _rB{"dynamics/buoyancy_center", {{}}};
        param_pair<double> _g{"dynamics/g", {}};
        param_pair<vector<double>> _radius{"dynamics/radius", {{}}};
        param_pair<vector<double>> _tensor{"dynamics/tensor", {{}}};

        /* damping param */
        param_pair<vector<double>> _ksurge_coefficients{"dynamics/damping_ksurge", {{}}};
        param_pair<vector<double>> _ksway_coefficients{"dynamics/damping_ksway", {{}}};
        param_pair<vector<double>> _kheave_coefficients{"dynamics/damping_kheave", {{}}};
        param_pair<vector<double>> _kroll_coefficients{"dynamics/damping_kroll", {{}}};
        param_pair<vector<double>> _kpitch_coefficients{"dynamics/damping_kpitch", {{}}};
        param_pair<vector<double>> _kyaw_coefficients{"dynamics/damping_kyaw", {{}}};

        param_pair<double> _density{"dynamics/density", {}};

        /* actuator param */
        param_pair<int> _num_actuators{"dynamics/num_actuators", {}};
        param_pair<vector<double>> _kF_coefficients{"dynamics/actuators_kf", {{}}};
        param_pair<vector<double>> _kM_coefficients{"dynamics/actuators_km", {{}}};
        param_pair<vector<double>> _kV_coefficients{"dynamics/actuators_kV", {{}}};
        param_pair<vector<double>> _kI_coefficients{"dynamics/actuators_kI", {{}}};
        param_pair<double> _actuators_r{"dynamics/actuators_radius", {}};
        param_pair<double> _actuators_tconst{"dynamics/actuators_tconst", {}};
        param_pair<double> _actuators_maxsat{"dynamics/actuators_maxsat", {}};
        param_pair<double> _actuators_minsat{"dynamics/actuators_minsat", {}};
        param_pair<vector<int>> _actuators_dir_inversion{"dynamics/actuators_inversion", {{}}};
        param_pair<vector<int>> _actuators_pwm_range{"dynamics/actuators_pwm", {{}}};

        /* battery param */
        param_pair<vector<double>> _kBd_coefficients{"battery/discharge_K", {{}}};
        param_pair<double> _bat_c_vmax{"battery/cell_Vmax", {}};
        param_pair<double> _bat_c_vcut{"battery/cell_Vcut", {}};
        param_pair<double> _bat_esr{"battery/ESR", {}};
        param_pair<double> _bat_cell_n{"battery/cell_n", {}};
        param_pair<int> _bat_mAh{"battery/capacity_mAh", {}};

        /* initial state param */
        param_pair<vector<double>> _p_initial{"dynamics/initial_pose", {{}}};
        param_pair<vector<double>> _v_initial{"dynamics/initial_velocity", {{}}};

        /** utilities */
        void set_parameters(void);
        void reset(void);

        /** Dynamics Handle */
        Dynamics::AUVDynamics *_auv_dyn;
    };
}