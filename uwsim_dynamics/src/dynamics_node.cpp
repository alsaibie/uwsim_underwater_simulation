//
// Created by alsaibie on 5/28/18.
//

#include "dynamics_node.hpp"
#include <math.h>
using namespace std;
using namespace Eigen;
using namespace uwsim;

Dynamics_Node::Dynamics_Node():
        _nh(),
        // TODO: abstract vehicle name in launch file
        _vehicle_name(std::string("dolphin")),
        _v_thrusters_input_sub( _nh.subscribe("/dolphin/dynamics/thruster_inputs", 1, &Dynamics_Node::thrusters_input_callback, this)),
        _v_uwsim_pose_ros_pub( _nh.advertise<geometry_msgs::Pose>("/dolphin/uwsim/pose_sp", 1)),
        _v_full_state_ros_pub( _nh.advertise<uwsim_msgs::full_state>("/dolphin/dynamics/full_state", 1)),
        _v_power_ros_pub(_nh.advertise<uwsim_msgs::power>("/dolphin/dynamics/power", 1))
{

    /* Get Parameters */
    _vehicle_name = "dolphin";
    _nh.getParam(_vehicle_name + "/dynamics" + "/diffq_period", _diffq_period);
    _nh.getParam(_vehicle_name + "/dynamics" + "/uwsim_period", _uwsim_period);
    _nh.getParam(_vehicle_name + "/dynamics" + "/mass", _mass);
    _nh.getParam(_vehicle_name + "/dynamics" + "/gravity_center", _rG);
    _nh.getParam(_vehicle_name + "/dynamics" + "/buoyancy_center", _rB);
    _nh.getParam(_vehicle_name + "/dynamics" + "/g", _g);
    _nh.getParam(_vehicle_name + "/dynamics" + "/radius", _radius);
    _nh.getParam(_vehicle_name + "/dynamics" + "/tensor", _tensor);
    _nh.getParam(_vehicle_name + "/dynamics" + "/damping", _damping);
    _nh.getParam(_vehicle_name + "/dynamics" + "/quadratic_damping", _quadratic_damping);
    _nh.getParam(_vehicle_name + "/dynamics" + "/dzv", _dzv);
    _nh.getParam(_vehicle_name + "/dynamics" + "/dv", _dv);
    _nh.getParam(_vehicle_name + "/dynamics" + "/dh", _dh);
    _nh.getParam(_vehicle_name + "/dynamics" + "/density", _density);
    _nh.getParam(_vehicle_name + "/num_actuators", _num_actuators);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_kf", _kF_coefficients);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_km", _kM_coefficients);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_kV", _kV_coefficients);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_kI", _kI_coefficients);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_radius", _actuators_r);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_tconst", _actuators_tconst);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_maxsat", _actuators_maxsat);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_minsat", _actuators_minsat);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_inversion", _actuators_dir_inversion);
    _nh.getParam(_vehicle_name + "/dynamics" + "/actuators_pwm", _actuators_pwm_range);
    _nh.getParam(_vehicle_name + "/battery" + "/vcut", _bat_c_vcut);
    _nh.getParam(_vehicle_name + "/battery" + "/vmax", _bat_c_vmax);
    _nh.getParam(_vehicle_name + "/battery" + "/ESR", _bat_esr);
    _nh.getParam(_vehicle_name + "/battery" + "/discharge_K", _kBd_coefficients);
    _nh.getParam(_vehicle_name + "/battery" + "/cell_n", _bat_cell_n);
    _nh.getParam(_vehicle_name + "/battery" + "/capacity_mAh", _bat_mAh);
    _nh.getParam(_vehicle_name + "/dynamics" + "/initial_pose", _p_initial);
    _nh.getParam(_vehicle_name + "/dynamics" + "/initial_v", _v_initial);

    _auv_dyn = new AUVDynamics();

    set_parameters();

    /* And we start node*/

    start();
}

void Dynamics_Node::reset() {
    /* Initial State */
    Dynamics::States state_;
    state_.pos <<_p_initial[0], _p_initial[1], _p_initial[2];
    state_.att.orientation << _p_initial[3], _p_initial[4], _p_initial[5];
    state_.att.orientation_q = Dynamics::euler2Quaterniond(state_.att.orientation);
    state_.vel << _v_initial[0], _v_initial[1], _v_initial[2], _v_initial[3], _v_initial[4], _v_initial[5];
    state_.power.bat_vnom = _bat_cell_n * _bat_c_vmax;
    _auv_dyn->setInitialState(state_);
}

void Dynamics_Node::set_parameters(void){

    Dynamics::Constants constants_;

    constants_.inertia.mass = _mass;
    constants_.inertia.tensor << _tensor[0], _tensor[1], _tensor[2],
                                _tensor[3], _tensor[4], _tensor[5],
                                _tensor[6], _tensor[7], _tensor[8];
    constants_.inertia.g = _g;
    constants_.inertia.radius <<_radius[0], _radius[1], _radius[2];
    constants_.inertia.rB << _rB[0], _rB[1], _rB[2];
    constants_.inertia.rG << _rG[0], _rG[1], _rG[2];

    constants_.damping.damping << _damping[0], _damping[1], _damping[2], _damping[3], _damping[4], _damping[5];
    constants_.damping.q_damping << _quadratic_damping[0], _quadratic_damping[1], _quadratic_damping[2],
            _quadratic_damping[3], _quadratic_damping[4], _quadratic_damping[5];
    constants_.damping.dh = _dh;
    constants_.damping.dv = _dv;
    constants_.damping.dzv = _dzv;

    constants_.actuator.L = _actuators_r;
    constants_.actuator.kf << _kF_coefficients[0], _kF_coefficients[1], _kF_coefficients[2];
    constants_.actuator.km << _kM_coefficients[0], _kM_coefficients[1], _kM_coefficients[2];
    constants_.actuator.kv << _kV_coefficients[0], _kV_coefficients[1], _kV_coefficients[2];
    constants_.actuator.count  = _num_actuators;
    constants_.actuator.maxsat = _actuators_maxsat;
    constants_.actuator.minsat = _actuators_minsat;
    constants_.actuator.tconst = _actuators_tconst;
    constants_.actuator.pwm_range << _actuators_pwm_range[0], _actuators_pwm_range[1], _actuators_pwm_range[2];
    constants_.actuator.dir_inversion << _actuators_dir_inversion[0], _actuators_dir_inversion[1],
            _actuators_dir_inversion[2], _actuators_dir_inversion[3];

    constants_.battery.c_vcut = _bat_c_vcut;
    constants_.battery.cell_n = _bat_cell_n;
    constants_.battery.c_vmax = _bat_c_vmax;
    constants_.battery.esr = _bat_esr;
    constants_.battery.mAh = _bat_mAh;
    constants_.battery.ki  << _kI_coefficients[0], _kI_coefficients[1], _kI_coefficients[2];
    constants_.battery.kBd << _kBd_coefficients[0], _kBd_coefficients[1], _kBd_coefficients[2];

    constants_.environment.density = _density;

    _auv_dyn->setConstants(constants_);

    reset();
}


void Dynamics_Node::thrusters_input_callback(const std_msgs::Float64MultiArray::ConstPtr &thruster_msg) {

    static Dynamics::Inputs input_;
    for (int k = 0; k < _num_actuators; k++){
        input_.w_pwm(k) = thruster_msg->data[k];
    }
    _auv_dyn->setInput(input_);
}


void Dynamics_Node::pub_vehicle_state(void) {

    _v_full_state_ros_pub.publish(_state24_msg);
    _v_power_ros_pub.publish(_power_msg);
}

void Dynamics_Node::pub_uwsim_pose_state(void) {
    _v_uwsim_pose_ros_pub.publish(_uwsim_pose_sp_msg);
}

void Dynamics_Node::start(void) {

    ros::Rate r_(1 / _diffq_period);
    ros::Time begin_ = ros::Time::now();
    double time_now_ = (ros::Time::now() - begin_).toSec();
    double last_uwsim_pub_sec_ = time_now_;
    double last_dyn_sec_ = time_now_;

    while(ros::ok()){
        /* Send messages at requested sample rates */
        time_now_ = (ros::Time::now() - begin_).toSec();

        /* Iterate Dynamics - at high rate */
        double dt_ = time_now_ - last_dyn_sec_;
        last_dyn_sec_ = time_now_;
        _auv_dyn->iterate(dt_);

        /* Publish msgs */
        double state_pub_dt_sec_ = time_now_ - last_uwsim_pub_sec_;
        if (state_pub_dt_sec_ != 0 && state_pub_dt_sec_ > _uwsim_period){

            Dynamics::States state_ = _auv_dyn->getStates();

            for(int k = 0; k < 6; k++){
                _state24_msg.v[k] = state_.vel(k);
                _state24_msg.v_dot[k] = state_.vel_dot(k);
            }

            for(int k = 0; k < 3; k++){
                _state24_msg.p_q[k] = state_.pos(k);
                _state24_msg.p_lin_dot[k] = state_.pos_dot(k);
            }

            _uwsim_pose_sp_msg.position.x = state_.pos(0);
            _uwsim_pose_sp_msg.position.y = state_.pos(1);
            _uwsim_pose_sp_msg.position.z = state_.pos(2);
            _state24_msg.p_q[3] = _uwsim_pose_sp_msg.orientation.w = state_.att.orientation_q.w();
            _state24_msg.p_q[4] = _uwsim_pose_sp_msg.orientation.x = state_.att.orientation_q.x();
            _state24_msg.p_q[5] = _uwsim_pose_sp_msg.orientation.y = state_.att.orientation_q.y();
            _state24_msg.p_q[6] = _uwsim_pose_sp_msg.orientation.z = state_.att.orientation_q.z();

            _power_msg.V   = state_.power.bat_vnom;
            _power_msg.I   = state_.power.bat_I;
            _power_msg.percent_remaining = state_.power.bat_percent_remaining;

            pub_vehicle_state();
            pub_uwsim_pose_state();

            last_uwsim_pub_sec_ = time_now_;
        }
        ros::spinOnce();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwsim_dynamics");
    ros::NodeHandle privateNh("~");

    /* Node runs exclusively within dn */
    Dynamics_Node dn_;

    return 0;
}