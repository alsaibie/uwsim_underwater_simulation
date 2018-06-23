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
        _state_control_sub( _nh.subscribe("/dolphin/dynamics/state_control", 1, &Dynamics_Node::state_control_callback, this)),
        _v_uwsim_pose_ros_pub( _nh.advertise<geometry_msgs::Pose>("/dolphin/uwsim/pose_sp", 1)),
        _v_full_state_ros_pub( _nh.advertise<uwsim_msgs::full_state>("/dolphin/dynamics/full_state", 1)),
        _v_power_ros_pub(_nh.advertise<uwsim_msgs::power>("/dolphin/dynamics/power", 1)),
        _v_dyn_param_ros_pub(_nh.advertise<uwsim_msgs::dynamics_param>("/dolphin/dynamics/parameters", 1))
{

    /* Get Parameters */
    _vehicle_name = "dolphin";

    uwsim::get_param<double>             (_nh, _diffq_period );
    uwsim::get_param<double>             (_nh, _pub_period );

    uwsim::get_param<double>         (_nh, _mass );
    uwsim::get_param<vector<double>> (_nh, _rG );
    uwsim::get_param<vector<double>> (_nh, _rB );
    uwsim::get_param<double>         (_nh, _g );
    uwsim::get_param<vector<double>> (_nh, _radius );
    uwsim::get_param<vector<double>> (_nh, _tensor );

    uwsim::get_param<vector<double>> (_nh, _ksurge_coefficients );
    uwsim::get_param<vector<double>> (_nh, _ksway_coefficients );
    uwsim::get_param<vector<double>> (_nh, _kheave_coefficients );
    uwsim::get_param<vector<double>> (_nh, _kroll_coefficients );
    uwsim::get_param<vector<double>> (_nh, _kpitch_coefficients );
    uwsim::get_param<vector<double>> (_nh, _kyaw_coefficients );

    uwsim::get_param<double> (_nh, _density );

    uwsim::get_param<int>            (_nh, _num_actuators );
    uwsim::get_param<vector<double>> (_nh, _kF_coefficients );
    uwsim::get_param<vector<double>> (_nh, _kM_coefficients );
    uwsim::get_param<vector<double>> (_nh, _kV_coefficients );
    uwsim::get_param<vector<double>> (_nh, _kI_coefficients );
    uwsim::get_param<double>         (_nh, _actuators_r );
    uwsim::get_param<double>         (_nh, _actuators_tconst );
    uwsim::get_param<double>         (_nh, _actuators_maxsat );
    uwsim::get_param<double>         (_nh, _actuators_minsat );
    uwsim::get_param<vector<int>>    (_nh, _actuators_dir_inversion );
    uwsim::get_param<vector<int>>    (_nh, _actuators_pwm_range );

    uwsim::get_param<vector<double>> (_nh, _kBd_coefficients );
    uwsim::get_param<double>         (_nh, _bat_c_vmax );
    uwsim::get_param<double>         (_nh, _bat_c_vcut );
    uwsim::get_param<double>         (_nh, _bat_esr );
    uwsim::get_param<double>         (_nh, _bat_cell_n );
    uwsim::get_param<int>            (_nh, _bat_mAh );

    uwsim::get_param<vector<double>> (_nh, _p_initial );
    uwsim::get_param<vector<double>> (_nh, _v_initial );

    cout << "Test:" << _p_initial.second[0] <<endl;
    _auv_dyn = new Dynamics::AUVDynamics();
    set_parameters();
    _auv_dyn->initialize();

    /* And we start node*/
    start();
}


void Dynamics_Node::set_parameters(void){

    Dynamics::Constants constants_;

    constants_.inertia.mass = _mass.second;
    constants_.inertia.tensor << _tensor.second[0], _tensor.second[1], _tensor.second[2],
            _tensor.second[3], _tensor.second[4], _tensor.second[5],
            _tensor.second[6], _tensor.second[7], _tensor.second[8];
    constants_.inertia.g = _g.second;
    constants_.inertia.radius <<_radius.second[0], _radius.second[1], _radius.second[2];
    constants_.inertia.rB << _rB.second[0], _rB.second[1], _rB.second[2];
    constants_.inertia.rG << _rG.second[0], _rG.second[1], _rG.second[2];

    constants_.damping.ksurge << _ksurge_coefficients.second[0], _ksurge_coefficients.second[1],
            _ksurge_coefficients.second[2];
    constants_.damping.ksway << _ksway_coefficients.second[0], _ksway_coefficients.second[1],
            _ksway_coefficients.second[2];
    constants_.damping.kheave << _kheave_coefficients.second[0], _kheave_coefficients.second[1],
            _kheave_coefficients.second[2];
    constants_.damping.kroll << _kroll_coefficients.second[0], _kroll_coefficients.second[1],
            _kroll_coefficients.second[2];
    constants_.damping.kpitch << _kpitch_coefficients.second[0], _kpitch_coefficients.second[1],
            _kpitch_coefficients.second[2];
    constants_.damping.kyaw << _kyaw_coefficients.second[0], _kyaw_coefficients.second[1],
            _kyaw_coefficients.second[2];

    constants_.actuator.L = _actuators_r.second;
    constants_.actuator.kf << _kF_coefficients.second[0], _kF_coefficients.second[1], _kF_coefficients.second[2];
    constants_.actuator.km << _kM_coefficients.second[0], _kM_coefficients.second[1], _kM_coefficients.second[2];
    constants_.actuator.kv << _kV_coefficients.second[0], _kV_coefficients.second[1], _kV_coefficients.second[2];
    constants_.actuator.count  = _num_actuators.second;
    constants_.actuator.maxsat = _actuators_maxsat.second;
    constants_.actuator.minsat = _actuators_minsat.second;
    constants_.actuator.tconst = _actuators_tconst.second;
    constants_.actuator.pwm_range << _actuators_pwm_range.second[0], _actuators_pwm_range.second[1], _actuators_pwm_range.second[2];
    constants_.actuator.dir_inversion << _actuators_dir_inversion.second[0], _actuators_dir_inversion.second[1],
            _actuators_dir_inversion.second[2], _actuators_dir_inversion.second[3];

    constants_.battery.c_vcut = _bat_c_vcut.second;
    constants_.battery.cell_n = _bat_cell_n.second;
    constants_.battery.c_vmax = _bat_c_vmax.second;
    constants_.battery.esr = _bat_esr.second;
    constants_.battery.mAh = _bat_mAh.second;
    constants_.battery.ki  << _kI_coefficients.second[0], _kI_coefficients.second[1], _kI_coefficients.second[2], _kI_coefficients.second[3];
    constants_.battery.kBd << _kBd_coefficients.second[0], _kBd_coefficients.second[1], _kBd_coefficients.second[2],
            _kBd_coefficients.second[3], _kBd_coefficients.second[4], _kBd_coefficients.second[5];

    constants_.environment.density = _density.second;

    _auv_dyn->setConstants(constants_);

    reset();


}

void Dynamics_Node::reset(){
    /* Initial Dynamic States */
    Dynamics::States state_ = _auv_dyn->getStates();
    state_.pos <<_p_initial.second[0], _p_initial.second[1], _p_initial.second[2];
    state_.att.orientation << _p_initial.second[3], _p_initial.second[4], _p_initial.second[5];
    state_.att.orientation_q = Dynamics_Math::euler2Quaterniond(state_.att.orientation);
    state_.vel << _v_initial.second[0], _v_initial.second[1], _v_initial.second[2],
            _v_initial.second[3], _v_initial.second[4], _v_initial.second[5];
    state_.power.bat_vnom = _bat_cell_n.second * _bat_c_vmax.second;
    _auv_dyn->setInitialState(state_);
}

void Dynamics_Node::thrusters_input_callback(const std_msgs::Float64MultiArray::ConstPtr &thruster_msg) {

    static Dynamics::Inputs input_;
    for (int k = 0; k < _num_actuators.second; k++){
        input_.w_pwm(k) = thruster_msg->data[k];
    }
    _auv_dyn->setInput(input_);
}

void Dynamics_Node::state_control_callback(const uwsim_msgs::state_control::ConstPtr &control_msg){
    if(control_msg->reset_dynamics){
        /* reset dynamic simulation */
        ROS_INFO("Resetting Dynamic Simulator");
        reset();
    }
}


void Dynamics_Node::pub_vehicle_state(void) {

    _v_full_state_ros_pub.publish(_state24_msg);
    _v_power_ros_pub.publish(_power_msg);
}

void Dynamics_Node::pub_uwsim_pose_state(void) {
    _v_uwsim_pose_ros_pub.publish(_uwsim_pose_sp_msg);
}

void Dynamics_Node::pub_uwsim_dyn_param(void) {

    Dynamics::DynamicParameters params_ = _auv_dyn->getDynamicParameters();
    for (int k=0; k<6; k++){
        _uwsim_dyn_param_msg.coriolis_vector[k] = params_.Cvv(k);
        _uwsim_dyn_param_msg.damping_vector[k] = params_.Dvv(k);
        _uwsim_dyn_param_msg.tau[k] = params_.tau(k);
        _uwsim_dyn_param_msg.gravity_vector[k] = params_.gRB(k);
    }

    _v_dyn_param_ros_pub.publish(_uwsim_dyn_param_msg);

}

void Dynamics_Node::start(void) {

    ros::Rate r_(1 / _diffq_period.second);
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
        if (state_pub_dt_sec_ != 0 && state_pub_dt_sec_ > _pub_period.second){

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
            _power_msg.I   = state_.power.bat_I_mA;
            _power_msg.percent_remaining = state_.power.bat_percent_remaining;

            pub_vehicle_state();
            pub_uwsim_pose_state();

            pub_uwsim_dyn_param();
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