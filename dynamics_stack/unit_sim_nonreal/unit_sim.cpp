//
// Created by alsaibie on 5/30/18.
//

#include "unit_sim.hpp"
#include <cstdlib>
#include <iostream>
#include <mathlib/math/Limits.hpp>

using namespace std;
using namespace uwsim;

unit_sim::unit_sim():
        _shared_lp_filters_d{{initial_update_rate_hz, 50.f},
                             {initial_update_rate_hz, 50.f},
                             {initial_update_rate_hz, 50.f}} // will be initialized correctly when params are loaded
{

    if(char const* tmp = std::getenv("HOME")){
        string home_p(tmp);
        _config_path = home_p + "/UWSim/src/underwater_simulation/dynamics_stack/unit_sim_nonreal/Install/config/";
        _data_path = home_p + "/UWSim/src/underwater_simulation/dynamics_stack/unit_sim_nonreal/Install/data/";
//        cout << "config path: " << _config_path << endl;
    }
    else {
        throw std::exception();
    }

    /* Instantiate and Initialize AUV Dynamics Object */
    _auv_dyn = new Dynamics::AUVDynamics();
    get_set_dynamics_parameters();
    _auv_dyn->initialize();

    /* Sensor Object */
    _sensor_dyn = new Sensor::SensorDynamics();
    get_set_sensor_parameters();
    _sensor_dyn->Initialize();

    /* Pos Control Object */
    _pos_control = new PController::PositionController();
    get_set_pos_controller_parameters();

    /* Att Control Object */
    _att_control = new AController::AttitudeController();
    get_set_att_controller_parameters();

    reset();

}

void unit_sim::reset() {

    /* Initial Dynamic States */
    Dynamics::States state_;
    state_.pos <<_p_initial.second[0], _p_initial.second[1], _p_initial.second[2];
    state_.att.orientation << _p_initial.second[3], _p_initial.second[4], _p_initial.second[5];
    state_.att.orientation_q = Dynamics_Math::euler2Quaterniond(state_.att.orientation);
    state_.vel << _v_initial.second[0], _v_initial.second[1], _v_initial.second[2],
            _v_initial.second[3], _v_initial.second[4], _v_initial.second[5];
    state_.power.bat_vnom = _bat_cell_n.second * _bat_c_vmax.second;
    _auv_dyn->setInitialState(state_);


    /* Initial Position Controller State */
    _pos_control->resetSetpoints();
    _pos_control->resetReferenceState();

    /* Initial Attitude Controller State */
    _att_control->resetSetpoints();

}


void unit_sim::get_set_dynamics_parameters(void) {

    YAML::Node yn_dynamics = YAML::LoadFile( _config_path + "dynamics_dolphin.yaml");
    uwsim::get_param<double>             (yn_dynamics, _diffq_period );
    uwsim::get_param<double>             (yn_dynamics, _pub_period );

    uwsim::get_param<double>         (yn_dynamics, _mass );
    uwsim::get_param<vector<double>> (yn_dynamics, _rG );
    uwsim::get_param<vector<double>> (yn_dynamics, _rB );
    uwsim::get_param<double>         (yn_dynamics, _g );
    uwsim::get_param<vector<double>> (yn_dynamics, _radius );
    uwsim::get_param<vector<double>> (yn_dynamics, _tensor );

    uwsim::get_param<vector<double>> (yn_dynamics, _ksurge_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _ksway_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _kheave_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _kroll_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _kpitch_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _kyaw_coefficients );

    uwsim::get_param<double> (yn_dynamics, _density );

    uwsim::get_param<int>            (yn_dynamics, _num_actuators );
    uwsim::get_param<vector<double>> (yn_dynamics, _kF_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _kM_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _kV_coefficients );
    uwsim::get_param<vector<double>> (yn_dynamics, _kI_coefficients );
    uwsim::get_param<double>         (yn_dynamics, _actuators_r );
    uwsim::get_param<double>         (yn_dynamics, _actuators_tconst );
    uwsim::get_param<double>         (yn_dynamics, _actuators_maxsat );
    uwsim::get_param<double>         (yn_dynamics, _actuators_minsat );
    uwsim::get_param<vector<int>>    (yn_dynamics, _actuators_dir_inversion );
    uwsim::get_param<vector<int>>    (yn_dynamics, _actuators_pwm_range );
    uwsim::get_param<double>         (yn_dynamics, _actuators_impeller_d );
    uwsim::get_param<double>         (yn_dynamics, _actuators_NKT_Coefficient );

    uwsim::get_param<vector<double>> (yn_dynamics, _kBd_coefficients );
    uwsim::get_param<double>         (yn_dynamics, _bat_c_vmax );
    uwsim::get_param<double>         (yn_dynamics, _bat_c_vcut );
    uwsim::get_param<double>         (yn_dynamics, _bat_esr );
    uwsim::get_param<double>         (yn_dynamics, _bat_cell_n );
    uwsim::get_param<int>            (yn_dynamics, _bat_mAh );

    uwsim::get_param<vector<double>> (yn_dynamics, _p_initial );
    uwsim::get_param<vector<double>> (yn_dynamics, _v_initial );

    /** Set Parameters */

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
    constants_.actuator.impeller_D = _actuators_impeller_d.second;
    constants_.actuator.NKT_Coefficient = _actuators_NKT_Coefficient.second;

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




}


void unit_sim::get_set_sensor_parameters(void){

    YAML::Node yn_sensor = YAML::LoadFile( _config_path + "uwsim_sensor.yaml");

    uwsim::get_param<double>             (yn_sensor, _hil_sensor_period_sec );
    uwsim::get_param<double>             (yn_sensor, _hil_quaternion_period_sec );
    uwsim::get_param<double>             (yn_sensor, _hil_battery_period_sec );

    uwsim::get_param<float>             (yn_sensor, _sen_accelerometer_std );
    uwsim::get_param<float>             (yn_sensor, _sen_acc_noise_density );
    uwsim::get_param<float>             (yn_sensor, _sen_acc_bias_diffusion );
    uwsim::get_param<float>             (yn_sensor, _sen_gyro_std );
    uwsim::get_param<float>             (yn_sensor, _sen_gyro_noise_density );
    uwsim::get_param<float>             (yn_sensor, _sen_gyro_bias_diffusion );
    uwsim::get_param<float>             (yn_sensor, _sen_mag_std );
    uwsim::get_param<float>             (yn_sensor, _sen_mag_inclination );
    uwsim::get_param<float>             (yn_sensor, _sen_mag_declination );
    uwsim::get_param<float>             (yn_sensor, _sen_pressure_ref );
    uwsim::get_param<float>             (yn_sensor, _sen_pressure_std );
    uwsim::get_param<float>             (yn_sensor, _sen_temp_ref );
    uwsim::get_param<float>             (yn_sensor, _sen_temp_std );

    uwsim::get_param<float>             (yn_sensor, _att_quaternion_std );
    uwsim::get_param<float>             (yn_sensor, _att_omega_std );
    uwsim::get_param<float>             (yn_sensor, _att_acceleration_std );

    /** Set Parameters */
    Sensor::Parameters param_;

    param_.acc.std = _sen_accelerometer_std.second;
    param_.acc.noise_density = _sen_acc_noise_density.second;
    param_.acc.bias_diffusion = _sen_acc_bias_diffusion.second;
    param_.gyro.std = _sen_gyro_std.second;
    param_.gyro.noise_density = _sen_gyro_noise_density.second;
    param_.gyro.bias_diffusion = _sen_gyro_bias_diffusion.second;
    param_.mag.std = _sen_mag_std.second;
    param_.mag.inclination = _sen_mag_inclination.second;
    param_.mag.declination = _sen_mag_declination.second;
    param_.pressure.std = _sen_pressure_std.second;
    param_.pressure.ref = _sen_pressure_ref.second;
    param_.temp.std = _sen_temp_std.second;
    param_.temp.ref = _sen_temp_ref.second;

    _sensor_dyn->setParameters(param_);

}



void unit_sim::get_set_pos_controller_parameters(void){

    YAML::Node yn_pcontroller = YAML::LoadFile( _config_path + "position_controller.yaml");

    uwsim::get_param<float>  (yn_pcontroller,_thrust_idle);
    uwsim::get_param<int>    (yn_pcontroller,_speed_ctrl_mode);
    uwsim::get_param<float>  (yn_pcontroller,_max_tilt_angle);
    uwsim::get_param<float>  (yn_pcontroller,_max_roll_angle);

    /* Set Parameters */
    PController::Limits limits;
    PController::Gains gains;

    //TODO: Define parameters

    limits.max_att_angle(0) = math::radians(_max_roll_angle.second);
    limits.max_att_angle(1) = math::radians(_max_tilt_angle.second);
    limits.max_att_angle(2) = math::radians(_max_tilt_angle.second);

    _pos_control->updateGains(gains);
    _pos_control->updateLimits(limits);
}

void unit_sim::get_set_att_controller_parameters(void){

    YAML::Node yn_acontroller = YAML::LoadFile( _config_path + "attitude_controller.yaml");

    uwsim::get_param<float>  (yn_acontroller,_roll_p);
    uwsim::get_param<float>  (yn_acontroller,_roll_rate_p);
    uwsim::get_param<float>  (yn_acontroller,_roll_rate_i);
    uwsim::get_param<float>  (yn_acontroller,_roll_rate_integ_lim);
    uwsim::get_param<float>  (yn_acontroller,_roll_rate_d);
    uwsim::get_param<float>  (yn_acontroller,_roll_rate_ff);
    uwsim::get_param<float>  (yn_acontroller,_roll_ff);
    uwsim::get_param<float>  (yn_acontroller,_roll_max_tau_sp);

    uwsim::get_param<float>  (yn_acontroller,_pitch_p);
    uwsim::get_param<float>  (yn_acontroller,_pitch_rate_p);
    uwsim::get_param<float>  (yn_acontroller,_pitch_rate_i);
    uwsim::get_param<float>  (yn_acontroller,_pitch_rate_integ_lim);
    uwsim::get_param<float>  (yn_acontroller,_pitch_rate_d);
    uwsim::get_param<float>  (yn_acontroller,_pitch_rate_ff);
    uwsim::get_param<float>  (yn_acontroller,_pitch_max_tau_sp);

    uwsim::get_param<float>  (yn_acontroller,_yaw_p);
    uwsim::get_param<float>  (yn_acontroller,_yaw_rate_p);
    uwsim::get_param<float>  (yn_acontroller,_yaw_rate_i);
    uwsim::get_param<float>  (yn_acontroller,_yaw_rate_integ_lim);
    uwsim::get_param<float>  (yn_acontroller,_yaw_rate_d);
    uwsim::get_param<float>  (yn_acontroller,_yaw_rate_ff);
    uwsim::get_param<float>  (yn_acontroller,_yaw_max_tau_sp);


    uwsim::get_param<float>  (yn_acontroller,_d_term_cutoff_freq);

    uwsim::get_param<float>  (yn_acontroller,_tpa_breakpoint_p);
    uwsim::get_param<float>  (yn_acontroller,_tpa_breakpoint_i);
    uwsim::get_param<float>  (yn_acontroller,_tpa_breakpoint_d);
    uwsim::get_param<float>  (yn_acontroller,_tpa_rate_p);
    uwsim::get_param<float>  (yn_acontroller,_tpa_rate_i);
    uwsim::get_param<float>  (yn_acontroller,_tpa_rate_d);

    uwsim::get_param<float>  (yn_acontroller,_roll_rate_max);
    uwsim::get_param<float>  (yn_acontroller,_pitch_rate_max);
    uwsim::get_param<float>  (yn_acontroller,_yaw_rate_max);
    uwsim::get_param<float>  (yn_acontroller,_roll_auto_max);
    uwsim::get_param<float>  (yn_acontroller,_acro_roll_max);
    uwsim::get_param<float>  (yn_acontroller,_acro_pitch_max);
    uwsim::get_param<float>  (yn_acontroller,_acro_yaw_max);
    uwsim::get_param<float>  (yn_acontroller,_acro_expo_py);
    uwsim::get_param<float>  (yn_acontroller,_acro_expo_r);
    uwsim::get_param<float>  (yn_acontroller,_acro_superexpo_py);
    uwsim::get_param<float>  (yn_acontroller,_acro_superexpo_r);

    uwsim::get_param<float>  (yn_acontroller,_rattitude_thres);
    uwsim::get_param<int>    (yn_acontroller,_bat_scale_en);


    uwsim::get_param<float>  (yn_acontroller,_c0_nThrust);
    uwsim::get_param<float>  (yn_acontroller,_c1_nThrust);
    uwsim::get_param<float>  (yn_acontroller,_c2_nThrust);
    uwsim::get_param<float>  (yn_acontroller,_c2_nTorque);
    uwsim::get_param<float>  (yn_acontroller,_c0_Vn);
    uwsim::get_param<float>  (yn_acontroller,_c1_Vn);
    uwsim::get_param<float>  (yn_acontroller,_c2_Vn);
    uwsim::get_param<float>  (yn_acontroller,_c0_Vn_norm);
    uwsim::get_param<float>  (yn_acontroller,_c1_Vn_norm);
    uwsim::get_param<float>  (yn_acontroller,_c2_Vn_norm);
//    uwsim::get_param<float>  (yn_acontroller,_rotor_radius);

    /* Set Parameters */

    AController::Limits limits;
    AController::Gains gains;
    AController::Constants constants;

    /* roll gains */
    gains.att_p(0) = _roll_p.second;
    gains.rate_p(0) = _roll_rate_p.second;
    gains.rate_i(0) = _roll_rate_i.second;
    limits.rate_int_lim(0) = _roll_rate_integ_lim.second;
    gains.rate_d(0) = _roll_rate_d.second;
    gains.rate_ff(0) = _roll_rate_ff.second;
    gains.att_ff(0) = _roll_ff.second;

    /* pitch gains */
    gains.att_p(1) = _pitch_p.second;
    gains.rate_p(1) = _pitch_rate_p.second;
    gains.rate_i(1) = _pitch_rate_i.second;
    limits.rate_int_lim(1) = _pitch_rate_integ_lim.second;
    gains.rate_d(1) = _pitch_rate_d.second;
    gains.rate_ff(1) = _pitch_rate_ff.second;

    /* yaw gains */
    gains.att_p(2) = _yaw_p.second;
    gains.rate_p(2) = _yaw_rate_p.second;
    gains.rate_i(2) = limits.rate_int_lim(2) = _yaw_rate_integ_lim.second;
    gains.rate_d(2) = _yaw_rate_d.second;
    gains.rate_ff(2) = _yaw_rate_ff.second;

    /* low pass filter*/
    if (fabsf(_shared_lp_filters_d[0].get_cutoff_freq() - _d_term_cutoff_freq.second) > 0.01f) {
        _shared_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.second);
        _shared_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.second);
        _shared_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.second);
    }

    /* angular rate limits */

    limits.manual_rate_max(0) = math::radians(_roll_rate_max.second);
    limits.manual_rate_max(1) = math::radians(_pitch_rate_max.second);
    limits.manual_rate_max(2) = math::radians(_yaw_rate_max.second);

    /* auto angular rate limits */
    limits.auto_rate_max(0) = math::radians(_roll_auto_max.second);
    limits.auto_rate_max(1) = math::radians(_pitch_rate_max.second);
    limits.auto_rate_max(2) = math::radians(_yaw_rate_max.second);

    /* manual rate control acro mode rate limits and expo */
    limits.acro_rate_max(0) = math::radians(_acro_roll_max.second);
    limits.acro_rate_max(1) = math::radians(_acro_pitch_max.second);
    limits.acro_rate_max(2) = math::radians(_acro_yaw_max.second);

    /* Expo Rates */
    limits.acro_expo_py = math::radians(_acro_expo_py.second);
    limits.acro_expo_r = math::radians(_acro_expo_r.second);
    limits.acro_superexpo_py = math::radians(_acro_superexpo_py.second);
    limits.acro_superexpo_r = math::radians(_acro_superexpo_r.second);

    limits.tau_max_sp(0) = _roll_max_tau_sp.second;
    limits.tau_max_sp(1) = _pitch_max_tau_sp.second;
    limits.tau_max_sp(2) = _yaw_max_tau_sp.second;

    constants.nThrust_coefficients(0) = _c0_nThrust.second;
    constants.nThrust_coefficients(1) = _c1_nThrust.second;
    constants.nThrust_coefficients(2) = _c2_nThrust.second;

    constants.nTorque_coefficients(0) = 0;
    constants.nTorque_coefficients(1) = 0;
    constants.nTorque_coefficients(2) = _c2_nTorque.second;

    constants.Vn_coefficients(0) = _c0_Vn.second;
    constants.Vn_coefficients(1) = _c1_Vn.second;
    constants.Vn_coefficients(2) = _c2_Vn.second;

    constants.Vn_norm_coefficients(0) = _c0_Vn_norm.second;
    constants.Vn_norm_coefficients(1) = _c1_Vn_norm.second;
    constants.Vn_norm_coefficients(2) = _c2_Vn_norm.second;

//    constants.rotor_radius = _rotor_radius.second;
    constants.rotor_radius = 0.03;

    _att_control->updateGains(gains);
    _att_control->updateLimits(limits);
    _att_control->updateConstants(constants);

    _att_control->setFilter(_shared_lp_filters_d);

}