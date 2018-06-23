//
// Created by alsaibie on 5/30/18.
//

#include "unit_sim.hpp"
#include <time.h>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <iostream>
#include <fstream>
using namespace std;



unit_sim::unit_sim() {

    if(char const* tmp = std::getenv("HOME")){
        string home_p(tmp);
        _config_path = home_p + "/UWSim/src/underwater_simulation/dynamics_stack/unit_sim/Install/config/";
        _data_path = home_p + "/UWSim/src/underwater_simulation/dynamics_stack/unit_sim/Install/data/";
//        cout << "config path: " << _config_path << endl;
    }
    else {
        throw std::exception();
    }

    /* Instantiate and Initialize AUV Dynamics Object */
    _auv_dyn = new AUVDynamics();
    get_set_dynamics_parameters();
    _auv_dyn->initialize();

    /* Sensor Object */


    /* Att Control Object */


    /* Pos Control Object */


}

void unit_sim::reset() {

    /* Initial State */
    Dynamics::States state_;
    state_.pos <<_p_initial.second[0], _p_initial.second[1], _p_initial.second[2];
    state_.att.orientation << _p_initial.second[3], _p_initial.second[4], _p_initial.second[5];
    state_.att.orientation_q = Dynamics_Math::euler2Quaterniond(state_.att.orientation);
    state_.vel << _v_initial.second[0], _v_initial.second[1], _v_initial.second[2],
            _v_initial.second[3], _v_initial.second[4], _v_initial.second[5];
    state_.power.bat_vnom = _bat_cell_n.second * _bat_c_vmax.second;
    _auv_dyn->setInitialState(state_);

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

    uwsim::get_param<vector<double>> (yn_dynamics, _damping );
    uwsim::get_param<vector<double>> (yn_dynamics, _quadratic_damping );

    uwsim::get_param<double> (yn_dynamics, _dzv );
    uwsim::get_param<double> (yn_dynamics, _dv );
    uwsim::get_param<double> (yn_dynamics, _dh );
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

    uwsim::get_param<vector<double>> (yn_dynamics, _kBd_coefficients );
    uwsim::get_param<double>         (yn_dynamics, _bat_c_vmax );
    uwsim::get_param<double>         (yn_dynamics, _bat_c_vcut );
    uwsim::get_param<double>         (yn_dynamics, _bat_esr );
    uwsim::get_param<double>         (yn_dynamics, _bat_cell_n );
    uwsim::get_param<int>            (yn_dynamics, _bat_mAh );

    uwsim::get_param<vector<double>> (yn_dynamics, _p_initial );
    uwsim::get_param<vector<double>> (yn_dynamics, _v_initial );

    /** Set Paramters */

    Dynamics::Constants constants_;

    constants_.inertia.mass = _mass.second;
    constants_.inertia.tensor << _tensor.second[0], _tensor.second[1], _tensor.second[2],
            _tensor.second[3], _tensor.second[4], _tensor.second[5],
            _tensor.second[6], _tensor.second[7], _tensor.second[8];
    constants_.inertia.g = _g.second;
    constants_.inertia.radius <<_radius.second[0], _radius.second[1], _radius.second[2];
    constants_.inertia.rB << _rB.second[0], _rB.second[1], _rB.second[2];
    constants_.inertia.rG << _rG.second[0], _rG.second[1], _rG.second[2];

    constants_.damping.damping << _damping.second[0], _damping.second[1], _damping.second[2],
            _damping.second[3], _damping.second[4], _damping.second[5];
    constants_.damping.q_damping << _quadratic_damping.second[0], _quadratic_damping.second[1],
            _quadratic_damping.second[2], _quadratic_damping.second[3],
            _quadratic_damping.second[4], _quadratic_damping.second[5];
    constants_.damping.dh = _dh.second;
    constants_.damping.dv = _dv.second;
    constants_.damping.dzv = _dzv.second;

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
    constants_.battery.ki  << _kI_coefficients.second[0], _kI_coefficients.second[1], _kI_coefficients.second[2];
    constants_.battery.kBd << _kBd_coefficients.second[0], _kBd_coefficients.second[1], _kBd_coefficients.second[2];

    constants_.environment.density = _density.second;

    _auv_dyn->setConstants(constants_);

    reset();

}

inline double unity_to_pwm(const double &v){ return v * 400.0 + 1500.0; }

void unit_sim::start() {
    auto auv_dyn_period_msec_  = _diffq_period.second * 1000;
    auto sensor_period_msec_   = _pub_period.second * 1000;
    auto att_control_period_msec_ = _pub_period.second * 1000;
    auto pos_control_period_msec_ = _pub_period.second * 1000;
    auto command_period_msec_ = 60.0;

    auto start = chrono::high_resolution_clock::now();

    auto last_auv_dyn_time_msec_ = std::chrono::high_resolution_clock::now();
    auto last_sensor_time_msec_ = std::chrono::high_resolution_clock::now();
    auto last_att_control_time_msec_ = std::chrono::high_resolution_clock::now();
    auto last_pos_control_time_msec_ = std::chrono::high_resolution_clock::now();
    auto last_command_time_msec_ = std::chrono::high_resolution_clock::now();

    auto current_time = chrono::high_resolution_clock::now();
    double auv_dyn_dt_msec_;
    double sensor_dt_msec_;
    double att_control_dt_msec_;
    double pos_control_dt_msec_;
    double command_dt_msec_;

    /* Initialize Input */
    Dynamics::Inputs inputs_;
    inputs_.w_pwm =  VectorXd::Constant(4, 1500);
    _auv_dyn->setInput(inputs_);

    /* Test files*/
    FILE *fp_in, *fp_out_dynamics, *fp_out_sensor, *fp_out_controller;
    float ch1_ = 0.0, ch2_ = 0.0, ch3_ = 0.0, ch4_ = 0.0;
    int ret;

    fp_in = fopen((_data_path + "generic_input.txt").c_str(), "rt");
    fscanf(fp_in, "%*[^\n]\n", NULL); // Dump First line

    fp_out_dynamics = fopen((_data_path + "dynamics_output.txt").c_str(), "wt");
    /* Format Header */
    fprintf(fp_out_dynamics, "time, p.x, p.y, p.z, roll, pitch, yaw, v.x, v.y, v.z, v.r, v.p, v.y\n");

    bool trajectory_end = false;
    int dynamics_sample_ = 0;

    while(!trajectory_end){

        /** Input / Trajectory Generation Loop */
        command_dt_msec_ =  chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() -
                                                                        last_command_time_msec_).count();
        if (command_dt_msec_ != 0 && command_dt_msec_ >= command_period_msec_ / time_scale){
            last_command_time_msec_ = chrono::high_resolution_clock::now();

            if(EOF == (ret = fscanf(fp_in, "%f,%f,%f,%f", &ch1_, &ch2_, &ch3_, &ch4_))) {
                ch1_ = 0.0;
                ch2_ = 0.0;
                ch3_ = 0.0;
                ch4_ = 0.0;
                trajectory_end = true;
            }
        }

        /** AUV Dynamics Loop */
        auv_dyn_dt_msec_ =  chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() -
                                                                        last_auv_dyn_time_msec_).count();
        if (auv_dyn_dt_msec_ != 0 && auv_dyn_dt_msec_ >= auv_dyn_period_msec_ / time_scale){
            last_auv_dyn_time_msec_ = chrono::high_resolution_clock::now();


            _auv_dyn->iterate(auv_dyn_dt_msec_ * time_scale / 1000.0);

        }

        /** Sensor Loop */
        sensor_dt_msec_ =  chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() -
                                                                               last_sensor_time_msec_).count();
        if (sensor_dt_msec_ != 0 && sensor_dt_msec_ >= sensor_period_msec_ / time_scale){
            last_sensor_time_msec_ = chrono::high_resolution_clock::now();

            Dynamics::States outputs_ = _auv_dyn->getStates();

            /* Log dynamics output with sensor sampling */
            fprintf(fp_out_dynamics, "%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                    (int)time_scale * chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start).count(),
                    outputs_.pos(0), outputs_.pos(1), outputs_.pos(2),
                    outputs_.att.orientation(0), outputs_.att.orientation(1), outputs_.att.orientation(2),
            outputs_.vel(0), outputs_.vel(1), outputs_.vel(2), outputs_.vel(3), outputs_.vel(4), outputs_.vel(5));
            dynamics_sample_++;

            /* Sensory Dynamics */
            // TODO: remember to multiply by timescale

        }

        /** Position Control Loop */
        pos_control_dt_msec_ =  chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() -
                                                                                    last_pos_control_time_msec_).count();
        if (pos_control_dt_msec_ != 0 && pos_control_dt_msec_ >= pos_control_period_msec_ / time_scale){
            last_pos_control_time_msec_ = chrono::high_resolution_clock::now();
            /* Log Sensor output */

            /* Pos Control */


            // TODO: remember to multiply by timescale
        }

        /** Attitude Control Loop */
        att_control_dt_msec_ =  chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() -
                                                                               last_att_control_time_msec_).count();
        if (att_control_dt_msec_ != 0 && att_control_dt_msec_ >= att_control_period_msec_ / time_scale){
            last_att_control_time_msec_ = chrono::high_resolution_clock::now();
            /* Log Pos Control Sp */


            /* Attitude Controller*/

            // TODO: remember to multiply by timescale

            /* Log Att Controller output */

            /* Pass new input to dynamics */
            inputs_.w_pwm << unity_to_pwm(ch1_), unity_to_pwm(ch2_), unity_to_pwm(ch3_), unity_to_pwm(ch4_);
            _auv_dyn->setInput(inputs_);

        }

       // this_thread::sleep_for (chrono::microseconds(100));

    }

    fclose(fp_in);
    fclose(fp_out_dynamics);
}

int main() {

    unit_sim us_;
    us_.start();

    /* Simulation Completed */
    return 1; 
}