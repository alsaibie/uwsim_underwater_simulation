//
// Created by alsaibie on 6/5/18.
//
#include "unit_sim.hpp"
#include <cstdlib>
#include <iostream>

using namespace uwsim;

//TODO: Move logging noise to another function if easily possible

void unit_sim::start() {

    /* sample times */
    auto auv_dyn_dt_msec_ = _diffq_period.second * 1000; // Assuming this is the fastest
    auto sensor_dt_msec_ = _pub_period.second * 1000;
    auto control_dt_msec_ = _pub_period.second * 1000;
    auto command_dt_msec_ = 10.0;

    /* scheduling multipliers */
    int auv_dyn_k_multiple_ = 1;
    int sensor_k_multiple_ = (int) (sensor_dt_msec_ / auv_dyn_dt_msec_);
    int control_k_multiple_ = (int) (control_dt_msec_ / auv_dyn_dt_msec_);
    int command_k_multiple_ = (int) (command_dt_msec_ / auv_dyn_dt_msec_);

    /* Initialize Input */
    Dynamics::Inputs inputs_;
    inputs_.w_pwm.setConstant(4, 1500);// =  Eigen::VectorXd::Constant(4, 1500);
    _auv_dyn->setInput(inputs_);
    //TODO: read pwm inputs from file

    /* Test files*/
    FILE *fp_in;

    ofstream fp_out_dynamics, fp_out_dyn_param_Mv_, fp_out_dyn_param_Cv_, fp_out_dyn_param_Dv_, fp_out_dyn_param_g_,
            fp_out_dyn_param_tau_, fp_out_power_, fp_out_sensor_, fp_out_pcontroller_, fp_out_acontroller_;

    float ch1_ = 0.0f, ch2_ = 0.0f, ch3_ = 0.0f, ch4_ = 0.0f;
    int ret;

    fp_in = fopen((_data_path + "generic_input.txt").c_str(), "rt");
    fscanf(fp_in, "%*[^\n]\n", NULL); // Dump First line

    fp_out_dynamics.open((_data_path + "dynamics_output.txt").c_str());
    fp_out_dynamics << "time,p.x,p.y,p.z,att.x,att.y,att.z,attq.w,attq.x,attq.y,attq.z,v.x,v.y,v.z,v.r,v.p,v.yaw,"
                       "pdot.x,pdot.y,pdot.z,vdot.x,vdot.y,vdot.z,vdot.r,vdot.p,vdot.yaw\n";

    fp_out_dyn_param_Mv_.open((_data_path + "dynamics_param_output_Mv.txt").c_str());
    uwsim::print_numerated_header(fp_out_dyn_param_Mv_, "m", 36);
    Dynamics::DynamicParameters dyn_param_ = _auv_dyn->getDynamicParameters();
    fp_out_dyn_param_Mv_ << "0,";
    uwsim::writeToCSVfile(fp_out_dyn_param_Mv_, dyn_param_.Mv, uwsim::CSVFormat_br);

    fp_out_power_.open((_data_path + "dynamics_power_output.txt").c_str());
    fp_out_power_ << "time,Vnom,I_mA,mAh_remaining\n";

    fp_out_dyn_param_Cv_.open((_data_path + "dynamics_param_output_Cv.txt").c_str());
    uwsim::print_numerated_header(fp_out_dyn_param_Cv_, "c", 6);

    fp_out_dyn_param_Dv_.open((_data_path + "dynamics_param_output_Dv.txt").c_str());
    uwsim::print_numerated_header(fp_out_dyn_param_Dv_, "d", 6);

    fp_out_dyn_param_g_.open((_data_path + "dynamics_param_output_g.txt").c_str());
    uwsim::print_numerated_header(fp_out_dyn_param_g_, "g", 6);

    fp_out_dyn_param_tau_.open((_data_path + "dynamics_param_output_tau.txt").c_str());
    uwsim::print_numerated_header(fp_out_dyn_param_tau_, "tau", 6);


    bool trajectory_end = false;
    int dynamics_iteration_ = 1;

    while (!trajectory_end) {

        /** Input / Trajectory Generation Loop */
        if (dynamics_iteration_ % command_k_multiple_ == 0) {
            if (EOF == (ret = fscanf(fp_in, "%f,%f,%f,%f", &ch1_, &ch2_, &ch3_, &ch4_))) {
                ch1_ = 0.0;
                ch2_ = 0.0;
                ch3_ = 0.0;
                ch4_ = 0.0;
                trajectory_end = true;
            }
        }

        /** Sensor Loop */
        if (dynamics_iteration_ % sensor_k_multiple_ == 0) {
            Dynamics::States outputs_ = _auv_dyn->getStates();
            dyn_param_ = _auv_dyn->getDynamicParameters();

            /* Log dynamics output with sensor sampling */

            fp_out_dynamics <<
                            (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ",";
            uwsim::writeToCSVfile(fp_out_dynamics, outputs_.pos, uwsim::CSVFormat_c);
            uwsim::writeToCSVfile(fp_out_dynamics, outputs_.att.orientation, uwsim::CSVFormat_c);
            fp_out_dynamics << outputs_.att.orientation_q.w() << "," << outputs_.att.orientation_q.x()
                            << "," << outputs_.att.orientation_q.y() << "," << outputs_.att.orientation_q.z()
                            << ",";
            uwsim::writeToCSVfile(fp_out_dynamics, outputs_.vel, uwsim::CSVFormat_c);
            uwsim::writeToCSVfile(fp_out_dynamics, outputs_.pos_dot, uwsim::CSVFormat_c);
            uwsim::writeToCSVfile(fp_out_dynamics, outputs_.vel_dot, uwsim::CSVFormat_br);
            fp_out_power_ <<
                            (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ",";
            fp_out_power_ << outputs_.power.bat_vnom << "," << outputs_.power.bat_I_mA
                          << "," << outputs_.power.bat_percent_remaining << "\n";

            fp_out_dyn_param_Cv_ <<
                                 (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ",";
            uwsim::writeToCSVfile(fp_out_dyn_param_Cv_, dyn_param_.Cvv, uwsim::CSVFormat_br);
            fp_out_dyn_param_Dv_ <<
                                 (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ",";
            uwsim::writeToCSVfile(fp_out_dyn_param_Dv_, dyn_param_.Dvv, uwsim::CSVFormat_br);
            fp_out_dyn_param_g_ <<
                                (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ",";
            uwsim::writeToCSVfile(fp_out_dyn_param_g_, dyn_param_.gRB, uwsim::CSVFormat_br);
            fp_out_dyn_param_tau_ <<
                                  (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ",";
            uwsim::writeToCSVfile(fp_out_dyn_param_tau_, dyn_param_.tau, uwsim::CSVFormat_br);

        }

        /**  Control Loop */
        if (dynamics_iteration_ % control_k_multiple_ == 0) {


            /*
             * Attitude Controller
             * */

            AController::ControlMode acontrol_mode_{};
            acontrol_mode_.mode = AController::CONTROL_MODE::Manual;
            _att_control->updateControlMode(acontrol_mode_);

            /* Pass new input to dynamics */

            inputs_.w_pwm << Dynamics_Math::unity_to_pwm(ch1_), Dynamics_Math::unity_to_pwm(ch2_),
                    Dynamics_Math::unity_to_pwm(ch3_), Dynamics_Math::unity_to_pwm(ch4_);
            _auv_dyn->setInput(inputs_);

        }

        /** AUV Dynamics Loop */


        if (dynamics_iteration_ % auv_dyn_k_multiple_ == 0) {
            _auv_dyn->iterate(auv_dyn_dt_msec_ / 1000.0);
            dynamics_iteration_++;
        }

    }

    fclose(fp_in);
    fp_out_dynamics.close();
    fp_out_dyn_param_g_.close();
    fp_out_dyn_param_tau_.close();
    fp_out_dyn_param_Mv_.close();
    fp_out_dyn_param_Cv_.close();
    fp_out_dyn_param_Dv_.close();
}

int main() {

    unit_sim us_;
    us_.start();

    /* Simulation Completed */
    return 1;
}