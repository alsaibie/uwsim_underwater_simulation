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

    int simulation_mode = 1; // 0: Direct Dynamics, 1: Attitude Control
    int attitude_control_mode = 1; // 0: Rate Mode, 1: Attitude Mode

    auto auv_dyn_dt_msec_ = _diffq_period.second * 1000; // Assuming this is the fastest
    auto sensor_dt_msec_ = _hil_sensor_period_sec.second * 1000;
    auto pos_control_dt_msec_ = _pub_period.second * 1000;
    auto att_control_dt_msec_ = _hil_sensor_period_sec.second * 1000;
    auto command_dt_msec_ = 10.0;

    /* scheduling multipliers */
    int auv_dyn_k_multiple_ = 1;
    int sensor_k_multiple_ = (int) (sensor_dt_msec_ / auv_dyn_dt_msec_);
    int att_control_k_multiple_ = (int) (att_control_dt_msec_ / auv_dyn_dt_msec_);
    int pos_control_k_multiple_ = (int) (pos_control_dt_msec_ / auv_dyn_dt_msec_);
    int command_k_multiple_ = (int) (command_dt_msec_ / auv_dyn_dt_msec_);

    /* Initialize Input */
    Dynamics::Inputs inputs_;
    inputs_.w_pwm.setConstant(4, 1500);// =  Eigen::VectorXd::Constant(4, 1500);
    _auv_dyn->setInput(inputs_);

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

    fp_out_sensor_.open((_data_path + "sensor_output.txt").c_str());
    fp_out_sensor_ << "time,acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,mag.x,mag.y,mag.z\n";

    fp_out_pcontroller_.open((_data_path + "pcontroller_output.txt").c_str());
    fp_out_pcontroller_ << "time,thrust,att.x,att.y,att.z,attq.w,attq.x,attq.y,attq.z\n";

    fp_out_acontroller_.open((_data_path + "acontroller_output.txt").c_str());
    fp_out_acontroller_
            << "time,thrust,rate_sp.x,rate_sp.y,rate_sp.z,tau_sp.x,tau_sp.y,tau_sp.z,m.1,m.2,m.3,m.4\n";

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


            /* Sensory Dynamics */
            Sensor::States sensor_input_;
            sensor_input_.att.orientation = outputs_.att.orientation;
            sensor_input_.att.orientation_q = outputs_.att.orientation_q;
            sensor_input_.vel_dot = outputs_.vel_dot;
            sensor_input_.vel = outputs_.vel;
            sensor_input_.pos_dot = outputs_.pos_dot;
            sensor_input_.pos = outputs_.pos;
            sensor_input_.power.bat_vnom = outputs_.power.bat_vnom;
            sensor_input_.power.bat_I_mA = outputs_.power.bat_I_mA;
            sensor_input_.power.bat_percent_remaining = outputs_.power.bat_percent_remaining;

            _sensor_dyn->setInput(sensor_input_);
            _sensor_dyn->Iterate(sensor_dt_msec_ / 1000.0);

        }

        /** Position Control Loop */
        if (dynamics_iteration_ % pos_control_k_multiple_ == 0) {

            /* Pos Control */
            /* Manual Control for Now */
            PController::ControlMode pcontrol_mode_{};
            pcontrol_mode_.mode = PController::CONTROL_MODE::Manual;

            _pos_control->updateControlMode(pcontrol_mode_);

            PController::Setpoints::Attitude att_sp;
            att_sp.thrust = ch4_;
            att_sp.orientation = matrix::Vector3f(ch1_, ch2_, ch3_);
//            att_sp.thrust = 0.0;
//            att_sp.orientation = matrix::Vector3f(0.0, 0.0, 0.0);
            _pos_control->updateAttitudeSetpoint(att_sp);

//            if(!_controller_mode.is_armed){ controller->resetReferenceState(); }

            _pos_control->controlAttitude((float) (pos_control_dt_msec_ / 1000.0f));

        }

        /** Attitude Control Loop */
        if (dynamics_iteration_ % att_control_k_multiple_ == 0) {

            /* Log Sensor output */
            Sensor::States sensor_output_ = _sensor_dyn->getOutputStates();
            fp_out_sensor_ <<
                           (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ",";
            uwsim::writeToCSVfile(fp_out_sensor_, sensor_output_.imu.acc, uwsim::CSVFormat_c);
            uwsim::writeToCSVfile(fp_out_sensor_, sensor_output_.imu.gyro, uwsim::CSVFormat_c);
            uwsim::writeToCSVfile(fp_out_sensor_, sensor_output_.imu.mag, uwsim::CSVFormat_br);

            /* Log Pos Control Sp */
            PController::Outputs pcontrol_output_ = _pos_control->getDesiredAttitude();
            fp_out_pcontroller_ << (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << ","
                                << pcontrol_output_.thrust << "," << pcontrol_output_.orientation(0)
                                << "," << pcontrol_output_.orientation(1) << "," << pcontrol_output_.orientation(2)
                                << ","
                                << pcontrol_output_.orientation_q(0) << ","
                                << pcontrol_output_.orientation_q(1) << "," << pcontrol_output_.orientation_q(2)
                                << "," << pcontrol_output_.orientation_q(3) << "\n";

            /*
             * Attitude Controller
             * */

            AController::ControlMode acontrol_mode_{};
            acontrol_mode_.mode = AController::CONTROL_MODE::Manual;
            _att_control->updateControlMode(acontrol_mode_);

            AController::States auv_state_;
            matrix::Quaternionf attq_((float) sensor_output_.att.orientation_q.w(),
                                      (float) sensor_output_.att.orientation_q.x(),
                                      (float) sensor_output_.att.orientation_q.y(),
                                      (float) sensor_output_.att.orientation_q.z());
            auv_state_.att.q = matrix::Quaternionf(attq_);

            //YZ axis are flipped on Bframe from IMUframe
            auv_state_.rates.rpy(0) = (float) sensor_output_.imu.gyro(0);
            auv_state_.rates.rpy(1) = -(float) sensor_output_.imu.gyro(1);
            auv_state_.rates.rpy(2) = -(float) sensor_output_.imu.gyro(2);

            // correct for in-run bias errors
//            auv_state_.rates.rpy(0) -= _sensor_bias.msg.gyro_x_bias;
//            auv_state_.rates.rpy(1) -= _sensor_bias.msg.gyro_y_bias;
//            auv_state_.rates.rpy(2) -= _sensor_bias.msg.gyro_z_bias;

            /* Power */
            //TODO: Add more battery parameters, like nominal voltage and such. Also add it in uwsim_sensor
            auv_state_.power.bat_scale_en = (bool) _bat_scale_en.second;
            auv_state_.power.bat_V_nom = 8.4;
            //TODO: Add remaining battery percentage
//            auv_state_.power.bat_scale_mA = _battery_status.msg.remaining;

            /* And update to controller */
            _att_control->updateStates(auv_state_);
            AController::Setpoints::Attitude controller_att_sp;
            AController::Setpoints::Rates controller_rates_sp;

            if(attitude_control_mode == 0){

                // TODO: Scale manual input accordingly
                controller_rates_sp.rpy = matrix::Vector3f(ch1_, ch2_, ch3_);
                controller_rates_sp.thrust = ch4_;

                _att_control->updateRateSetpoint(controller_rates_sp);
                _att_control->controlRates((float) (att_control_dt_msec_ / 1000.0f));
            }
            else if (attitude_control_mode == 1){
                controller_att_sp.q = pcontrol_output_.orientation_q;
                _att_control->updateAttitudeSetpoint(controller_att_sp);

                _att_control->controlAttitude((float) (att_control_dt_msec_ / 1000.0f));

                controller_rates_sp = _att_control->getRateSetpoint();
                controller_rates_sp.thrust = pcontrol_output_.thrust;

                _att_control->updateRateSetpoint(controller_rates_sp);
                _att_control->controlRates((float) (att_control_dt_msec_ / 1000.0f));
            }


            /* Log Att Controller output */
            matrix::Vector3f acontrol_tau_outputs_ = _att_control->getTauSetpoint();
            AController::Outputs acontrol_outputs_ = _att_control->getMixedControlOutput();
            fp_out_acontroller_ << (long) (dynamics_iteration_ * auv_dyn_dt_msec_) << "," <<
                                controller_rates_sp.thrust << "," << controller_rates_sp.rpy(0) << "," <<
                                controller_rates_sp.rpy(1) << "," << controller_rates_sp.rpy(2) << "," <<
                                acontrol_tau_outputs_(0) << "," << acontrol_tau_outputs_(1) << "," <<
                                acontrol_tau_outputs_(2) << "," <<
                                acontrol_outputs_.actuator(0) << "," << acontrol_outputs_.actuator(1) << "," <<
                                acontrol_outputs_.actuator(2) << "," << acontrol_outputs_.actuator(3) << "\n";

            /* Pass new input to dynamics */
            if (simulation_mode == 0){
                inputs_.w_pwm << Dynamics_Math::unity_to_pwm(ch1_), Dynamics_Math::unity_to_pwm(ch2_),
                        Dynamics_Math::unity_to_pwm(ch3_), Dynamics_Math::unity_to_pwm(ch4_);
            }
            else if (simulation_mode == 1){
                inputs_.w_pwm << Dynamics_Math::unity_to_pwm(acontrol_outputs_.actuator(0)),
                    Dynamics_Math::unity_to_pwm(acontrol_outputs_.actuator(1)),
                    Dynamics_Math::unity_to_pwm(acontrol_outputs_.actuator(2)),
                    Dynamics_Math::unity_to_pwm(acontrol_outputs_.actuator(3));
            }

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
    fp_out_sensor_.close();
    fp_out_pcontroller_.close();
    fp_out_acontroller_.close();
}

int main() {

    unit_sim us_;
    us_.start();

    /* Simulation Completed */
    return 1;
}