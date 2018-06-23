//
// Created by alsaibie on 5/30/18.
//


#include "Sensor.hpp"
#include <iostream>
using namespace Eigen;
using namespace Dynamics_Math;
using namespace Sensor;

SensorDynamics::SensorDynamics(){
    _output_state.imu.acc.setZero();
    _output_state.imu.gyro.setZero();
    _output_state.imu.mag.setZero();
    _input_state.vel.setZero();
    _input_state.pos.setZero();
    _input_state.vel_dot.setZero();
    _input_state.pos_dot.setZero();
    _input_state.att.orientation_q.setIdentity();
    _input_state.att.orientation.setZero();

}

void SensorDynamics::Initialize(){
    /* Initialize Stochastics */
    _sen_acc_dist = std::normal_distribution<double>(0.0, _param.acc.std);
    _sen_gyro_dist = std::normal_distribution<double>(0.0, _param.gyro.std);
    _sen_mag_dist = std::normal_distribution<double>(0.0, _param.mag.std);
    _sen_pressure_dist = std::normal_distribution<double>(0.0, _param.pressure.std);
    _sen_temp_dist = std::normal_distribution<double>(0.0, _param.temp.std);

/* Store Magnetic Declination/Inclination Vector q = R(Yaw:Declination)*R(Pitch:Inclination) */
    qmag_fu = euler2Quaterniond(Vector3d(0.0, _param.mag.inclination * M_PI / 180.0,
                                         _param.mag.declination * M_PI / 180.0)).inverse();

    qmag_fu.normalize();

}

/** For acc and gyro sing noise model suggested in here:
 * https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
 * where acc_hat = acc + b[k] + n[k]
 * the bias term b[k] = b[k-1] + sigma_b * w[k] * sqrt(dt)
 * and n[k] = sigma * w[k] / sqrt(dt)
 * **/
void SensorDynamics::acclerometer_real(Vector3d &acc, double dt) {

    static double bias = 0;
    for (uint8_t k = 0; k < 3; k++){
        bias = bias + _param.acc.bias_diffusion * _sen_acc_dist(_rand_generator) * sqrt(dt);
        acc[k] +=(float)(bias + _param.acc.noise_density * _sen_acc_dist(_rand_generator) / sqrt(dt));
    }

}

void SensorDynamics::gyroscope_real(Vector3d &gyro, double dt) {

    static double bias = 0;
    for (uint8_t k = 0; k < 3; k++){
        bias = bias + _param.gyro.bias_diffusion * _sen_gyro_dist(_rand_generator) * sqrt(dt);
        gyro[k] +=(float)(bias + _param.gyro.noise_density * _sen_gyro_dist(_rand_generator) / sqrt(dt));
    }
}

void SensorDynamics::magnetometer_real(Vector3d &mag) {

    /** Add Gaussian noise  **/
    for (uint8_t k = 0; k < 3; k++){
        mag[k]   += _sen_mag_dist(_rand_generator);
    }
}
void SensorDynamics::Iterate(const double &dt){

    /* u - UWSim uses NED convention: North, East, Down */
    /* d - Dolphin uses FRD: Forward, Right, Down */
    /* m - IMU uses FLU: Forward, Left, Up */

    /* Orientation - frame d expressed in u frame */

    Quaterniond q_orientation_du =_input_state.att.orientation_q;
    q_orientation_du.normalize();

    /*
     * ACCELEROMETER
     */

    /* Get Linear Acceleration in d frame */
    Vector3d acc_d = _input_state.vel_dot.head(3);

    /* Add g - rotated into d frame */
    Quaterniond g_u = QuaternionVectord( Vector3d(0,0,9.81));
    acc_d += (q_orientation_du.inverse() * g_u * q_orientation_du).vec();

    /* Rotate Linear Acceleration to m frame */
    Quaterniond q_dm(0, 1, 0, 0);
    Quaterniond qacc_d = QuaternionVectord(acc_d);
    Vector3d acc_m = (q_dm * qacc_d * q_dm.inverse()).vec();

    /* Make real */
    acclerometer_real(acc_m, dt);
    _output_state.imu.acc = acc_m;

    /*
     * GYROSCOPE
     */
    Vector3d gyro_d = _input_state.vel.tail(3);

    /* Bring to m frame */
    Quaterniond qgyro_d = QuaternionVectord(gyro_d);
    Vector3d gyro_m = (q_dm * qgyro_d * q_dm.inverse()).vec();

    gyroscope_real(gyro_m, dt);

    _output_state.imu.gyro = gyro_m;
    /*
     * MAGNETOMETER TODO: STILL NEEDS WORK.
     */

    /* Compensate true magnetic inclination/declination */
    Quaterniond qnorth = QuaternionVectord(Vector3d(1, 0, 0));
    Vector3d mag_u  = (qmag_fu * qnorth * qmag_fu.inverse()).vec();

    /* Bring to body d frame */
    Quaterniond qmag_u = QuaternionVectord(mag_u);
    Vector3d mag_m =  (q_orientation_du.inverse() * qmag_u * q_orientation_du).vec();

    magnetometer_real(mag_m);

    _output_state.imu.mag = mag_m;

    // TODO: Properly populate pressure and temperature
    _output_state.pressure.abs              += _sen_pressure_dist(_rand_generator);
    _output_state.pressure.differential     += _sen_pressure_dist(_rand_generator);
    _output_state.pressure.altitude         += _sen_pressure_dist(_rand_generator);
    _output_state.temp.Celcius              += _sen_temp_dist(_rand_generator);

    /**
     * Pass true values to output for export, used to simulate global estimater
     * TODO: Add some noise
     * */

    _output_state.vel       = _input_state.vel;
    _output_state.vel_dot   = _input_state.vel_dot;
    _output_state.att       = _input_state.att;
    _output_state.pos       = _input_state.pos;
    _output_state.pos_dot   = _input_state.pos_dot;

}