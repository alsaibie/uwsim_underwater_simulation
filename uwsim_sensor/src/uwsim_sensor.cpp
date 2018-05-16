//
// Created by alsaibie on 5/7/18.
//
#include "uwsim_sensor/uwsim_sensor.hpp"
#include <inttypes.h>
using namespace uwsim;

Sensor::Sensor():
        _nh(),
        // TODO: abstract vehicle name in launch file
        _vehicle_name(std::string("dolphin")),
        _v_full_state_ros_sub( _nh.subscribe("/dolphin/dynamics/full_state", 1, &Sensor::vehicle_state_callback, this)),
        _v_power_ros_sub( _nh.subscribe("/dolphin/dynamics/power", 1, &Sensor::vehicle_power_callback, this)),
        _v_hil_sensor_ros_pub(_nh.advertise<uwsim_msgs::hil_sensor>("/dolphin/dynamics/hil_sensor", 1)),
        _v_hil_quaternion_ros_pub(_nh.advertise<uwsim_msgs::hil_quaternion>("/dolphin/dynamics/hil_quaternion", 1)),
        _v_hil_battery_ros_pub(_nh.advertise<uwsim_msgs::hil_battery>("/dolphin/dynamics/hil_battery", 1))
{
    /* Get Parameters */
    _nh.getParam("uwsim_sensor/hil_sensor_period", _hil_sensor_period_sec);
    _nh.getParam("uwsim_sensor/hil_quaternion_period", _hil_quaternion_period_sec);
    _nh.getParam("uwsim_sensor/hil_battery_period", _hil_battery_period_sec);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_std", _sen_accelerometer_std);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_bias_correlation_time", _sen_acc_bias_correlation_time);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_noise_density", _sen_acc_noise_density);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_random_walk", _sen_acc_random_walk);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_turn_on_bias", _sen_acc_turn_on_bias);
    _nh.getParam("uwsim_sensor/sensor/gyro_std", _sen_gyro_std);
    _nh.getParam("uwsim_sensor/sensor/gyro_bias_correlation_time", _sen_gyro_bias_correlation_time);
    _nh.getParam("uwsim_sensor/sensor/gyro_noise_density", _sen_gyro_noise_density);
    _nh.getParam("uwsim_sensor/sensor/gyro_random_walk", _sen_gyro_random_walk);
    _nh.getParam("uwsim_sensor/sensor/gyro_turn_on_bias", _sen_gyro_turn_on_bias);
    _nh.getParam("uwsim_sensor/sensor/mag_std", _sen_mag_std);
    _nh.getParam("uwsim_sensor/sensor/mag_inclination", _sen_mag_inclination);
    _nh.getParam("uwsim_sensor/sensor/mag_declination", _sen_mag_declination);
    _nh.getParam("uwsim_sensor/sensor/pressure_ref", _sen_pressure_ref);
    _nh.getParam("uwsim_sensor/sensor/pressure_std", _sen_pressure_std);
    _nh.getParam("uwsim_sensor/sensor/temp_ref", _sen_temp_ref);
    _nh.getParam("uwsim_sensor/sensor/temp_std", _sen_temp_std);
    _nh.getParam("uwsim_sensor/att/quaternion_std", _att_quaternion_std);
    _nh.getParam("uwsim_sensor/att/omega_std", _att_omega_std);
    _nh.getParam("uwsim_sensor/att/acceleration_std", _att_acceleration_std);


    /* Initialize Stochastics */
    _sen_acc_dist = std::normal_distribution<float>(0.0, _sen_accelerometer_std);
    _sen_gyro_dist = std::normal_distribution<float>(0.0, _sen_gyro_std);
    _sen_mag_dist = std::normal_distribution<float>(0.0, _sen_mag_std);
    _sen_pressure_dist = std::normal_distribution<float>(0.0, _sen_pressure_std);
    _sen_temp_dist = std::normal_distribution<float>(0.0, _sen_temp_std);

    /* Get Magnetic Declination/Inclination Vector q = R(Yaw:Declination)*R(Pitch:Inclination) */
    q_magu = AngleAxisd(_sen_mag_declination * M_PI / 180.0, Vector3d::UnitZ()) * AngleAxisd(- _sen_mag_inclination * M_PI / 180.0, Vector3d::UnitY());

    start();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwsim_sensor");
    ros::NodeHandle privateNh("~");

    /* Node runs exclusively within sn */
    Sensor sn;

    return 0;
}
void Sensor::start(void) {

    ros::Rate r(400); /* Run at 400 Hz and down sample */
    ros::Time begin = ros::Time::now();
    double time_now = (ros::Time::now() - begin).toSec();
    _last_hil_sensor_sec = time_now;
    _last_hil_quaternion_sec = time_now;
    _last_hil_battery_sec = time_now;

    while(ros::ok()){
        /* Send messages at requested sample rates */
        double time_now = (ros::Time::now() - begin).toSec();

        double hil_sensor_dt_sec = time_now - _last_hil_sensor_sec;
        if (hil_sensor_dt_sec != 0 && hil_sensor_dt_sec > _hil_sensor_period_sec){
            send_hil_sensor_msg(hil_sensor_dt_sec);
            _last_hil_sensor_sec = time_now;
        }

        double hil_quaternion_dt_sec = time_now - _last_hil_quaternion_sec;
        if (hil_quaternion_dt_sec != 0 && hil_quaternion_dt_sec > _hil_quaternion_period_sec){
            send_hil_quaternion_msg(hil_quaternion_dt_sec);
            _last_hil_quaternion_sec = time_now;
        }

        double hil_battery_dt_sec = time_now - _last_hil_battery_sec;
        if (hil_battery_dt_sec != 0 && hil_battery_dt_sec > _hil_battery_period_sec){
            send_hil_batt_msg(hil_battery_dt_sec);
            _last_hil_battery_sec = time_now;
        }
        ros::spinOnce();
    }
}

void Sensor::vehicle_state_callback(const uwsim_msgs::full_state::ConstPtr &state22_msg){
    /* update msg handle */
    _state22 = *state22_msg;
}

void Sensor::vehicle_power_callback(const uwsim_msgs::power::ConstPtr &power_msg){
    /* update msg handle */
    _power = *power_msg;
}

void Sensor::send_hil_sensor_msg(double dt){
    /* Make a local copy of _state22 - In case of thread implementation (make atomic then) */
    uwsim_msgs::full_state full_state = _state22;
    /* u - UWSim uses NED convention: North, East, Down */
    /* d - Dolphin uses FRD: Forward, Right, Down */
    /* m - IMU uses FLU: Forward, Left, Up */

    /* Orientation - frame d expressed in u frame */

    Quaterniond q_orientation_du(full_state.p_q[3], full_state.p_q[4], full_state.p_q[5], full_state.p_q[6]);
    q_orientation_du.normalize();

    /*
     * ACCELEROMETER
     */

    /* Get Linear Acceleration in d frame */
    Vector3d acc_d(full_state.v_dot[0],full_state.v_dot[1], full_state.v_dot[2]);

    /* Add g - rotated into d frame */
    Quaterniond g_u; g_u.w() = 0.0; g_u.vec() = Vector3d(0,0,9.81);
    acc_d += (q_orientation_du.inverse() * g_u * q_orientation_du).vec();

    /* Rotate Linear Acceleration to m frame */
    Quaterniond q_dm(0, 1, 0, 0);
    Quaterniond qacc_d; qacc_d.w() = 0; qacc_d.vec() = acc_d;
    Vector3d acc_m = (q_dm * qacc_d * q_dm.inverse()).vec();

    /* Make real */
    acclerometer_real(acc_d, dt);

    /*
     * GYROSCOPE
     */
    Vector3d gyro_d(full_state.v[3], full_state.v[4], full_state.v[5]);

    /* Bring to m frame */
    Quaterniond qgyro_d; qgyro_d.w() = 0; qgyro_d.vec() = gyro_d;
    Vector3d gyro_m = (q_dm * qgyro_d * q_dm.inverse()).vec();

    gyroscope_real(gyro_d, dt);

    /*
     * MAGNETOMETER
     */

    /* Compensate for magnetic inclination/declination */
    Quaterniond qmag; qmag.w() = 0; qmag.vec() = Vector3d(1, 0, 0);
    Quaterniond qmag_u  = q_magu * qmag * q_magu.inverse();

    /* Bring to body frame d then to m frame */
    Vector3d mag_m =  (q_dm * (q_orientation_du.inverse() * qmag_u * q_orientation_du) * q_dm.inverse()).vec();

    magnetometer_real(mag_m, dt);

    /* And we stuff and ship msgs */
    uwsim_msgs::hil_sensor msg;
    msg.acc.x = acc_m[0]; msg.acc.y = acc_m[1]; msg.acc.z = acc_m[2];
    msg.gyro.x = gyro_m[0]; msg.gyro.y = gyro_m[1]; msg.gyro.z = gyro_m[2];
    msg.mag.x = mag_m[0]; msg.mag.y = mag_m[1]; msg.mag.z = mag_m[2];
    msg.abs_pressure   += _sen_pressure_dist(_rand_generator);
    msg.temperature    += _sen_temp_dist(_rand_generator);
    _v_hil_sensor_ros_pub.publish(msg);
}

void Sensor::send_hil_quaternion_msg(double dt){
    /* Make a local copy of _state22 */
    uwsim_msgs::full_state full_state = _state22;

    /* Fill _hil_quaternion_msg and send */
    uwsim_msgs::hil_quaternion msg;

    msg.orientation.w = full_state.p_q[3];
    msg.orientation.x = full_state.p_q[4];
    msg.orientation.y = full_state.p_q[5];
    msg.orientation.z = full_state.p_q[6];

    // TODO: Fix
    msg.orientation_covariance[0] = 0;
    msg.orientation_covariance[1] = 0;
    msg.orientation_covariance[2] = 0;
    msg.orientation_covariance[3] = 0;

    // TODO: Linear acceleration in which frame?
//    msg.linear_acceleration.x = 0;
//    msg.linear_acceleration.y = 0;
//    msg.linear_acceleration.z = 0;
//    msg.linear_acceleration_covariance[0] = 0;
//    msg.linear_acceleration_covariance[1] = 0;
//    msg.linear_acceleration_covariance[2] = 0;

    msg.angular_velocity.x = full_state.v[0];
    msg.angular_velocity.y = full_state.v[1];
    msg.angular_velocity.z = full_state.v[2];
//    msg.angular_velocity_covariance[0] = 0;
//    msg.angular_velocity_covariance[1] = 0;
//    msg.angular_velocity_covariance[2] = 0;

    _v_hil_quaternion_ros_pub.publish(msg);
}
void Sensor::send_hil_batt_msg(double dt){
    uwsim_msgs::hil_battery msg;

    _v_hil_battery_ros_pub.publish(msg);
}


void Sensor::acclerometer_real(Vector3d &acc, double dt) {

    static double bias = 0;
    for (uint8_t k = 0; k < 3; k++){
        double sig_1 = sqrt(-_sen_acc_random_walk * _sen_acc_random_walk * _sen_acc_bias_correlation_time / 2.0 *
                            exp(-2.0 * dt / _sen_acc_bias_correlation_time));
        double sig_2 = 1 / sqrt(dt) * _sen_acc_noise_density;
        double sig_3 = _sen_acc_turn_on_bias;
        double phi   = exp(-dt / _sen_acc_bias_correlation_time);
        bias = phi * bias + sig_1 * _sen_acc_dist(_rand_generator);
        acc[k] +=(float)(bias + sig_2 * _sen_acc_dist(_rand_generator) + sig_3 * _sen_acc_dist(_rand_generator));
    }
}

void Sensor::gyroscope_real(Vector3d &gyro, double dt) {

    static double bias = 0;
    for (uint8_t k = 0; k < 3; k++){
        double sig_1 = sqrt(-_sen_gyro_random_walk * _sen_gyro_random_walk * _sen_gyro_bias_correlation_time / 2.0 *
                            exp(-2.0 * dt / _sen_gyro_bias_correlation_time));
        double sig_2 = 1 / sqrt(dt) * _sen_gyro_noise_density;
        double sig_3 = _sen_gyro_turn_on_bias;
        double phi   = exp(-dt / _sen_gyro_bias_correlation_time);
        bias = phi * bias + sig_1 * _sen_gyro_dist(_rand_generator);
        gyro[k] +=(float)(bias + sig_2 * _sen_gyro_dist(_rand_generator) + sig_3 * _sen_gyro_dist(_rand_generator));
    }
}

void Sensor::magnetometer_real(Vector3d &mag, double dt) {


    mag[0]   += _sen_mag_dist(_rand_generator);
    mag[1]   += _sen_mag_dist(_rand_generator);
    mag[2]   += _sen_mag_dist(_rand_generator);

}




