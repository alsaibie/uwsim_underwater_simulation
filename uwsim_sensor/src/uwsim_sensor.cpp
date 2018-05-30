//
// Created by alsaibie on 5/7/18.
//
#include "uwsim_sensor/uwsim_sensor.hpp"
#include <inttypes.h>
using namespace uwsim;

UWSim_Sensor::UWSim_Sensor():
        _nh(),
        // TODO: abstract vehicle name in launch file
        _vehicle_name(std::string("dolphin")),
        _v_full_state_ros_sub( _nh.subscribe("/dolphin/dynamics/full_state", 1, &UWSim_Sensor::vehicle_state_callback, this)),
        _v_power_ros_sub( _nh.subscribe("/dolphin/dynamics/power", 1, &UWSim_Sensor::vehicle_power_callback, this)),
        _v_hil_sensor_ros_pub(_nh.advertise<uwsim_msgs::hil_sensor>("/dolphin/dynamics/hil_sensor", 1)),
        _v_hil_quaternion_ros_pub(_nh.advertise<uwsim_msgs::hil_quaternion>("/dolphin/dynamics/hil_quaternion", 1)),
        _v_hil_battery_ros_pub(_nh.advertise<uwsim_msgs::hil_battery>("/dolphin/dynamics/hil_battery", 1))
{
    /* Get Parameters */
    _nh.getParam("uwsim_sensor/hil_sensor_period", _hil_sensor_period_sec);
    _nh.getParam("uwsim_sensor/hil_quaternion_period", _hil_quaternion_period_sec);
    _nh.getParam("uwsim_sensor/hil_battery_period", _hil_battery_period_sec);

    _nh.getParam("uwsim_sensor/sensor/accelerometer_std", _sen_accelerometer_std);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_bias_diffusion", _sen_acc_bias_diffusion);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_noise_density", _sen_acc_noise_density);

    _nh.getParam("uwsim_sensor/sensor/gyro_std", _sen_gyro_std);
    _nh.getParam("uwsim_sensor/sensor/gyro_noise_density", _sen_gyro_noise_density);
    _nh.getParam("uwsim_sensor/sensor/gyro_bias_diffusion", _sen_gyro_bias_diffusion);

    _nh.getParam("uwsim_sensor/sensor/mag_std", _sen_mag_std);
    _nh.getParam("uwsim_sensor/sensor/mag_inclination", _sen_mag_inclination);
    _nh.getParam("uwsim_sensor/sensor/mag_declination", _sen_mag_declination);

    _nh.getParam("uwsim_sensor/sensor/pressure_ref", _sen_pressure_ref);
    _nh.getParam("uwsim_sensor/sensor/pressure_std", _sen_pressure_std);

    _nh.getParam("uwsim_sensor/sensor/temp_ref", _sen_temp_ref);
    _nh.getParam("uwsim_sensor/sensor/temp_std", _sen_temp_std);

    //TODO: Implement Properly
    _nh.getParam("uwsim_sensor/att/quaternion_std", _att_quaternion_std);
    _nh.getParam("uwsim_sensor/att/omega_std", _att_omega_std);
    _nh.getParam("uwsim_sensor/att/acceleration_std", _att_acceleration_std);

    _sensor_dyn = new SensorDynamics();

    set_parameters();

    start();
}

void UWSim_Sensor::set_parameters(){

    Sensor::Parameters param_;

    param_.acc.std = _sen_accelerometer_std;
    param_.acc.noise_density = _sen_acc_noise_density;
    param_.acc.bias_diffusion = _sen_acc_bias_diffusion;
    param_.gyro.std = _sen_gyro_std;
    param_.gyro.noise_density = _sen_gyro_noise_density;
    param_.gyro.bias_diffusion = _sen_gyro_bias_diffusion;
    param_.mag.std = _sen_mag_std;
    param_.mag.inclination = _sen_mag_inclination;
    param_.mag.declination = _sen_mag_declination;
    param_.pressure.std = _sen_pressure_std;
    param_.pressure.ref = _sen_pressure_ref;
    param_.temp.std = _sen_temp_std;
    param_.temp.ref = _sen_temp_ref;

    _sensor_dyn->setParameters(param_);

}

void UWSim_Sensor::vehicle_state_callback(const uwsim_msgs::full_state::ConstPtr &state22_msg){
    /* Update Inputs and Iterate */
    Sensor::States input_state_ = _sensor_dyn->getInputStates();

    for(int k = 0; k < 6; k++){
        input_state_.vel(k) = state22_msg->v[k];
        input_state_.vel_dot(k) = state22_msg->v_dot[k];
    }

    for(int k = 0; k < 3; k++){
        input_state_.pos(k) = state22_msg->p_q[k];
        input_state_.pos_dot(k) = state22_msg->p_lin_dot[k];

    }

    input_state_.att.orientation_q.w() = state22_msg->p_q[3];
    input_state_.att.orientation_q.x() = state22_msg->p_q[4];
    input_state_.att.orientation_q.y() = state22_msg->p_q[5];
    input_state_.att.orientation_q.z() = state22_msg->p_q[6];

    _sensor_dyn->setInput(input_state_);

}

void UWSim_Sensor::vehicle_power_callback(const uwsim_msgs::power::ConstPtr &power_msg){
    /* update msg handle */
    Sensor::States input_state_ = _sensor_dyn->getInputStates();

    input_state_.power.bat_vnom = power_msg->V;
    input_state_.power.bat_I    = power_msg->I;
    input_state_.power.bat_percent_remaining = power_msg->percent_remaining;

    _sensor_dyn->setInput(input_state_);
}

void UWSim_Sensor::send_hil_sensor_msg(){

    Sensor::States output_state_ = _sensor_dyn->getOutputStates();

    /* And we stuff and ship msgs */
    uwsim_msgs::hil_sensor msg;
    msg.acc.x   = output_state_.imu.acc(0);
    msg.acc.y   = output_state_.imu.acc(1);
    msg.acc.z   = output_state_.imu.acc(2);
    msg.gyro.x  = output_state_.imu.gyro(0);
    msg.gyro.y  = output_state_.imu.gyro(1);
    msg.gyro.z  = output_state_.imu.gyro(2);
    msg.mag.x   = output_state_.imu.mag(0);
    msg.mag.y   = output_state_.imu.mag(1);
    msg.mag.z   = output_state_.imu.mag(2);
    msg.abs_pressure  = output_state_.pressure.abs;
    msg.diff_pressure = output_state_.pressure.differential;
    msg.pressure_alt  = output_state_.pressure.altitude;
    msg.temperature   = output_state_.temp.Celcius;

    _v_hil_sensor_ros_pub.publish(msg);
}

void UWSim_Sensor::send_hil_quaternion_msg(){

    Sensor::States output_state_ = _sensor_dyn->getOutputStates();

    /* Fill _hil_quaternion_msg and send */
    uwsim_msgs::hil_quaternion msg;

//    ROS_INFO("Orientation %f, %f, %f, %f", full_state.p_q[3], full_state.p_q[4], full_state.p_q[5], full_state.p_q[6]);
    msg.orientation.w = output_state_.att.orientation_q.w();
    msg.orientation.x = output_state_.att.orientation_q.x();
    msg.orientation.y = output_state_.att.orientation_q.y();
    msg.orientation.z = output_state_.att.orientation_q.z();

    // TODO: Fix
    msg.orientation_covariance[0] = 0;
    msg.orientation_covariance[1] = 0;
    msg.orientation_covariance[2] = 0;
    msg.orientation_covariance[3] = 0;

    // TODO: Linear acceleration in which frame?
    msg.linear_acceleration.x = output_state_.pos_dot(0);
    msg.linear_acceleration.y = output_state_.pos_dot(1);
    msg.linear_acceleration.z = output_state_.pos_dot(2);
//    msg.linear_acceleration_covariance[0] = 0;
//    msg.linear_acceleration_covariance[1] = 0;
//    msg.linear_acceleration_covariance[2] = 0;

    msg.angular_velocity.x = output_state_.vel(3);
    msg.angular_velocity.y = output_state_.vel(4);
    msg.angular_velocity.z = output_state_.vel(5);
//    msg.angular_velocity_covariance[0] = 0;
//    msg.angular_velocity_covariance[1] = 0;
//    msg.angular_velocity_covariance[2] = 0;

//    _v_hil_quaternion_ros_pub.publish(msg);
}
void UWSim_Sensor::send_hil_batt_msg(){
    uwsim_msgs::hil_battery msg;
    _v_hil_battery_ros_pub.publish(msg);
}


void UWSim_Sensor::start() {

    ros::Rate r(400); /* Run at 400 Hz and down sample */
    ros::Time begin = ros::Time::now();
    double time_now = (ros::Time::now() - begin).toSec();
    double last_hil_sensor_sec_ = time_now;
    double last_hil_quaternion_sec_ = time_now;
    double last_hil_battery_sec_ = time_now;
    double last_sensor_sec_ = time_now;

    while(ros::ok()){
        /* Send messages at requested sample rates */
        time_now = (ros::Time::now() - begin).toSec();

        //TODO: iterate at highest hil frequency requested
        double dt = time_now - last_sensor_sec_;
        _sensor_dyn->Iterate(dt);
        last_sensor_sec_ = time_now;


        double hil_sensor_dt_sec = time_now - last_hil_sensor_sec_;
        if (hil_sensor_dt_sec != 0 && hil_sensor_dt_sec > _hil_sensor_period_sec){
            send_hil_sensor_msg();
            last_hil_sensor_sec_ = time_now;
        }

        double hil_quaternion_dt_sec = time_now - last_hil_quaternion_sec_;
        if (hil_quaternion_dt_sec != 0 && hil_quaternion_dt_sec > _hil_quaternion_period_sec){
            send_hil_quaternion_msg();
            last_hil_quaternion_sec_ = time_now;
        }

        double hil_battery_dt_sec = time_now - last_hil_battery_sec_;
        if (hil_battery_dt_sec != 0 && hil_battery_dt_sec > _hil_battery_period_sec){
            send_hil_batt_msg();
            last_hil_battery_sec_ = time_now;
        }

        ros::spinOnce();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwsim_sensor");
    ros::NodeHandle privateNh("~");

    /* Node runs exclusively within sn */
    UWSim_Sensor sn_;

    return 0;
}




