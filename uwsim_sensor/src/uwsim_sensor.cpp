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
        _v_full_state_ros_sub( _nh.subscribe("/dolphin/dynamics/full_state", 1, &Sensor::VehicleStateCallback, this)),
        _v_power_ros_sub( _nh.subscribe("/dolphin/dynamics/power", 1, &Sensor::VehiclePowerCallback, this)),
        _v_hil_sensor_ros_pub(_nh.advertise<uwsim_msgs::hil_sensor>("/dolphin/dynamics/hil_sensor", 1)),
        _v_hil_quaternion_ros_pub(_nh.advertise<uwsim_msgs::hil_quaternion>("/dolphin/dynamics/hil_quaternion", 1)),
        _v_hil_battery_ros_pub(_nh.advertise<uwsim_msgs::hil_battery>("/dolphin/dynamics/hil_battery", 1))
{
    /* Get Parameters */
    _nh.getParam("uwsim_sensor/hil_sensor_period", _hil_sensor_period_sec);

    _nh.getParam("uwsim_sensor/hil_quaternion_period", _hil_quaternion_period_sec);
    _nh.getParam("uwsim_sensor/hil_battery_period", _hil_battery_period_sec);
    _nh.getParam("uwsim_sensor/sensor/accelerometer_st", _sen_accelerometer_std);
    _nh.getParam("uwsim_sensor/sensor/gyro_std", _sen_gyro_std);
    _nh.getParam("uwsim_sensor/sensor/mag_std", _sen_mag_std);
    _nh.getParam("uwsim_sensor/att/quaternion_std", _att_quaternion_std);
    _nh.getParam("uwsim_sensor/att/omega_std", _att_omega_std);
    _nh.getParam("uwsim_sensor/att/acceleration_std", _att_acceleration_std);

    start();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwsim_sensor");
    ros::NodeHandle privateNh("~");
    Sensor sn;

    /* Node runs exclusively within sn */

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
//        ROS_INFO("Time now %i", (int)time_now);

        if (time_now - _last_hil_sensor_sec != 0 && time_now - _last_hil_sensor_sec > _hil_sensor_period_sec){
            send_hil_sensor_msg();
            _last_hil_sensor_sec = time_now;
        }

        if (time_now - _last_hil_quaternion_sec != 0 && time_now - _last_hil_quaternion_sec > _hil_quaternion_period_sec){
            send_hil_quaternion_msg();
            _last_hil_quaternion_sec = time_now;
        }

        if (time_now - _last_hil_battery_sec != 0 && time_now - _last_hil_battery_sec > _hil_battery_period_sec){
            send_hil_batt_msg();
            _last_hil_battery_sec = time_now;
        }
//        r.sleep();
        ros::spinOnce();
    }
}

void Sensor::VehicleStateCallback(const uwsim_msgs::full_state::ConstPtr& state22_msg){
    /* update msg handle */
    _state22 = *state22_msg;
//    ROS_INFO("Got full state %f", _state22.p_q[4]);
}

void Sensor::VehiclePowerCallback(const uwsim_msgs::power::ConstPtr& power_msg){
    /* update msg handle */
    _power = *power_msg;
}

void Sensor::send_hil_sensor_msg(void){
    /* Fill _hil_sensor_msg and send */
    uwsim_msgs::hil_sensor msg;
    _v_hil_sensor_ros_pub.publish(msg);
}

void Sensor::send_hil_quaternion_msg(void){
    /* Fill _hil_quaternion_msg and send */
    uwsim_msgs::hil_quaternion msg;
    _v_hil_quaternion_ros_pub.publish(msg);
}
void Sensor::send_hil_batt_msg(void){
    uwsim_msgs::hil_battery msg;


    _v_hil_sensor_ros_pub.publish(msg);
}





