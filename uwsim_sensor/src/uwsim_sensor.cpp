//
// Created by alsaibie on 5/7/18.
//
#include "uwsim_sensor/uwsim_sensor.hpp"

using namespace uwsim;

Sensor::Sensor():
        _nh(),
        _v_full_state_ros_sub( _nh.subscribe("/uwsim/full_state", 1, &Sensor::VehicleStateCallback, this)),
        _v_power_ros_sub( _nh.subscribe("/uwsim/power", 1, &Sensor::VehiclePowerCallback, this)),
        _v_hil_sensor_ros_pub(_nh.advertise<uwsim_msgs::hil_sensor>("/uwsim/hil_sensor", 1)),
        _v_hil_quaternion_ros_pub(_nh.advertise<uwsim_msgs::hil_quaternion>("/uwsim/hil_quaternion", 1)),
        _v_hil_battery_ros_pub(_nh.advertise<uwsim_msgs::hil_battery>("/uwsim/hil_battery", 1))
{
    /* Get Paramters */
    _nh.getParam("hil_sensor_period", _hil_sensor_period);
    _nh.getParam("hil_quaternion_period", _hil_quaternion_period);
    _nh.getParam("hil_battery_period", _hil_battery_period);
    _nh.getParam("sensor/accelerometer_st", _sen_accelerometer_std);
    _nh.getParam("sensor/gyro_std", _sen_gyro_std);
    _nh.getParam("sensor/mag_std", _sen_mag_std);
    _nh.getParam("att/quaternion_std", _att_quaternion_std);
    _nh.getParam("att/omega_std", _att_omega_std);
    _nh.getParam("att/acceleration_std", _att_acceleration_std);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwsim_sensor");
    ros::NodeHandle privateNh("~");

    Sensor sn;

    ros::Rate r(400); /* Run at 400 Hz and down sample to */
//    ros::s
    while(ros::ok()){
        /* Send messages at requested sample rates */
        sn.print_internals();

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

void Sensor::VehicleStateCallback(const uwsim_msgs::full_state::ConstPtr& state24){
    /* update msg handle */
    _state24_msg = *state24;
}

void Sensor::VehiclePowerCallback(const uwsim_msgs::power::ConstPtr& power){
    /* update msg handle */
    _power_msg = *power;
}

void Sensor::send_hil_sensor_msg(void){
    /* Fill _hil_sensor_msg and send */
}

void Sensor::send_hil_quaternion_msg(void){
    /* Fill _hil_quaternion_msg and send */
}
void Sensor::send_hil_batt_msg(void){


}

void Sensor::print_internals(void) {
    int test_value;
    bool gotparam = _nh.param("hil_sensor_period", test_value);

    ROS_INFO("Got parameter %i Test Parameters %i", gotparam, test_value);
}



