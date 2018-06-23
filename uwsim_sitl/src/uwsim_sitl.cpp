//
// Created by alsaibie on 5/7/18.
//
#include <functional>
#include "uwsim_sitl/uwsim_sitl.hpp"

#define PWM_DEFAULT_MIN 1100
#define PWM_DEFAULT_MAX 1900

using namespace px4;

Mavlink::Mavlink(std::string mavlink_fcu_url) :
        _nh(),
        _mixed_motor_cmd_ros_pub(_nh.advertise<std_msgs::Float64MultiArray>("/dolphin/dynamics/thruster_inputs", 1)),
        _v_hil_sensor_ros_sub( _nh.subscribe("/dolphin/dynamics/hil_sensor", 1,
                                                 &Mavlink::vehicle_hil_sensor_callback, this)),
        _v_hil_quaternion_ros_sub(_nh.subscribe("/dolphin/dynamics/hil_quaternion", 1,
                                                &Mavlink::vehicle_hil_quaternion_callback, this)),
        _started_thruster_msg_publishing(false)

{
    _mavconnlink = mavconn::MAVConnInterface::open_url(mavlink_fcu_url);
    _mavconnlink->message_received_cb = std::bind(&Mavlink::handle_msg, this, std::placeholders::_1,
                                               std::placeholders::_2); // TODO: what is placeholder?

    /* Send a message to PX4 simulator but wait a bit */
    ROS_INFO("Sending an empty nonHeartbeat message");
    mavlink::common::msg::ATTITUDE_QUATERNION att_msg;
    _mavconnlink->send_message(att_msg);

    start();

}

void Mavlink::start(){

    std_msgs::Float64MultiArray outputmsg_;
    outputmsg_.data.resize(4);
    outputmsg_.data = {1500, 1500, 1500, 1500};
    _time_since_last_thruster_msg = ros::Time::now().toSec();
    ros::Rate r(600); // 600 hz
    while(ros::ok()){
        /* Check status of thruster messages - zero if stale */
        if(ros::Time::now().toSec() - _time_since_last_thruster_msg > 0.5 && _started_thruster_msg_publishing){
            ROS_INFO("Thruster Input Stale. Reset Input");
            _mixed_motor_cmd_ros_pub.publish(outputmsg_);
            _started_thruster_msg_publishing = false;
            _time_since_last_thruster_msg = ros::Time::now().toSec();
        }

        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavlink");
    ros::NodeHandle privateNh("~");
    std::string mavlink_fcu_url;
    privateNh.param<std::string>("mavlink_fcu_url",      mavlink_fcu_url,
                                 std::string("udp://localhost:14565@localhost:14560"));
    Mavlink m(mavlink_fcu_url);

    return 0;
}


void Mavlink::handle_msg(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing)
{

    switch (mmsg->msgid) {
        case mavlink::common::msg::HIL_ACTUATOR_CONTROLS::MSG_ID:
            handle_msg_actuator_output(mmsg);
            break;

        default:
            break;
    }
}

void Mavlink::handle_msg_actuator_output(const mavlink::mavlink_message_t *msg){

    _started_thruster_msg_publishing = true;
    _time_since_last_thruster_msg = ros::Time::now().toSec();

    mavlink::MsgMap _map(msg);
    mavlink::common::msg::HIL_ACTUATOR_CONTROLS _actuators;
    _actuators.deserialize(_map);

    std_msgs::Float64MultiArray _outputmsg;

    _outputmsg.data.resize(4);


    /* PX4 scales output to 0-1 */
    _outputmsg.data[0] = _actuators.controls[0] * (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;
    _outputmsg.data[1] = _actuators.controls[1] * (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;
    _outputmsg.data[2] = _actuators.controls[2] * (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;
    _outputmsg.data[3] = _actuators.controls[3] * (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;
    _mixed_motor_cmd_ros_pub.publish(_outputmsg);
}

void Mavlink::vehicle_hil_sensor_callback(const uwsim_msgs::hil_sensor::ConstPtr &sen_msg){

    mavlink::common::msg::HIL_SENSOR mmsg;

    /* Fill and send */
    mmsg.xacc = (float)sen_msg->acc.x;
    mmsg.yacc = (float)sen_msg->acc.y;
    mmsg.zacc = (float)sen_msg->acc.z;
    mmsg.xgyro = (float)sen_msg->gyro.x;
    mmsg.ygyro = (float)sen_msg->gyro.y;
    mmsg.zgyro = (float)sen_msg->gyro.z;
    mmsg.xmag = (float)sen_msg->mag.x;
    mmsg.ymag = (float)sen_msg->mag.y;
    mmsg.zmag = (float)sen_msg->mag.z;
    mmsg.abs_pressure = (float)sen_msg->abs_pressure;
    mmsg.diff_pressure = (float)sen_msg->diff_pressure;
    mmsg.pressure_alt = (float)sen_msg->pressure_alt;
    mmsg.temperature = (float)sen_msg->temperature;
    /* mavconn will pack a mavlink message with a defined id */
    _mavconnlink->send_message(mmsg);
}

void Mavlink::vehicle_hil_quaternion_callback(const uwsim_msgs::hil_quaternion::ConstPtr &att_msg){

    /* Ground Truth */
    mavlink::common::msg::HIL_STATE_QUATERNION mmsg;

    /* Fill and send */
    mmsg.attitude_quaternion[0] = (float)att_msg->orientation.w;
    mmsg.attitude_quaternion[1] = (float)att_msg->orientation.x;
    mmsg.attitude_quaternion[2] = (float)att_msg->orientation.y;
    mmsg.attitude_quaternion[3] = (float)att_msg->orientation.z;

    mmsg.rollspeed  = (float)att_msg->angular_velocity.x;
    mmsg.pitchspeed = (float)att_msg->angular_velocity.y;
    mmsg.yawspeed   = (float)att_msg->angular_velocity.z;


    /* mavconn will pack a mavlink message with a defined id */
//    _mavconnlink->send_message(mmsg);

}




