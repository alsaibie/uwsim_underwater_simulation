//
// Created by alsaibie on 5/7/18.
//
#include <functional>
#include "uwsim_px4_sitl/uwsim_px4_sitl.hpp"

using namespace px4;

Mavlink::Mavlink(std::string mavlink_fcu_url) :
        _nh(),
        _mixed_motor_cmd_ros_pub(_nh.advertise<std_msgs::Float64MultiArray>("/dolphin/thrusters_input", 1)),
        _v_attitude_quaternion_ros_sub( _nh.subscribe("/dolphin/imu_dyn", 1, &Mavlink::VehicleAttitudeCallback, this))

{
    _mavconnlink = mavconn::MAVConnInterface::open_url(mavlink_fcu_url);
    _mavconnlink->message_received_cb = std::bind(&Mavlink::handle_msg, this, std::placeholders::_1,
                                               std::placeholders::_2); // TODO: what is placeholder?

    /* Send a message to PX4 simulator but wait a bit */
    ROS_INFO("Sending an empty nonHeartbeat message");
    mavlink::common::msg::ATTITUDE_QUATERNION att_msg;
    _mavconnlink->send_message(att_msg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavlink");
    ros::NodeHandle privateNh("~");
    std::string mavlink_fcu_url;
    privateNh.param<std::string>("mavlink_fcu_url",      mavlink_fcu_url,
                                 std::string("udp://localhost:14565@localhost:14560"));
    Mavlink m(mavlink_fcu_url);

    ros::Rate r(200);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
    }
//    ros::spin();
    return 0;
}


void Mavlink::handle_msg(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing)
{
    ROS_INFO("Gor message, id %i", mmsg->msgid);
    switch (mmsg->msgid) {
        case mavlink::common::msg::HIL_ACTUATOR_CONTROLS::MSG_ID :

            handle_msg_actuator_output(mmsg);
            break;


        default:
            break;
    }
}

void Mavlink::handle_msg_actuator_output(const mavlink::mavlink_message_t *msg){

    mavlink::MsgMap _map(msg);
    mavlink::common::msg::ACTUATOR_CONTROL_TARGET _actuators;
    _actuators.deserialize(_map);

    std_msgs::Float64MultiArray _outputmsg;

    _outputmsg.data.resize(4);
    ROS_INFO("Got actuator control, id %f", _actuators.controls[0]);
    _outputmsg.data[0] = _actuators.controls[0] * 800 + 1100;
    _outputmsg.data[1] = _actuators.controls[1] * 800 + 1100;
    _outputmsg.data[2] = _actuators.controls[2] * 800 + 1100;
    _outputmsg.data[3] = _actuators.controls[3] * 800 + 1100;

    _mixed_motor_cmd_ros_pub.publish(_outputmsg);
}

void Mavlink::VehicleAttitudeCallback(const sensor_msgs::Imu::ConstPtr& att_q){

    mavlink::common::msg::ATTITUDE_QUATERNION mmsg;
    /* Fill and send */
    mmsg.q1 = (float)att_q->orientation.w;
    mmsg.q1 = (float)att_q->orientation.x;
    mmsg.q1 = (float)att_q->orientation.y;
    mmsg.q1 = (float)att_q->orientation.z;

    mmsg.rollspeed  = (float)att_q->angular_velocity.x;
    mmsg.pitchspeed = (float)att_q->angular_velocity.y;
    mmsg.yawspeed   = (float)att_q->angular_velocity.z;

    /* mavconn will pack a mavlink message with a defined id */
    _mavconnlink->send_message(mmsg);
}



