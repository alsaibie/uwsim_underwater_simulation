#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <ros/console.h>

/***
 * This is a simple node that reads the joystick data and aligns it properly to be sent as manual_sp. Nothin fancy
 */
class JoystickUWSim
{
public:
    JoystickUWSim();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  int  thrust_, roll_, pitch_, yaw_;
  int  thrust_invert_, roll_invert_, pitch_invert_, yaw_invert_;
  int  l_range_, l_start_;
  ros::Publisher  manual_sp_pub_;
  ros::Subscriber joy_sub_;
  bool thrust_init, roll_init, pitch_init, yaw_init;
};


JoystickUWSim::JoystickUWSim():
  thrust_invert_(1), roll_invert_(1), pitch_invert_(1), yaw_invert_(1),
  thrust_init(false), roll_init(false), pitch_init(false), yaw_init(false)
{
  nh_.param("thrust", thrust_, thrust_);
  nh_.param("roll", roll_, roll_);
  nh_.param("pitch", pitch_, pitch_);
  nh_.param("yaw", yaw_, yaw_);
  nh_.param("thrust_inv", thrust_invert_, thrust_invert_);
  nh_.param("roll_inv", roll_invert_, roll_invert_);
  nh_.param("pitch_inv", pitch_invert_, pitch_invert_);
  nh_.param("yaw_inv", yaw_invert_, yaw_invert_);
  nh_.param("linear_range", l_range_, l_range_);
  nh_.param("linear_start_", l_start_, l_start_);

  manual_sp_pub_ = nh_.advertise<sensor_msgs::Joy>("/dolphin/manual_sp", 1000);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickUWSim::joyCallback, this);
}

void JoystickUWSim::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  sensor_msgs::Joy msg;
  msg.axes.clear();

  // Roll
//  ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f, Thrust: %f", (double) joy->axes[roll_],
//           (double) joy->axes[pitch_], (double) joy->axes[yaw_], (double) joy->axes[thrust_]);

  /** Remap the sticks and make sure to send a zero if not initialized **/
  if(!roll_init && (int) (roll_invert_ * joy->axes[roll_] * 1000) <= 1){
    msg.axes.push_back(0);
  }
  else
  {
    roll_init = true;
    msg.axes.push_back(roll_invert_ * (joy->axes[roll_] * 2 / l_range_) - l_start_ - 1);
  }

  // Pitch
  if(!pitch_init && (int) (pitch_invert_ * joy->axes[pitch_] * 1000) <= 1){
    msg.axes.push_back(0);
  }
  else
  {
    msg.axes.push_back(pitch_invert_ * (joy->axes[pitch_] * 2 / l_range_) - l_start_ - 1);
    pitch_init = true;
  }
  // Yaw
  if(!yaw_init &&(int) (yaw_invert_ * joy->axes[yaw_] * 1000) <= 1){
    msg.axes.push_back(0);
  }
  else
  {
    msg.axes.push_back(yaw_invert_ * (joy->axes[yaw_] * 2 / l_range_) - l_start_ - 1);
    yaw_init = true;
  }
  // Thrust
  if(!thrust_init && (int) (thrust_invert_ * joy->axes[thrust_] * 1000) <= 1){
    msg.axes.push_back(0);
  }
  else
  {
    msg.axes.push_back(thrust_invert_ * (joy->axes[thrust_] * 2 / l_range_) - l_start_ - 1);
    thrust_init = true;
  }

  // Thrust
  manual_sp_pub_.publish(msg);

}

void run(int* publish_rate){
  while(ros::ok()){
    ros::spinOnce();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_uwsim");
  JoystickUWSim uwsim_joystick;
  int rate = 100; // 100 Hz
  boost::thread thread_main(run, &rate);
  thread_main.join();
  return 0;
}