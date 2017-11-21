#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <ros/console.h>
#include <uwsim_control/px4_compatibility.hpp>

// TODO: add IMU(attitude) data

#define PX4_INFO ROS_INFO
#define PX4_ISFINITE(x) std::isfinite(x)

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f
#define MANUAL_THROTTLE_MAX_DOLPHIN	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3




class ControlUWSim
{
public:
    ControlUWSim();

    void spin_controller();

private:
  //TODO: change manual_sp data type
  void manualspCallback(const sensor_msgs::Joy::ConstPtr& manual_sp);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& att);
  ros::NodeHandle nh_;
  ros::Publisher mixed_pwm_pub_;
  ros::Publisher att_rates_pid_rates_pub_;
  ros::Subscriber manual_sp_sub_;
  ros::Subscriber att_sub_;
  Eigen::Vector4f mixed_att_output_;

  void control_attitude(double dt);
  void control_attitude_rates(double dt);
  void mix_control_output(Eigen::Vector3f &att_control_, float thrust_sp, Eigen::Vector4f &mixed_att_control_);
  Eigen::Vector3f pid_attenuations(float tpa_breakpoint, float tpa_rate);
  /** A few defines to match those of px setup **/
  vehicle_attitude_setpoint_s _v_att_sp;
  control_state_s _ctrl_state;
  vehicle_control_mode_s _v_control_mode;
  vehicle_rates_setpoint_s _v_rates_sp;
  actuator_armed_s _armed;
  manual_control_setpoint_s  _manual_control_sp;
  vehicle_status_s _vehicle_status;
  dp_att_ctrl_status_s _controller_status;
  actuator_controls_s			_actuators;
  sensor_gyro_s			_sensor_gyro;

    union {
      struct {
          uint16_t motor_pos	: 1; // 0 - true when any motor has saturated in the positive direction
          uint16_t motor_neg	: 1; // 1 - true when any motor has saturated in the negative direction
          uint16_t roll_pos	: 1; // 2 - true when a positive roll demand change will increase saturation
          uint16_t roll_neg	: 1; // 3 - true when a negative roll demand change will increase saturation
          uint16_t pitch_pos	: 1; // 4 - true when a positive pitch demand change will increase saturation
          uint16_t pitch_neg	: 1; // 5 - true when a negative pitch demand change will increase saturation
          uint16_t yaw_pos	: 1; // 6 - true when a positive yaw demand change will increase saturation
          uint16_t yaw_neg	: 1; // 7 - true when a negative yaw demand change will increase saturation
          uint16_t thrust_pos	: 1; // 8 - true when a positive thrust demand change will increase saturation
          uint16_t thrust_neg	: 1; // 9 - true when a negative thrust demand change will increase saturation
      } flags;
      uint16_t value;
  } _saturation_status;

  Eigen::Vector3f _rates_prev;
  Eigen::Vector3f _rates_sp;
  Eigen::Vector3f _rates_sp_prev;
  Eigen::Vector3f _rates_int;
  float _thrust_sp;
  Eigen::Vector3f _att_control;
  Eigen::Vector4f _mixed_att_control;

  Eigen::Matrix3f _I;
  Eigen::Matrix3f _board_rotation = {};

    struct {
      Eigen::Vector3f att_p;					/**< P gain for angular error */
      Eigen::Vector3f rate_p;				/**< P gain for angular rate error */
      Eigen::Vector3f rate_i;				/**< I gain for angular rate error */
      Eigen::Vector3f rate_int_lim;			/**< integrator state limit for rate loop */
      Eigen::Vector3f rate_d;				/**< D gain for angular rate error */
      Eigen::Vector3f	rate_ff;			/**< Feedforward gain for desired rates */
      float yaw_ff;						/**< yaw control feed-forward */
      float roll_ff;						/**< roll control feed-forward */

      float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
      float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
      float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
      float tpa_rate_p;					/**< Throttle PID Attenuation slope */
      float tpa_rate_i;					/**< Throttle PID Attenuation slope */
      float tpa_rate_d;					/**< Throttle PID Attenuation slope */

      float roll_rate_max;
      float pitch_rate_max;
      float yaw_rate_max;
      float yaw_auto_max;
      Eigen::Vector3f dp_rate_max;		/**< attitude rate limits in stabilized modes */
      Eigen::Vector3f auto_rate_max;		/**< attitude rate limits in auto modes */
      Eigen::Vector3f acro_rate_max;		/**< max attitude rates in acro mode */
      float rattitude_thres;

      int motion_type;
      float thrust_factor;
      float idle_speed;

      int bat_scale_en;

      int board_rotation;

      float board_offset[3];

  }		_params;
    /**
   * Rotor Mixing scales
   */
    static const int _rotor_count{4};
    static const int _mix_length{5};
    struct  rotors_vector{
        float	roll_scale;	/**< scales roll for this rotor */
        float	pitch_scale;	/**< scales pitch for this rotor */
        float	yaw_scale;	/**< scales yaw for this rotor */
        float thrust_scale; /**< scales Thrust for this rotor */
        float	out_scale;	/**< scales total out for this rotor */
    } _rotors[_rotor_count];

    /** Mixing Table. Order: Rollscale, PitchScale, YawScale, ThrustScale, OutScale */
//    static constexpr float _dolphin_x_table[_rotor_count][_mix_length] = {
//            { -1.000000,  0.707107,  -0.707107, 1.000000, 1.000000 },
//            { -1.000000,  -0.707107,  0.707107, 1.000000, 1.000000 },
//            { 1.000000, 0.707107,   0.707107, 1.000000, 1.000000 },
//            { 1.000000, -0.707107, -0.707107, 1.000000, 1.000000 },
//    };
    float _dolphin_x_table[_rotor_count][_mix_length] = {
            { -1.000000,  0.5,  -0.5, 1.000000, 1.000000 },
            { -1.000000,  -0.5,  0.5, 1.000000, 1.000000 },
            { 1.000000, 0.5,   0.5, 1.000000, 1.000000 },
            { 1.000000, -0.5, -0.5, 1.000000, 1.000000 },
    };
};

ControlUWSim::ControlUWSim():
        _v_att_sp{},
        _ctrl_state{},
        _v_control_mode{},
        _v_rates_sp{},
        _armed{},
        _manual_control_sp{},
        _vehicle_status{},
        _controller_status{},
        _actuators{},
        _sensor_gyro{}

{
  mixed_pwm_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/dolphin/thrusters_input", 1);
  att_rates_pid_rates_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/dolphin/att_rates_pid_rates", 1);
  att_sub_ =       nh_.subscribe<sensor_msgs::Imu>("/dolphin/imu_dyn", 10, &ControlUWSim::imuCallback, this);
  manual_sp_sub_ = nh_.subscribe<sensor_msgs::Joy>("/dolphin/manual_sp", 10, &ControlUWSim::manualspCallback, this);
  //  mixed_att_output_.set

  for (int k = 0; k < _rotor_count; k++) {
    _rotors[k].roll_scale = _dolphin_x_table[k][0];
    _rotors[k].pitch_scale = _dolphin_x_table[k][1];
    _rotors[k].yaw_scale = _dolphin_x_table[k][2];
    _rotors[k].thrust_scale = _dolphin_x_table[k][3];
    _rotors[k].out_scale = _dolphin_x_table[k][4];
  }

  _armed.armed = true;
  _v_control_mode.flag_control_attitude_enabled = true;
  _v_control_mode.flag_control_rates_enabled = true;
  _v_control_mode.flag_control_manual_enabled = true;

  _I.setIdentity();

  // TODO: Populate Parameters Preferabbly from a YAML file?
  float v;
  float roll_tc, pitch_tc;
  roll_tc = 0.2f;
  pitch_tc = 0.2f;
  v = 0.5f; //roll_p
  _params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
  v = 0.15f; //roll_rate_p
  _params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
  v = 0.05f; // roll rate i
  _params.rate_i(0) = v;
  v = 0.3f; //roll rate integ lim
  _params.rate_int_lim(0) = v;
  v = 0.003f; //roll rate d
  _params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
  v = 0.0f; // roll rate ff
  _params.rate_ff(0) = v;

  /* pitch gains */
  v = 0.5f; // pitch p
  _params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
  v = 0.15f; // pitch rate p
  _params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
  v = 0.05f; // pitch rate i
  _params.rate_i(1) = v;
  v = 0.3f; // pitch rate integ limit
  _params.rate_int_lim(1) = v;
  v = 0.003f; // pitch rate d
  _params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
  v = 0.0f; // pitch rate ff
  _params.rate_ff(1) = v;


  _params.tpa_breakpoint_p = 0.0f;
  _params.tpa_breakpoint_i = 0.0f;
  _params.tpa_breakpoint_d = 0.0f;
  _params.tpa_rate_p = 0.0f;
  _params.tpa_rate_i = 0.0f;
  _params.tpa_rate_d = 0.0f;

  /* yaw gains TODO: Maybe I need to move the roll_tc multiplier to yaw here ? */
  v = 0.8f; // yaw p
  _params.att_p(2) = v;
  v = 0.2f; // yaw rate p
  _params.rate_p(2) = v;
  v = 0.1f; // yaw rate i
  _params.rate_i(2) = v;
  v = 0.3f; // yaw rate int limit
  _params.rate_int_lim(2) = v;
  v = 0.0f; // yaw rate d
  _params.rate_d(2) = v;
  v = 0.0f; // yaw rate ff
  _params.rate_ff(2) = v;
  _params.yaw_ff = 0.5f;

  /* angular rate limits */
  _params.roll_rate_max = 220.0f;
  _params.dp_rate_max(0) = math::radians(_params.roll_rate_max);
  _params.pitch_rate_max = 220.0f;
  _params.dp_rate_max(1) = math::radians(_params.pitch_rate_max);
  _params.yaw_rate_max = 200.0f;
  _params.dp_rate_max(2) = math::radians(_params.yaw_rate_max);

  /* auto angular rate limits */
  _params.roll_rate_max = 0.0f;
  _params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
  _params.pitch_rate_max = 0.0f;
  _params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
  _params.yaw_auto_max = 45.0f;
  _params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

  /* manual rate control scale and auto mode roll/pitch rate limits */
  v = 360.0f; // acro roll max
  _params.acro_rate_max(0) = math::radians(v);
  v = 360.0f; // acro pitch max
  _params.acro_rate_max(1) = math::radians(v);
  v = 360.0f; // acro yaw max
  _params.acro_rate_max(2) = math::radians(v);

  /* stick deflection needed in rattitude mode to control rates not angles */

  _params.rattitude_thres = 1.0f;

  _params.bat_scale_en = 0.0f;

  _params.tpa_breakpoint_p = 1.0f;
  _params.tpa_breakpoint_i = 1.0f;
  _params.tpa_breakpoint_d = 1.0f;
  _params.tpa_rate_p = 0.0f;
  _params.tpa_rate_i = 0.0f;
  _params.tpa_rate_d = 0.0f;
  _params.tpa_rate_d = 0.0f;

  /* rotation of the autopilot relative to the body */
  _params.board_rotation = 0.0f;

  /* fine adjustment of the rotation */
  _params.board_offset[0] = 0.0f;
  _params.board_offset[1] = 0.0f;
  _params.board_offset[2] = 0.0f;

  _params.thrust_factor = 0.25f;
  _params.idle_speed = 0.15f;

  _ctrl_state.roll_rate_bias = 0;
  _ctrl_state.pitch_rate_bias = 0;
  _ctrl_state.yaw_rate_bias = 0;

}

void ControlUWSim::manualspCallback(const sensor_msgs::Joy::ConstPtr& manual_sp)
{
  /* Pass manual_sp to struct  */
  _manual_control_sp.y = manual_sp->axes[0];
  _manual_control_sp.x = manual_sp->axes[1];
  _manual_control_sp.r = manual_sp->axes[2];
  _manual_control_sp.z = manual_sp->axes[3];
}

void ControlUWSim::imuCallback(const sensor_msgs::Imu::ConstPtr& att) {

  //  /** Convert to RPY **/
  //  tf::Quaternion quat;
  //  tf::quaternionMsgToTF(att->orientation, quat);
  //  double roll, pitch, yaw;
  //
  //  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  //  geometry_msgs::Vector3 rpy;
  //  rpy.x = roll;
  //  rpy.y = pitch;
  //  rpy.z = yaw;
  //
  //
  //  mixed_att_output_(0) = rpy.x;
  //  mixed_att_output_(1) = rpy.y;
  //  mixed_att_output_(2) = rpy.z;
  //  mixed_att_output_(3) = 1;

  /* Pass att quaternion to ctrl state struct */
  //  tf::Quaternion q(att->orientation.x, att->orientation.y, att->orientation.z, att->orientation.z);
  //  tf::Matrix3x3 m(q);
  //  double roll, pitch, yaw;
  //  m.getRPY(roll, pitch, yaw);
  //  ROS_INFO("Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);

  _ctrl_state.q[0] =   att->orientation.x;
  _ctrl_state.q[1] =   att->orientation.y;
  _ctrl_state.q[2] =   att->orientation.z;
  _ctrl_state.q[3] =   att->orientation.w;
  ROS_INFO("Quaternion x:%f, w:%f, z:%f, w:%f", att->orientation.x, att->orientation.y, att->orientation.z,
           att->orientation.w);
  _sensor_gyro.x = att->angular_velocity.x;
  _sensor_gyro.y = att->angular_velocity.y;
  _sensor_gyro.z = att->angular_velocity.z;

}



void ControlUWSim::spin_controller() {
  static double last_run = 0;
  double dt = ros::Time::now().toSec() - last_run ; //TODO: compute dt
  last_run = ros::Time::now().toSec();

  if (_v_control_mode.flag_control_attitude_enabled) {

    //normal attitude control
    control_attitude(dt);

    /* publish attitude rates setpoint */
    _v_rates_sp.roll = _rates_sp(0);
    _v_rates_sp.pitch = _rates_sp(1);
    _v_rates_sp.yaw = _rates_sp(2);
    _v_rates_sp.thrust = _thrust_sp;


  } else {
    /* attitude controller disabled, poll rates setpoint topic */
    if (_v_control_mode.flag_control_manual_enabled) {
      /* manual rates control - ACRO mode */
      /** replace emult simple multiplication: Eigen **/
      _rates_sp << _manual_control_sp.y, -_manual_control_sp.x, _manual_control_sp.r;
      _rates_sp = _rates_sp.cwiseProduct(_params.acro_rate_max);

      if (_thrust_sp >= 0.0){
        _thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_DOLPHIN);
      }
      else if (_thrust_sp < 0.0)
      {
        _thrust_sp = math::max(_manual_control_sp.z, -MANUAL_THROTTLE_MAX_DOLPHIN);
      }

      /* publish attitude rates setpoint TODO: Is this needed for sim? */
      _v_rates_sp.roll = _rates_sp(0);
      _v_rates_sp.pitch = _rates_sp(1);
      _v_rates_sp.yaw = _rates_sp(2);
      _v_rates_sp.thrust = _thrust_sp;
    }
  }

  /* should be enabled by default if controlling vehicle*/
  if (_v_control_mode.flag_control_rates_enabled) {

    control_attitude_rates(dt);

    /* Now we mix */
    mix_control_output(_att_control, _v_rates_sp.thrust, _mixed_att_control);

    /* publish actuator controls */
    _actuators.control[0] = (PX4_ISFINITE(_mixed_att_control(0))) ? _mixed_att_control(0) : 0.0f;
    _actuators.control[1] = (PX4_ISFINITE(_mixed_att_control(1))) ? _mixed_att_control(1) : 0.0f;
    _actuators.control[2] = (PX4_ISFINITE(_mixed_att_control(2))) ? _mixed_att_control(2) : 0.0f;
    _actuators.control[3] = (PX4_ISFINITE(_mixed_att_control(3))) ? _mixed_att_control(3) : 0.0f;

    _controller_status.roll_rate_integ = _rates_int(0);
    _controller_status.pitch_rate_integ = _rates_int(1);
    _controller_status.yaw_rate_integ = _rates_int(2);
  }

  /** convert _actuators to ROS format and push **/
  std_msgs::Float64MultiArray msg;

  msg.data.resize(4);

//
  msg.data[0] = _actuators.control[0] * 800 + 1100;
  msg.data[1] = _actuators.control[1] * 800 + 1100;
  msg.data[2] = _actuators.control[2] * 800 + 1100;
  msg.data[3] = _actuators.control[3] * 800 + 1100;

  mixed_pwm_pub_.publish(msg);

}

/** Dolphin Attitude Controller
 * Input:
 * Output:
 * **/

void ControlUWSim::control_attitude(double dt) {

  _thrust_sp = _v_att_sp.thrust;

  /* construct attitude setpoint rotation matrix */
  Eigen::Quaternionf q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
  Eigen::Matrix3f R_sp = q_sp.matrix();

  /* get current rotation matrix from control state quaternions */
  Eigen::Quaternionf q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
  Eigen::Matrix3f R = q_att.matrix();

  /* all input data is ready, run controller itself */

  /* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */

  Eigen::Vector3f R_x(R(0, 0), R(1, 0), R(2, 0));
  Eigen::Vector3f R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));

  /* axis and sin(angle) of desired rotation */

  Eigen::Vector3f eMod = mod(R_x, R_sp_x);
  Eigen::Vector3f e_R = R.transpose() * eMod; //e_R is vector between current and desired

  /* calculate angle error */
  float e_R_x_sin = e_R.norm(); // Vector Geometric length not array length. With sin(angle) = Vector2-Vector1
  float e_R_x_cos = R_x.transpose() * R_sp_x; // Dot Product A . B = |A||B|cos(angle)

  /* calculate weight for roll control TODO: Understand this, also, do I need to change it to roll? */
  float roll_w = R_sp(0, 0) * R_sp(0, 0);

  /* The rotation will start with
   * calculate rotation matrix after pitch/yaw only rotation */
  Eigen::Matrix3f R_py;

  /* If the sine is greater than 0, this means there is some pitch/yaw rotation */
  if (e_R_x_sin > 0.0f) {

    /* get axis-angle representation */
    float e_R_x_angle = atan2f(e_R_x_sin, e_R_x_cos); // angle between current and sp thrust vectors
    Eigen::Vector3f e_R_x_axis = e_R / e_R_x_sin; // Rotation Axis

    e_R = e_R_x_angle * e_R_x_axis ; //TODO: I don't get this! We find eRx_axis by diving by sin(alpha) then get it
    // again by multiply it eRx_axis with alpha? Why is that? Perhaps because this time it carries the sign?

    /* e_R_x_axis axis is essentially the rotation axis, which sould be normal to Rsp_x and Rx */
    /* cross product matrix for e_R_axis */
    Eigen::Matrix3f e_R_cp;
    e_R_cp.setZero();
    e_R_cp(0, 1) = -e_R_x_axis(2);
    e_R_cp(0, 2) = e_R_x_axis(1);
    e_R_cp(1, 0) = e_R_x_axis(2);
    e_R_cp(1, 2) = -e_R_x_axis(0);
    e_R_cp(2, 0) = -e_R_x_axis(1);
    e_R_cp(2, 1) = e_R_x_axis(0);

    /* rotation matrix for pitch/yaw only rotation */
    R_py = R * (_I + e_R_x_sin * e_R_cp  + e_R_cp * e_R_cp * (1.0f - e_R_x_cos)); // Rodrigues' formula

  } else {
    /* zero pitch/yaw rotation */
    R_py = R;
  }

  /* R_py and R_sp has the same X axis, calculate roll error */
  Eigen::Vector3f R_sp_y(R_sp(0, 1), R_sp(1, 1), R_sp(2, 1));
  Eigen::Vector3f R_py_y(R_py(0, 1), R_py(1, 1), R_py(2, 1));

  Eigen::Vector3f rmodx = mod(R_py_y, R_sp_y) ;
  e_R(0) = atan2f(rmodx.transpose() * R_sp_x, R_py_y.transpose() * R_sp_y) * roll_w;

  if (e_R_x_cos < 0.0f) {
    /* for large thrust vector rotations use another rotation method:
     * calculate angle and axis for R -> R_sp rotation directly */

    Eigen::Quaternionf q_error;
    q_error = (R.transpose() * R_sp);
    // TODO: Fix. Need imaginary

//    Eigen::Vector3f e_R_d = q_error(0) >= 0.0f ? q_error.imag() * 2.0f : -q_error.imag() * 2.0f;
    Eigen::Vector3f e_R_d;
    /* use fusion of Z axis based rotation and direct rotation */
    float direct_w = e_R_x_cos * e_R_x_cos * roll_w;
    e_R = (1.0f - direct_w) * e_R  + direct_w * e_R_d;

  }

  /* calculate angular rates setpoint */
  _rates_sp = _params.att_p.cwiseProduct(e_R); // Multiply by p gain, no I nor D for angular error

  /* limit rates */
  for (int i = 0; i < 3; i++) {
    if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
        !_v_control_mode.flag_control_manual_enabled) {
      _rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));

    } else {
      _rates_sp(i) = math::constrain(_rates_sp(i), -_params.dp_rate_max(i), _params.dp_rate_max(i));
    }
  }

  /* feed forward roll setpoint rate */
  _rates_sp(2) += _v_att_sp.roll_sp_move_rate * roll_w * _params.roll_ff;

}


void ControlUWSim::control_attitude_rates(double dt) {
  /* reset integral if disarmed */
  if (!_armed.armed) {
    _rates_int.setZero();
  }

  /* get transformation matrix from sensor/board to body frame TODO: fix */
  get_rot_matrix((enum Rotation) _params.board_rotation, _board_rotation);

  /* fine tune the rotation */
  Eigen::Matrix3f board_rotation_offset;

  board_rotation_offset = euler2Quaternion(M_DEG_TO_RAD_F * _params.board_offset[0],
                                           M_DEG_TO_RAD_F * _params.board_offset[1],
                                           M_DEG_TO_RAD_F * _params.board_offset[2]);
  _board_rotation = board_rotation_offset * _board_rotation;

  // get the raw gyro data and correct for thermal errors
  Eigen::Vector3f rates;

  rates.setZero();
  rates(0) = _sensor_gyro.x;
  rates(1) = _sensor_gyro.y;
  rates(2) = _sensor_gyro.z;


  // rotate corrected measurements from sensor to body frame
  rates = _board_rotation * rates;

  // correct for in-run bias errors TODO: Add bias
//  rates(0) -= _ctrl_state.roll_rate_bias;
//  rates(1) -= _ctrl_state.pitch_rate_bias;
//  rates(2) -= _ctrl_state.yaw_rate_bias;


  /** TODO: fix and bring back PID attentuation. For now don't scale p and d rates **/
  //  Eigen::Vector3f rates_p_scaled = _params.rate_p.cwiseProduct(pid_attenuations(_params.tpa_breakpoint_p, _params
  //          .tpa_rate_p));
  //  Eigen::Vector3f  rates_d_scaled = _params.rate_d.cwiseProduct(pid_attenuations(_params.tpa_breakpoint_d, _params
  //          .tpa_rate_d));
  Eigen::Vector3f rates_p_scaled = _params.rate_p;
  Eigen::Vector3f rates_d_scaled = _params.rate_d;

  /* angular rates error */
  Eigen::Vector3f  rates_err = _rates_sp - rates;
  ROS_INFO("P Rates %f, %f, %f", _params.rate_p(0), _params.rate_p(1), _params.rate_p(2));
  ROS_INFO("Rates SP %f, %f, %f", _rates_sp(1), _rates_sp(1), _rates_sp(2));
  ROS_INFO("Rates error %f, %f, %f", rates_err(0), rates_err(1), rates_err(2));
//  PX4_INFO("Rates error - r ctlr: %f, %f, %f", (double) rates_err(0), (double) rates_err(1), (double) rates_err(2));
  ROS_INFO("dt: %f", dt);
  /* PID Att Control */
  _att_control = rates_p_scaled.cwiseProduct(rates_err);// +
                 // _rates_int +
                 // rates_d_scaled.cwiseProduct(_rates_prev - rates) / dt +
                 //_params.rate_ff.cwiseProduct(_rates_sp);
  ROS_INFO("att control: %f, %f, %f", _att_control(0), _att_control(1), _att_control(2));
  _rates_sp_prev = _rates_sp;
  _rates_prev = rates;

  /* update integral only if motors are providing enough thrust to be effective  */
  float epsilon = 0.09; // TODO: change to MACRO instead of min_takeoff_thrust
  if (std::abs(_thrust_sp) > epsilon ||  std::abs(_att_control(0)) > epsilon
      ||  std::abs(_att_control(1)) > epsilon ||  std::abs(_att_control(2)) > epsilon){

    for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
      // Check for positive control saturation
      bool positive_saturation =
              ((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
              ((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
              ((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

      // Check for negative control saturation
      bool negative_saturation =
              ((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
              ((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
              ((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

      // prevent further positive control saturation
      if (positive_saturation) {
        rates_err(i) = math::min(rates_err(i), 0.0f);

      }

      // prevent further negative control saturation
      if (negative_saturation) {
        rates_err(i) = math::max(rates_err(i), 0.0f);

      }

      // Perform the integration using a first order method and do not propaate the result if out of range or invalid
      float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;
//      ROS_INFO("dt: %f, rate_i %f, p_sat %f, n_sat %f", (double) dt, (double) rate_i, (double) positive_saturation,
//               (double) negative_saturation);
      if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
        _rates_int(i) = rate_i;
      }
    }
  }

  /* explicitly limit the integrator state */
  for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
    _rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));
  }


  std_msgs::Float64MultiArray att_rates_pid_rates_msg;
  att_rates_pid_rates_msg.data.resize(9);
  att_rates_pid_rates_msg.data[0] = rates_p_scaled(0);
  att_rates_pid_rates_msg.data[1] = rates_p_scaled(1);
  att_rates_pid_rates_msg.data[2] = rates_p_scaled(2);
  att_rates_pid_rates_msg.data[3] = _rates_int(0);
  att_rates_pid_rates_msg.data[4] = _rates_int(1);
  att_rates_pid_rates_msg.data[5] = _rates_int(2);
  att_rates_pid_rates_msg.data[6] = rates_d_scaled(0);
  att_rates_pid_rates_msg.data[7] = rates_d_scaled(1);
  att_rates_pid_rates_msg.data[8] = rates_d_scaled(2);
  att_rates_pid_rates_pub_.publish(att_rates_pid_rates_msg);

}

/** Dolphin Mixer
 * Input: att_control_
 * Output: mixed_att_output_**/
void
ControlUWSim::mix_control_output(Eigen::Vector3f &att_control, float thrust_sp, Eigen::Vector4f &mixed_att_control) {

  /***
 * Mixing strategy: adopted from mixer_multirotor.cpp
 * 1) Mix pitch, yaw and thrust without roll. And calculate max and min outputs
 * 2) Shift all outputs to minimize out of range violations, min or max. If not enough room to shift all outputs, then scale back pitch/yaw.
 * 3) If there is violation on both upper and lower end then shift such that the violation is equal on both sides.
 * 4) Mix in roll and see if it leads to limit violation, scale back output to allow for roll.
 * 5) Scale all output to range [-1, 1]
 * */

  float roll = math::constrain((float)att_control(0), -1.0f, 1.0f);
  float pitch = math::constrain((float)att_control(1), -1.0f, 1.0f);
  float yaw = math::constrain((float)att_control(2), -1.0f, 1.0f);
  float thrust = math::constrain(thrust_sp, -1.0f, 1.0f);
  float min_out = 1.0f;
  float max_out = -1.0f;

  /*** TODO: Understand how to translate the code to scale to forward and reverse motion.
   * I think the attitude is independent and will scale just fine with motor reverses, well assuming bidirectional
   * equality in thrust power per rotor. But here in att control I should just receive thrust command as a range from
   * -1 to 1 TODO: scale the thrust input early on to match -1 to 1 for bidirectional thrust.
   * */

  // thrust boost parameters AA: This is to limit the max increase/decrease of thrust to account for saturation
  float thrust_increase_factor = 1.5f;
  float thrust_decrease_factor = 0.75f;

  float outputs[4];

  /*** perform initial mix pass yielding unbounded outputs, ignore roll
  */
  for (unsigned i = 0; i < _rotor_count; i++) {
    float out = pitch * _rotors[i].pitch_scale +
                yaw * _rotors[i].yaw_scale +
                thrust * _rotors[i].thrust_scale;

    out *= _rotors[i].out_scale;

    /* calculate min and max output values AA: is in lowest and highest motors outputs */
    if (out < min_out) {
      min_out = out;
    }

    if (out > max_out) {
      max_out = out;
    }
    outputs[i] = out;
  }

  float boost = 0.0f;              // value added to demanded thrust (can also be negative)
  float pitch_yaw_scale = 1.0f;    // scale for demanded pitch and yaw
  float low_bound = -1.0f;
  float upp_bound = 1.0f;


  /*** Now we check if the outputs violate the bounds */
  // TODO review the math
  /* If things are fine - for completeness sake */
  if (max_out < upp_bound && min_out > low_bound) {
    // Keep calm and move on
  }
    // If min is out of bound
  else if (min_out < low_bound && max_out < upp_bound) {

    // In this case we need to increase thrust to bring motors within bound.
    float max_thrust_diff = (float) fabs(thrust * thrust_increase_factor - thrust);

    // if amount out of bound is less than gap between max and upper bound - shift up to make min = lower_bound
    if (-(min_out - low_bound) <= (upp_bound - max_out)) {

      if (max_thrust_diff >= -(min_out - low_bound)) {
        boost = -(min_out - low_bound);
      } else {

        boost = max_thrust_diff;
        pitch_yaw_scale = ((thrust + boost + 1) / (thrust - min_out));
      }
    }
      // shift max increase possible and scale back pitch_yaw
    else {
      boost = math::constrain(-(min_out - low_bound) - (1.0f - max_out) / 2.0f, 0.0f, max_thrust_diff); //TODO: FIX
//      ROS_INFO("Shift back pitch_yaw Boost Value Positive: %f", (double) boost);
      pitch_yaw_scale = ((thrust + boost + 1) / (thrust - min_out));
    }

  }
    // if max is out of bound
  else if (max_out > upp_bound && min_out > low_bound) {

    float max_thrust_diff = (float) fabs(thrust - thrust_decrease_factor * thrust);

    // if amount out of bound is less than gap between min and lower bound - shift down to make max = upper_bound
    if ((max_out - upp_bound) <= -(low_bound - min_out)) {

      if (max_thrust_diff >= (max_out - upp_bound)) {
        boost = -(max_out - upp_bound);
      } else {
        boost = -max_thrust_diff;
        pitch_yaw_scale = (1 - (thrust + boost)) / (max_out - thrust);
      }
    }
      // shift max decrease possible and scale back pitch_yaw
    else {
      boost = math::constrain(-(max_out - 1.0f - min_out) / 2.0f, (float) -max_thrust_diff, 0.0f);
      pitch_yaw_scale = (1 - (thrust + boost)) / (max_out - thrust);
    }
  }
    // if both are out of bound
  else if (max_out > upp_bound && min_out < low_bound) {
    // Scale back so that both violations are equal
    boost = math::constrain(-(max_out - 1.0f + min_out) / 2.0f, (float) -fabs(thrust_decrease_factor * thrust - thrust),
                            (float) fabs(thrust_increase_factor * thrust - thrust));
    pitch_yaw_scale = (thrust + boost) / (thrust - (min_out - low_bound));
  } else {
    // I should never get here!
  }

//  PX4_INFO("New Roll Value %f", (double) roll);
  float thrust_reduction = 0.0f;
  float thrust_increase = 0.0f;
  float roll_scale_2 = 1.0f;
  float pitch_yaw_mix[_rotor_count] = {0.0f, 0.0f, 0.0f, 0.0f};
  /*** Mix now with boost, pitch_yaw scale and roll */
  for (unsigned i = 0; i < _rotor_count; i++) {
    pitch_yaw_mix[i] = (pitch * _rotors[i].pitch_scale + yaw * _rotors[i].yaw_scale) * pitch_yaw_scale;
    float out = pitch_yaw_mix[i] +
                roll * _rotors[i].roll_scale +
                (thrust + thrust_increase - thrust_reduction) + boost;
    out *= _rotors[i].out_scale;

    if (thrust >= 0.0f) {
      if (out > 1.0f) {
        // Thrust Positive and Output with roll exceeds upper bound: reduce thrust and scale back roll
        // Max prop reduction
        float prop_reduction = fminf(0.15f, out - 1.0f);
        thrust_reduction = fmaxf(thrust_reduction, prop_reduction);
        // roll scaled back s.t out = 1.0f TODO: Change this, I need a function to scale back roll, not to recalculate it.
        roll_scale_2 =
                (1.0f - (pitch_yaw_mix[i] + (thrust - thrust_reduction) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("M%i +Thrust +Out Roll Scale 2: %f", i, (double) roll_scale_2);
      } else if (out < -1.0f) {
        // Roll scaled back s.t. out = -1.0f
        roll_scale_2 =
                (-1.0f - (pitch_yaw_mix[i] + (thrust - thrust_reduction) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("+Thrust -Out Roll Scale 2: %f", (double) roll_scale_2);
      }
    } else if (thrust < 0.0f) {
      if (out > 1.0f) {
        // Scale back roll
        roll_scale_2 =
                (1.0f - (pitch_yaw_mix[i] + (thrust + thrust_increase) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("-Thrust +Out Roll Scale 2: %f", (double) roll_scale_2);

      } else if (out < -1.0f) {
        // Thrust negative and output with roll violates lower bound: increase thrust and scale back roll 50/50
        float prop_increase = fminf(0.15f, -(out + 1.0f));
        thrust_increase = fmaxf(thrust_increase, prop_increase);
        // roll scaled back s.t out = 1.0f
        roll_scale_2 =
                (-1.0f - (pitch_yaw_mix[i] + (thrust + thrust_increase) + boost)) / (roll * _rotors[i].roll_scale);
//        PX4_INFO("-Thrust -Out Roll Scale 2: %f", (double) roll_scale_2);
      }
    }

    roll = roll * roll_scale_2;
  }

  // Apply collective thrust reduction/increase (one shall be zero), the maximum for one prop
  thrust = thrust - thrust_reduction + thrust_increase;

  // add roll and scale outputs to range idle_speed...1
  for (unsigned i = 0; i < _rotor_count; i++) {
    outputs[i] = pitch_yaw_mix[i] +
                 roll * _rotors[i].roll_scale * roll_scale_2 +
                 thrust + boost;

    /*
     * TODO: After evaluating my own thrusters, change this to suit model better. for now keep as is.
     * TODO: Confirm where thrust_factor is set, if it doesn't cascade to here, then just set it here or make a
     * module param.
     *
      implement simple model for static relationship between applied motor pwm and motor thrust
      model: thrust = (1 - _thrust_factor) * PWM + _thrust_factor * PWM^2
      this model assumes normalized input / output in the range [0,1] so this is the right place
      to do it as at this stage the outputs are in that range.
      // TODO: This model needs to change to reflect the face that I'm using a bidrectional thrust. A reverse scaler must be used that is different than the forward one, since non-symmetrical thrust
      // TODO: I need to split this, if an output (+) use one equation, if (-) use equation for reverse thrust and
      add sign.
     */
    auto _thrust_factor = 1.0f;
    auto _idle_speed = .1f;

    if (_thrust_factor > 0.0f) {
      float _output = outputs[i];
      if(_output > 0.0f){
        _output = -(1.0f - _thrust_factor) /
                  (2.0f * _thrust_factor) + sqrtf((1.0f - _thrust_factor) *
                                                  (1.0f - _thrust_factor) /
                                                  (4.0f * _thrust_factor * _thrust_factor) + (_output / _thrust_factor));
        _output = math::constrain(_idle_speed + (_output * (1.0f - _idle_speed)), 0.0f, 1.0f);
        outputs[i] = _output;
      }
      else if (_output < 0.0f){
        _output = - _output; // Work with positive
        _output = -(1.0f - _thrust_factor) /
                  (2.0f * _thrust_factor) + sqrtf((1.0f - _thrust_factor) *
                                                  (1.0f - _thrust_factor) /
                                                  (4.0f * _thrust_factor * _thrust_factor) + (_output / _thrust_factor));
        _output = math::constrain(_idle_speed + (_output * (1.0f - _idle_speed)), 0.0f, 1.0f);
        outputs[i] = - _output; // Bring back the sign
      }

    }


  }


  /* TODO: Incorporate slew rate limiting
   * slew rate limiting and saturation checking */
//  for (unsigned i = 0; i < _rotor_count; i++) {
//    bool clipping_high = false;
//    bool clipping_low = false;
//
//    // check for saturation against static limits
//    if (outputs[i] > 0.99f) {
//      clipping_high = true;
//
//    } else if (outputs[i] < _idle_speed + 0.01f) {
//      clipping_low = true;
//
//    }
//
//    // check for saturation against slew rate limits
//    if (_delta_out_max > 0.0f) {
//      float delta_out = outputs[i] - _outputs_prev[i];
//
//      if (delta_out > _delta_out_max) {
//        outputs[i] = _outputs_prev[i] + _delta_out_max;
//        clipping_high = true;
//
//      } else if (delta_out < -_delta_out_max) {
//        outputs[i] = _outputs_prev[i] - _delta_out_max;
//        clipping_low = true;
//
//      }
//    }
//
//    _outputs_prev[i] = outputs[i];


  /* Copy outputs to premixed att control output */
  mixed_att_control(0) = (outputs[0] + 1.0f) / 2.0f;
  mixed_att_control(1) = (outputs[1] + 1.0f) / 2.0f;
  mixed_att_control(2) = (outputs[2] + 1.0f) / 2.0f;
  mixed_att_control(3) = (outputs[3] + 1.0f) / 2.0f;
//  PX4_INFO("Mixed Output: %f, %f, %f, %f", (double) mixed_att_control(0),
//           (double) mixed_att_control(1),
//           (double) mixed_att_control(2),
//           (double) mixed_att_control(3));

}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Eigen::Vector3f ControlUWSim::pid_attenuations(float tpa_breakpoint, float tpa_rate) {
  /* throttle pid attenuation factor TODO: Understand and fix this to my use */
  float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
  tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

  Eigen::Vector3f pidAttenuationPerAxis;
  pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
  pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
  pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

  return pidAttenuationPerAxis;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_uwsim");
  ControlUWSim uwsim_control;
  int rate = 400; // 400 Hz
  ros::Rate r(rate);
  while(ros::ok()){
    ros::spinOnce();
    /** Call my control functions here **/
    /** TODO: Measure execution time of the controller and publish */
    uwsim_control.spin_controller();
    /** This will sleep for the remaining period to achieve the required rate, presumably per ROS's docs TODO: Verify **/
    r.sleep();
  }

  return 0;
}


