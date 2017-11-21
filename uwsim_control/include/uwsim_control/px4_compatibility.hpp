/****************************************************************************
 *
 *   Copyright (c) 2017 Ali AlSaibie. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file 
 * 
 *
 * @author Ali AlSaibie
 */
#define M_E_F			2.71828183f
#define M_LOG2E_F		1.44269504f
#define M_LOG10E_F		0.43429448f
#define M_LN2_F			0.69314718f
#define M_LN10_F		2.30258509f
#define M_PI_F			3.14159265f
#define M_TWOPI_F		6.28318531f
#define M_PI_2_F		1.57079632f
#define M_PI_4_F		0.78539816f
#define M_3PI_4_F		2.35619449f
#define M_SQRTPI_F		1.77245385f
#define M_1_PI_F		0.31830989f
#define M_2_PI_F		0.63661977f
#define M_2_SQRTPI_F		1.12837917f
#define M_DEG_TO_RAD_F		0.0174532925f
#define M_RAD_TO_DEG_F		57.2957795f
#define M_SQRT2_F		1.41421356f
#define M_SQRT1_2_F		0.70710678f
#define M_LN2LO_F		1.90821484E-10f
#define M_LN2HI_F		0.69314718f
#define M_SQRT3_F		1.73205081f
#define M_IVLN10_F		0.43429448f	// 1 / log(10)
#define M_LOG2_E_F		0.69314718f
#define M_INVLN2_F		1.44269504f	// 1 / log(2)

#define M_DEG_TO_RAD 		0.017453292519943295
#define M_RAD_TO_DEG 		57.295779513082323
namespace math {
    template<typename _Tp>
    inline constexpr const _Tp &min(const _Tp &a, const _Tp &b)
    {
      return (a < b) ? a : b;
    }

    template<typename _Tp>
    inline constexpr const _Tp &max(const _Tp &a, const _Tp &b)
    {
      return (a > b) ? a : b;
    }
    template<typename _Tp>
    inline constexpr const _Tp &constrain(const _Tp &val, const _Tp &min_val, const _Tp &max_val)
    {
      return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
    }

    float radians(float degrees)
    {
      return (degrees / 180.0f) * M_PI_F;
    }

    double radians(double degrees)
    {
      return (degrees / 180.0) * M_PI;
    }

    float degrees(float radians)
    {
      return (radians / M_PI_F) * 180.0f;
    }

    double degrees(double radians)
    {
      return (radians / M_PI) * 180.0;
    }
}

Eigen::Quaternionf
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
  return q;
}

Eigen::Matrix3f euler2Matrix(const double roll,
                             const double pitch,
                             const double yaw){
  Eigen::Quaternionf q;
  q = euler2Quaternion(roll, pitch, yaw);
  return q.matrix();
}

Eigen::Vector3f mod(Eigen::Vector3f r1 , Eigen::Vector3f r2){

  Eigen::Vector3f r;

  for (int k = 0; k<r1.size(); k++){
    r(k) = fmod(r1(k),r2(k));

  }
  return r;
}

#pragma once
enum Rotation {
    ROTATION_NONE                = 0,
    ROTATION_YAW_45              = 1,
    ROTATION_YAW_90              = 2,
    ROTATION_YAW_135             = 3,
    ROTATION_YAW_180             = 4,
    ROTATION_YAW_225             = 5,
    ROTATION_YAW_270             = 6,
    ROTATION_YAW_315             = 7,
    ROTATION_ROLL_180            = 8,
    ROTATION_ROLL_180_YAW_45     = 9,
    ROTATION_ROLL_180_YAW_90     = 10,
    ROTATION_ROLL_180_YAW_135    = 11,
    ROTATION_PITCH_180           = 12,
    ROTATION_ROLL_180_YAW_225    = 13,
    ROTATION_ROLL_180_YAW_270    = 14,
    ROTATION_ROLL_180_YAW_315    = 15,
    ROTATION_ROLL_90             = 16,
    ROTATION_ROLL_90_YAW_45      = 17,
    ROTATION_ROLL_90_YAW_90      = 18,
    ROTATION_ROLL_90_YAW_135     = 19,
    ROTATION_ROLL_270            = 20,
    ROTATION_ROLL_270_YAW_45     = 21,
    ROTATION_ROLL_270_YAW_90     = 22,
    ROTATION_ROLL_270_YAW_135    = 23,
    ROTATION_PITCH_90            = 24,
    ROTATION_PITCH_270           = 25,
    ROTATION_ROLL_270_YAW_270    = 26,
    ROTATION_ROLL_180_PITCH_270  = 27,
    ROTATION_PITCH_90_YAW_180    = 28,
    ROTATION_PITCH_90_ROLL_90	 = 29,
    ROTATION_YAW_293_PITCH_68_ROLL_90 = 30,
    ROTATION_PITCH_90_ROLL_270	 = 31,
    ROTATION_PITCH_9_YAW_180 = 32,
    ROTATION_MAX
};
typedef struct {
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
} rot_lookup_t;

const rot_lookup_t rot_lookup[] = {
        {  0,   0,   0 },
        {  0,   0,  45 },
        {  0,   0,  90 },
        {  0,   0, 135 },
        {  0,   0, 180 },
        {  0,   0, 225 },
        {  0,   0, 270 },
        {  0,   0, 315 },
        {180,   0,   0 },
        {180,   0,  45 },
        {180,   0,  90 },
        {180,   0, 135 },
        {  0, 180,   0 },
        {180,   0, 225 },
        {180,   0, 270 },
        {180,   0, 315 },
        { 90,   0,   0 },
        { 90,   0,  45 },
        { 90,   0,  90 },
        { 90,   0, 135 },
        {270,   0,   0 },
        {270,   0,  45 },
        {270,   0,  90 },
        {270,   0, 135 },
        {  0,  90,   0 },
        {  0, 270,   0 },
        {270,   0, 270 },
        {180, 270,   0 },
        {  0,  90, 180 },
        { 90,  90,   0 },
        { 90,  68, 293 },
        {270,  90,   0 },
        {  0,   9, 180 },
};

void
get_rot_matrix(enum Rotation rot, Eigen::Matrix3f &rot_matrix)
{
  float roll  = M_DEG_TO_RAD_F * (float)rot_lookup[rot].roll;
  float pitch = M_DEG_TO_RAD_F * (float)rot_lookup[rot].pitch;
  float yaw   = M_DEG_TO_RAD_F * (float)rot_lookup[rot].yaw;

  rot_matrix = euler2Matrix(roll, pitch, yaw);
}




struct vehicle_attitude_setpoint_s {
    uint64_t timestamp; // required for logger
    float roll_body;
    float pitch_body;
    float yaw_body;
//    float yaw_sp_move_rate;
    float roll_sp_move_rate;
    float q_d[4];
    float thrust;
    float landing_gear;
    bool q_d_valid;
    bool roll_reset_integral;
    bool pitch_reset_integral;
    bool yaw_reset_integral;
    bool fw_control_yaw;
    bool disable_mc_yaw_control;
    bool apply_flaps;
    uint8_t _padding0[1]; // required for logger
};

struct control_state_s {
    uint64_t timestamp; // required for logger
    float x_acc;
    float y_acc;
    float z_acc;
    float x_vel;
    float y_vel;
    float z_vel;
    float x_pos;
    float y_pos;
    float z_pos;
    float airspeed;
    float vel_variance[3];
    float pos_variance[3];
    float q[4];
    float delta_q_reset[4];
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float horz_acc_mag;
    float roll_rate_bias;
    float pitch_rate_bias;
    float yaw_rate_bias;
    bool airspeed_valid;
    uint8_t quat_reset_counter;
    uint8_t _padding0[2]; // required for logger
};


struct vehicle_control_mode_s {
    uint64_t timestamp; // required for logger
    bool flag_armed;
    bool flag_external_manual_override_ok;
    bool flag_system_hil_enabled;
    bool flag_control_manual_enabled;
    bool flag_control_auto_enabled;
    bool flag_control_offboard_enabled;
    bool flag_control_rates_enabled;
    bool flag_control_attitude_enabled;
    bool flag_control_rattitude_enabled;
    bool flag_control_force_enabled;
    bool flag_control_acceleration_enabled;
    bool flag_control_velocity_enabled;
    bool flag_control_position_enabled;
    bool flag_control_altitude_enabled;
    bool flag_control_climb_rate_enabled;
    bool flag_control_termination_enabled;
    bool flag_control_fixed_hdg_enabled;
    uint8_t _padding0[7]; // required for
};

struct vehicle_rates_setpoint_s{
    uint64_t timestamp; // required for logger
    float roll;
    float pitch;
    float yaw;
    float thrust;
};

struct actuator_armed_s {
    uint64_t timestamp; // required for logger
    bool armed;
    bool prearmed;
    bool ready_to_arm;
    bool lockdown;
    bool manual_lockdown;
    bool force_failsafe;
    bool in_esc_calibration_mode;
    bool soft_stop;
};

struct vehicle_status_s {
    uint64_t timestamp; // required for logger
    uint32_t system_id;
    uint32_t component_id;
    uint32_t onboard_control_sensors_present;
    uint32_t onboard_control_sensors_enabled;
    uint32_t onboard_control_sensors_health;
    uint8_t nav_state;
    uint8_t arming_state;
    uint8_t hil_state;
    bool failsafe;
    uint8_t system_type;
    bool is_rotary_wing;
    bool is_vtol;
    bool vtol_fw_permanent_stab;
    bool in_transition_mode;
    bool in_transition_to_fw;
    bool rc_signal_lost;
    uint8_t rc_input_mode;
    bool data_link_lost;
    uint8_t data_link_lost_counter;
    bool engine_failure;
    bool engine_failure_cmd;
    bool mission_failure;
    uint8_t _padding0[3]; // required for logger
};

struct manual_control_setpoint_s {
    uint64_t timestamp; // required for logger
    float x;
    float y;
    float z;
    float r;
    float flaps;
    float aux1;
    float aux2;
    float aux3;
    float aux4;
    float aux5;
    uint8_t mode_switch;
    uint8_t return_switch;
    uint8_t rattitude_switch;
    uint8_t posctl_switch;
    uint8_t loiter_switch;
    uint8_t acro_switch;
    uint8_t offboard_switch;
    uint8_t kill_switch;
    uint8_t arm_switch;
    uint8_t transition_switch;
    uint8_t gear_switch;
    int8_t mode_slot;
    uint8_t data_source;
    uint8_t stab_switch;
    uint8_t man_switch;
    uint8_t _padding0[1]; // required for logger
};

struct dp_att_ctrl_status_s {
    uint64_t timestamp; // required for logger
    float roll_rate_integ;
    float pitch_rate_integ;
    float yaw_rate_integ;
    uint8_t _padding0[4]; // required for logger
};

struct actuator_controls_s {
    uint64_t timestamp; // required for logger
    uint64_t timestamp_sample;
    float control[8];
};

struct sensor_gyro_s {
    uint64_t timestamp; // required for logger
    uint64_t integral_dt;
    uint64_t error_count;
    float x;
    float y;
    float z;
    float x_integral;
    float y_integral;
    float z_integral;
    float temperature;
    float range_rad_s;
    float scaling;
    uint32_t device_id;
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t temperature_raw;
};