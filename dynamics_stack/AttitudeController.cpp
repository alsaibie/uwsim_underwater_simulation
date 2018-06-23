//
// Created by alsaibie on 5/18/18.
//

#include "AttitudeController.hpp"
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
//TODO: remove later
#include <iostream>
#define TPA_RATE_LOWER_LIMIT 0.05f
#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

using namespace AController;
AttitudeController::AttitudeController()
{

    /* Zero States, Setpoints and Outputs */
    _state.att.q.zero();
    _state.att.q(0) = 1.0;
    _state.rates.rpy.zero();
    _state.power.bat_scale_mA = 0;
    _state.power.bat_scale_en = false;
    _state.power.bat_V_nom = 8.4;
    _att_sp.q.zero();
    _att_sp.q(0) = 1.0;
    _rates_sp.rpy.zero();
    _rates_sp.roll_move_rate = 0;
    _rates_sp.thrust = 0;
    _tau_sp.zero();
    _compensated_V_sp.zero();
    _mixed_u.actuator.zero();

    for (int k = 0; k < _rotor_count; k++) {
        _rotors[k].roll_scale = _dolphin_x_table[k][0];
        _rotors[k].pitch_scale = _dolphin_x_table[k][1];
        _rotors[k].yaw_scale = _dolphin_x_table[k][2];
        _rotors[k].thrust_scale = _dolphin_x_table[k][3];
        _rotors[k].out_scale = _dolphin_x_table[k][4];
    }
}

void
AttitudeController::resetSetpoints() {

    _rates_sp.rpy.zero();
    _rates_sp.thrust = 0.0f;
    _ctrl_status.rates_int.zero();
    _tau_sp.zero();
    _compensated_V_sp.zero();
    _mixed_u.actuator.zero();
    _state.power.bat_V_nom = 8.4; // TODO: this should come from a param
}

void
AttitudeController::controlAttitude(const float &dt) {

    /* prepare yaw weight from the ratio between roll/pitch and yaw gains */
    Vector3f attitude_gain = _gains.att_p;

    const float pitch_yaw_gain = (attitude_gain(1) + attitude_gain(2)) / 2.f;
    const float roll_w = math::constrain(attitude_gain(2) / pitch_yaw_gain, 0.f, 1.f);
    attitude_gain(0) = pitch_yaw_gain;
    Quatf q(_state.att.q);
    Quatf qd(_att_sp.q);


    /* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
    q.normalize();
    qd.normalize();
    //TODO: q needs time to be valid, can I check validity somewhere before I do any work with it?


    /* calculate reduced desired attitude neglecting vehicle's roll to prioritize pitch and yaw */
    Vector3f e_x = q.dcm_x();
    Vector3f e_x_d = qd.dcm_x();


    Quatf qd_red(e_x, e_x_d);


    if (abs(qd_red(2)) > (1.f - 1e-5f) || abs(qd_red(3)) > (1.f - 1e-5f)) {
        /* In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction */
        qd_red = qd;
    } else {
        /* transform rotation from current to desired thrust vector into a world frame reduced desired attitude */
        qd_red *= q;
    }


    /* mix full and reduced desired attitude */
    Quatf q_mix = qd * qd_red.inversed();
    q_mix *= math::signNoZero(q_mix(0));

    /* catch numerical problems with the domain of acosf and asinf */
    q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
    q_mix(1) = math::constrain(q_mix(1), -1.f, 1.f);
    qd = Quatf(cosf(roll_w * acosf(q_mix(0))), sinf(roll_w * asinf(q_mix(1))), 0, 0) * qd_red;

    /* quaternion attitude control law, qe is rotation from q to qd */
    Quatf qe = q.inversed() * qd;

    /* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
     * also taking care of the antipodal unit quaternion ambiguity */
    Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();
    /* calculate angular rates setpoint */
    _rates_sp.rpy = eq.emult(attitude_gain);

    /* Feed forward the roll setpoint rate. We need to apply the roll rate in the body frame.
     * We infer the body x axis by taking the first column of R.transposed (== q.inversed)
     * because it's the rotation axis for body roll and multiply it by the rate and gain. */
    // TODO: Change this to full FF model based control.
    Vector3f roll_feedforward_rate = q.inversed().dcm_x();
    //TODO: Fix, either param or calculated in pos control
    _rates_sp.roll_move_rate = 0.0;
    roll_feedforward_rate *= _rates_sp.roll_move_rate * _gains.att_ff(0);
    _rates_sp.rpy += roll_feedforward_rate;

    /* limit rates */
    for (int i = 0; i < 3; i++) {
        if (_mode.mode == AController::CONTROL_MODE::Auto){
            _rates_sp.rpy(i) = math::constrain(_rates_sp.rpy(i), -_limits.auto_rate_max(i), _limits.acro_rate_max(i));

        } else {
            _rates_sp.rpy(i) = math::constrain(_rates_sp.rpy(i), -_limits.manual_rate_max(i), _limits.manual_rate_max(i));

        }
    }

}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Vector3f
AttitudeController::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
    /* throttle pid attenuation factor */
    float tpa = 1.0f - tpa_rate * (fabsf(_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
    tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

    Vector3f pidAttenuationPerAxis;
    pidAttenuationPerAxis(AXIS_INDEX_ROLL) = 1.0;
    pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
    pidAttenuationPerAxis(AXIS_INDEX_YAW) = tpa;

    return pidAttenuationPerAxis;
}


void
AttitudeController::controlRates(const float &dt) {


    /* reset integral if disarmed */
    if (!_mode.is_armed) {
        _ctrl_status.rates_int.zero();
    }

    if(_mode.mode == AController::CONTROL_MODE::Acro) {
        matrix::Vector3f acro_rate_sp(
                math::superexpo(_rates_sp.rpy(0), _limits.acro_expo_r, _limits.acro_superexpo_r),
                math::superexpo(_rates_sp.rpy(1), _limits.acro_expo_py, _limits.acro_superexpo_py),
                math::superexpo(_rates_sp.rpy(2), _limits.acro_expo_py, _limits.acro_superexpo_py));
        _rates_sp.rpy = acro_rate_sp.emult(_limits.acro_rate_max);

    }

    /* TPA - Throttle PID Attenuation: Attenuates PID gains beyond a certain throttle to reduce oscillation
     * TODO: Remove for now, revisit if necessary
     * */
//    Vector3f rates_p_scaled = _gains.rate_p.emult(pid_attenuations(_gains._tpa_breakpoint_p, _gains._tpa_rate_p));
//    Vector3f rates_i_scaled = _gains.rate_i.emult(pid_attenuations(_gains._tpa_breakpoint_i, _gains._tpa_rate_i));
//    Vector3f rates_d_scaled = _gains.rate_d.emult(pid_attenuations(_gains._tpa_breakpoint_d, _gains._tpa_rate_d));

    /* angular rates error */
    matrix::Vector3f rates_err = _rates_sp.rpy - _state.rates.rpy;

    // TODO: Add low pass filter later
    /* Apply low-pass filtering to the rates for D-term */
    matrix::Vector3f rates_filtered(
            _filters[0].apply(_state.rates.rpy(0)),
            _filters[1].apply(_state.rates.rpy(1)),
            _filters[2].apply(_state.rates.rpy(2)));

    _tau_sp = _gains.rate_p.emult(rates_err)
            + _ctrl_status.rates_int
//          -  _gains.rate_d.emult(rates_filtered - _ctrl_status.rates_prev_filtered) / dt
            + _gains.rate_d.emult(rates_err) / dt
            + _gains.rate_ff.emult(_rates_sp.rpy);

    _ctrl_status.rates_prev = _state.rates.rpy;
//    _ctrl_status.rates_prev_filtered = rates_filtered;

    _filters[0].reset(_ctrl_status.rates_prev(0));
    _filters[1].reset(_ctrl_status.rates_prev(1));
    _filters[2].reset(_ctrl_status.rates_prev(2));


    /* Limit Tau Rates  */
    for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
        _tau_sp(i) = math::constrain(_tau_sp(i), -_limits.tau_max_sp(i), _limits.tau_max_sp(i));
    }

    for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
        /* Update integral only if some torque above min value and no torque saturation
         * TODO: Change saturation to dynamic looking at motor saturation vs p gain max saturation and clamping */
        
        if (((i == AXIS_INDEX_ROLL && (fabs(_tau_sp(i)) > 0.005)) || // TODO: make this a param
            ((i > AXIS_INDEX_ROLL) && (fabs(_tau_sp(i)) > 0.1))) &&
            _tau_sp(i) > -_limits.tau_max_sp(i) && _tau_sp(i) < _limits.tau_max_sp(i)) {

            float rate_i = _ctrl_status.rates_int(i) + _gains.rate_i(i) + rates_err(i) * dt;

            if (PX4_ISFINITE(rate_i) && rate_i > -_ctrl_status.rate_int_lim(i) && rate_i <
                                                                                  _ctrl_status.rate_int_lim(i)) {
                _ctrl_status.rates_int(i) = rate_i;
            }
        }
        else{

        }
    }

    compensateThrusterDynamics();

    mixOutput();

}


/**
 * Scale throttle (commanded thrust) and torques force setpoints to motor speed sp
 * and then to normalized voltage sp
 * Input: actuator tau vector (_tau_sp)
 * Output: actuator scaled voltage (PWM) setpoint (_compensated_tau_sp)
 * @param dt
 */
void
AttitudeController::compensateThrusterDynamics() {

    float thrust_V_, roll_V_, pitch_V_, yaw_V_;

    /* Convert RPY Torque Sp to motor Hz */
    float roll_T_    = _tau_sp(0) / _rotor_count;
    float pitch_F_   = _tau_sp(1) / ( _rotor_count * _constants.rotor_radius);
    float yaw_F_     = _tau_sp(2) / ( _rotor_count * _constants.rotor_radius);

    float roll_Hz_   = AController::poly_abs_discontinuos(_constants.nTorque_coefficients, roll_T_);
    float pitch_Hz_  = AController::poly_abs_discontinuos(_constants.nThrust_coefficients, pitch_F_);
    float yaw_Hz_    = AController::poly_abs_discontinuos(_constants.nThrust_coefficients, yaw_F_);

//    std::cout << " Roll Hz: " << roll_T_ << " Pitch Hz: " << pitch_F_ << " Yaw T: " << yaw_F_ << std::endl;

    /* Convert RPY Hz to Voltage Sp */
    roll_V_   = AController::poly_abs_discontinuos(_constants.Vn_coefficients, roll_Hz_);
    pitch_V_  = AController::poly_abs_discontinuos(_constants.Vn_coefficients, pitch_Hz_);
    yaw_V_    = AController::poly_abs_discontinuos(_constants.Vn_coefficients, yaw_Hz_);

    /* Linearize Throttle sp */
    thrust_V_ = AController::poly_abs_discontinuos(_constants.Vn_norm_coefficients, _rates_sp.thrust) ;

    /* Normalized by dividing by nominal batt voltage TODO: Check that this logic is correct */

    _compensated_V_sp(0) = roll_V_      / _state.power.bat_V_nom;
    _compensated_V_sp(1) = pitch_V_     / _state.power.bat_V_nom;
    _compensated_V_sp(2) = yaw_V_       / _state.power.bat_V_nom;
    _compensated_V_sp(3) = thrust_V_;
    // TODO: compensate properly for bat voltage

}


void AttitudeController::mixOutput() {

    /***
  * Mixing strategy: adopted from mixer_multirotor.cpp
  * 1) Mix pitch, yaw and thrust without roll. And calculate max and min outputs
  * 2) Shift all outputs to minimize out of range violations, min or max. If not enough room to shift all outputs, then scale back pitch/yaw.
  * 3) If there is violation on both upper and lower end then shift such that the violation is equal on both sides.
  * 4) Mix in roll and see if it leads to limit violation, scale back output to allow for roll.
  * 5) Scale all output to range [-1, 1]
  * */

    float roll = math::constrain(_compensated_V_sp(0), -1.0f, 1.0f);
    float pitch = math::constrain(_compensated_V_sp(1), -1.0f, 1.0f);
    float yaw = math::constrain(_compensated_V_sp(2), -1.0f, 1.0f);
    float thrust = math::constrain(_compensated_V_sp(3), -1.0f, 1.0f);
    float min_out = 1.00000f;
    float max_out = -1.00000f;


    //    PX4_INFO("u Roll %f, Pitch %f, Yaw %f", (double) roll, (double) pitch, (double) yaw);
    /*** TODO: Understand how to translate the code to scale to forward and reverse motion.
     * I think the attitude is independent and will scale just fine with motor reverses, well assuming bidirectional
     * equality in thrust power per rotor. But here in att control I should just receive thrust command as a range from
     * -1 to 1 TODO: scale the thrust input early on to match -1 to 1 for bidirectional thrust.
     * */

    float thrust_increase_factor = 1.25f;
    float thrust_decrease_factor = 0.75f;
    float outputs[4];

    /*** perform initial mix pass yielding unbounded outputs, ignore roll */
    for (unsigned i = 0; i < _rotor_count; i++) {
        float out = pitch * _rotors[i].pitch_scale +
                    yaw * _rotors[i].yaw_scale +
                    thrust * _rotors[i].thrust_scale;

        out *= _rotors[i].out_scale;

        if (out < min_out) { min_out = out; }
        if (out > max_out) { max_out = out; }

        outputs[i] = out;
    }

    float boost = 0.0f;
    float pitch_yaw_scale = 1.0f;
    float low_bound = -1.0f;
    float upp_bound = 1.0f;

    /*
     * Step 1: Shift Output to within bounds
     */
    // TODO review the math
    if (max_out < upp_bound && min_out > low_bound) { /* No further checks */ }

    else if (min_out < low_bound && max_out < upp_bound) {
        /* If min is out of bound, increase thrust to bring motors within bounds */

        float max_thrust_shift = fabs(thrust * (thrust_increase_factor - 1));

        if ((low_bound - min_out) <= (upp_bound - max_out)) {
            /* If lower out of bound amount, is less than the gap between max and upper bound
             * then shift thrust up to make: min = lower_bound
             * */
            if (max_thrust_shift >= (low_bound - min_out)) {
                boost = (low_bound - min_out);

            } else {
                /* If amount out of bounds is greater than max thrust shift, scale back pitch/yaw as well */
                boost = max_thrust_shift;
                pitch_yaw_scale = ((thrust + boost + 1) / (thrust - min_out));
            }
        }
        else {
            /* Upper gap is not big enough, constrain increase */
            boost = math::constrain((low_bound - min_out) - (upp_bound - max_out) / 2.0f, 0.0f, max_thrust_shift); //TODO: FIX
            pitch_yaw_scale = ((thrust + boost + 1) / (thrust - min_out));
        }
    }

    else if (max_out > upp_bound && min_out > low_bound) {
        /* If max is out of bound, decrease thrust to bring motors within bounds */

        float max_thrust_diff = fabs((1 - thrust_decrease_factor) * thrust);

        if ((max_out - upp_bound) <= (min_out - low_bound)) {
            /* If upper out of bound amount, is less than gap between min and lower bound, shift thrust down
             * to make: max = upper bound
             * */
            if (max_thrust_diff >= (max_out - upp_bound)) {
                boost = upp_bound - max_out;

            } else {
                /* If amount out of bounds is greater than max thrust shift, scale back pitch/yaw as well */
                boost = -max_thrust_diff;
                pitch_yaw_scale = (1 - (thrust + boost)) / (max_out - thrust);
            }
        }
        else {
            /* Lower gap is not big enough, constrain decrease */
            boost = math::constrain(-(max_out - 1.0f - min_out) / 2.0f, -max_thrust_diff, 0.0f);
            pitch_yaw_scale = (1 - (thrust + boost)) / (max_out - thrust);
        }
    }

    else if (max_out > upp_bound && min_out < low_bound) {
        /* If out of bounds on both ends constrain output on both ends equally */
        boost = math::constrain(-(max_out - 1.0f + min_out) / 2.0f, -fabs(thrust_decrease_factor * thrust - thrust),
                                fabs(thrust_increase_factor * thrust - thrust));
        pitch_yaw_scale = (thrust + boost) / (thrust - (min_out - low_bound));
    } else {
//        PX4_WARN("Mixing Error");
    }

    /*
     * Step 1: Scale with pitch_yaw, add roll and scale again appropriately
     */
    float thrust_reduction = 0.0f;
    float thrust_increase = 0.0f;
    float roll_scale_2 = 1.0f;
    float pitch_yaw_mix[_rotor_count] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (unsigned i = 0; i < _rotor_count; i++) {
        /* Mix now with boost, pitch_yaw scale and roll */
        pitch_yaw_mix[i] = (pitch * _rotors[i].pitch_scale + yaw * _rotors[i].yaw_scale) * pitch_yaw_scale;
//        PX4_INFO("PitchYaw Mix %f", (double)pitch_yaw_mix[i]);
        float out = pitch_yaw_mix[i] +
                    roll * _rotors[i].roll_scale +
                    (thrust + thrust_increase - thrust_reduction) + boost;
        out *= _rotors[i].out_scale;

        if (thrust >= 0.0f) {
            if (out > upp_bound) {
                /* Thrust with roll (+) > upper bound */
                float prop_reduction = fminf(0.15f, out - upp_bound);
                thrust_reduction = fmaxf(thrust_reduction, prop_reduction);
                /* roll scaled back s.t out = 1.0f  */
                //TODO: Change this, I need a function to scale back roll, not to recalculate it.
                roll_scale_2 =
                        (upp_bound - (pitch_yaw_mix[i] + (thrust - thrust_reduction) + boost)) /
                        (roll * _rotors[i].roll_scale);
            } else if (out < low_bound) {
                /* roll scaled back s.t out = -1.0f  */
                roll_scale_2 =
                        (low_bound - (pitch_yaw_mix[i] + (thrust - thrust_reduction) + boost)) /
                        (roll * _rotors[i].roll_scale);
            }
        } else if (thrust < 0.0f) {
            if (out > upp_bound) {
                roll_scale_2 =
                        (upp_bound - (pitch_yaw_mix[i] + (thrust + thrust_increase) + boost)) /
                        (roll * _rotors[i].roll_scale);

            } else if (out < low_bound) {
                /* Thrust negative and output with roll violates lower bound:
                 * increase thrust and scale back roll 50/50
                 * */
                float prop_increase = fminf(0.15f, -(out + 1.0f));
                thrust_increase = fmaxf(thrust_increase, prop_increase);
                roll_scale_2 =
                        (low_bound- (pitch_yaw_mix[i] + (thrust + thrust_increase) + boost)) /
                        (roll * _rotors[i].roll_scale);
            }
        }

        roll = roll * roll_scale_2;
    }

    /* Apply collective thrust reduction/increase (one shall be zero), the maximum for one prop */
    thrust = thrust - thrust_reduction + thrust_increase;

    /* Add roll and scale outputs */
    for (unsigned i = 0; i < _rotor_count; i++) {
        outputs[i] = pitch_yaw_mix[i] +
                     roll * _rotors[i].roll_scale * roll_scale_2 +
                     thrust + boost;
    }

//    PX4_INFO("Roll %f, Roll %f, Roll %f", (double) roll, (double) pitch, (double) yaw);
    _mixed_u.actuator(0) = outputs[0];
    _mixed_u.actuator(1) = outputs[1];
    _mixed_u.actuator(2) = outputs[2];
    _mixed_u.actuator(3) = outputs[3];
//    _mixed_u.actuator(0) = -0.641797423;
//    _mixed_u.actuator(1) = 0.970197439;
//    _mixed_u.actuator(2) = -0.641797423 ;
//    _mixed_u.actuator(3) = 0.970197439 ;
}












