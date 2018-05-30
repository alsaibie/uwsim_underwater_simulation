//
// Created by alsaibie on 5/18/18.
//

/**
 * @file AttitudeController.hpp
 *
 * @inputs: attitude, rate setpoints
 * @outputs: actuator outputs
 *
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

using namespace matrix;

namespace Controller
{
    struct States {
        struct Attitude {
            Quatf q;
        } att;
        struct Rates {
            Vector3f rpy;
        } rates;
        struct Power{
            bool bat_scale_en;
            float bat_scale_mA;
        } power;
    };

    struct Setpoints {
        struct Attitude {
            Quatf q;
        };
        struct Rates {
            float thrust;
            Vector3f rpy;
            float roll_move_rate;
        };
    };
        struct Outputs {
        Vector<float, 4> actuator;
    };

    struct Gains {
        Vector3f att_p;
        Vector3f att_ff;
        Vector3f rate_p;
        Vector3f rate_i;
        Vector3f rate_d;
        Vector3f rate_ff;
        float _tpa_breakpoint_p;
        float _tpa_breakpoint_i;
        float _tpa_breakpoint_d;
        float _tpa_rate_p;
        float _tpa_rate_i;
        float _tpa_rate_d;

    };


    struct Limits {
        Vector3f manual_rate_max;
        Vector3f auto_rate_max;
        Vector3f acro_rate_max;
        Vector3f rate_int_lim;
        float acro_expo_py;
        float acro_expo_r;
        float acro_superexpo_py;
        float acro_superexpo_r;
    };

    struct Saturation {

    };

    typedef enum {
        Manual = 0,
        Auto,
        Acro
    }CONTROL_MODE;

    struct ControlMode{
        uint8_t mode;
        bool is_armed;
    };

    struct ControllerStatus {
        Vector3f rates_int;
        Vector3f rates_prev;
        Vector3f rates_prev_filtered;
        Vector3f rate_int_lim;
    };
}

class AttitudeController {

public:
    AttitudeController();
    ~AttitudeController() {};

    /* Input Interfaces */
    void updateStates(const Controller::States &state){ _state = state; }
    void updateAttitudeSetpoint(const Controller::Setpoints::Attitude &att_sp){ _att_sp = att_sp;}
    void updateRateSetpoint(const Controller::Setpoints::Rates &rates_sp){ _rates_sp = rates_sp;}
    void updateGains(const Controller::Gains &gains) {_gains = gains;}
    void updateLimits(const Controller::Limits &limits) {_limits = limits; }
//    void updateFilters(const Controller::Filters &filters) {_filters = filters;}
    void updateSaturations(const Controller::Saturation &saturation) {_saturation = saturation;}
    void updateControlMode(const Controller::ControlMode mode){_mode = mode;}
    void setFilter(math::LowPassFilter2p *filt){ _filters = filt; }
    void resetSetpoints();
    /* Controller Executions */
    void controlAttitude(const float &dt); /**/
    void controlRates(const float &dt);

    /* Output Interfaces */
    Controller::Limits           getLimits() {return _limits;}
    Controller::Gains            getGains() {return _gains;}
    Controller::Setpoints::Rates getRateSetpoint() { return _rates_sp; }
    Controller::Outputs getCompensatedControlOutput() { return _compensated_u;}
    Controller::ControllerStatus getControllerStatus() {return _ctrl_status;}

private:

    /* States */
    Controller::States _state {};

    /* Setpoints - Not necessary provided together in the controller configuration */
    Controller::Setpoints::Attitude   _att_sp {};
    Controller::Setpoints::Rates      _rates_sp {};

    /* Control Intermediate Outputs */
    Vector3f         _rates_actuator_u {};
    Vector<float, 4> _mixed_actuator_u {};

    /* Control Gains,  Limits and Saturation */
    Controller::Gains _gains {};
    Controller::Limits _limits {};
    math::LowPassFilter2p *_filters ; // TODO: Share ptr for now, but break dependency for complete abstraction
    Controller::Saturation _saturation {};
    static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
    Controller::ControlMode _mode {};

    /* Control Variables */
    Controller::ControllerStatus _ctrl_status;

    /* Control Output */
    Controller::Outputs _compensated_u {};

    /* Internal Routines */
    void compensateThrusterDynamics(const float &dt);
    void limitSaturation();
    void mixOutput();
    Vector3f pid_attenuations(float tpa_breakpoint, float tpa_rate);

    /**
    * Rotor Mixing scales
    */
    static const int _rotor_count{4};
    static const int _mix_length{5};
    struct  rotors_vector{
        float	roll_scale;	    /**< scales roll for this rotor */
        float	pitch_scale;	/**< scales pitch for this rotor */
        float	yaw_scale;	    /**< scales yaw for this rotor */
        float thrust_scale;     /**< scales Thrust for this rotor */
        float	out_scale;	    /**< scales total out for this rotor */
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



