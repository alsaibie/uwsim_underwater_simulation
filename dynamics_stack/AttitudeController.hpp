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


namespace AController {
    using namespace matrix;

    struct States {
        struct Attitude {
            Quatf q;
        } att;
        struct Rates {
            Vector3f rpy;
        } rates;
        struct Power {
            bool bat_scale_en;
            float bat_scale_mA;
            float bat_V_nom;
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
        Vector3f tau_max_sp;
        Vector3f rate_int_lim;
        float acro_expo_py;
        float acro_expo_r;
        float acro_superexpo_py;
        float acro_superexpo_r;
    };

    struct Constants {
        Vector3f nThrust_coefficients;
        Vector3f nTorque_coefficients;
        Vector3f Vn_coefficients;
        Vector3f Vn_norm_coefficients;
        float rotor_radius;
    };

    struct Saturation {

    };

    typedef enum {
        Manual = 0,
        Auto,
        Acro
    } CONTROL_MODE;

    struct ControlMode {
        uint8_t mode;
        bool is_armed;
    };

    struct ControllerStatus {
        Vector3f rates_int;
        Vector3f rates_prev;
        Vector3f rates_prev_filtered;
        Vector3f rate_int_lim;
    };

    inline float poly_abs_discontinuos(const Vector3f &v, const float x) {
        /** Simple 2nd degree polynomial - reserves sign of x, assumes positive curve fitting */
        float ret = v(0);
        for (int k = 1; k < 3; k++ ){
            ret += v(k) * pow(fabs(x), k);
        }
        if (x > 0.0f){
            if (ret < 0.0f){
                return 0.0f;
            }
            else return ret;
        }
        else if (x < 0.0f){
            if (-ret > 0.0f){
                return 0.0f;
            }
            else return -ret;
        }
        else {
            return 0.0f;
        }
    }


    class AttitudeController {

    public:
        AttitudeController();

        ~AttitudeController() {};

        /* Input Interfaces */
        void updateStates(const AController::States &state) { _state = state; }

        void updateAttitudeSetpoint(const AController::Setpoints::Attitude &att_sp) { _att_sp = att_sp; }

        void updateRateSetpoint(const AController::Setpoints::Rates &rates_sp) { _rates_sp = rates_sp; }

        void updateGains(const AController::Gains &gains) { _gains = gains; }

        void updateLimits(const AController::Limits &limits) { _limits = limits; }

        void updateConstants(const AController::Constants &constants) { _constants = constants; }

//    void updateFilters(const Controller::Filters &filters) {_filters = filters;}
        void updateSaturations(const AController::Saturation &saturation) { _saturation = saturation; }

        void updateControlMode(const AController::ControlMode mode) { _mode = mode; }

        void setFilter(math::LowPassFilter2p *filt) { _filters = filt; }

        void resetSetpoints();

        /* Controller Executions */
        void controlAttitude(const float &dt); /**/
        void controlRates(const float &dt);

        /* Output Interfaces */
        AController::Limits getLimits() { return _limits; }

        AController::Gains getGains() { return _gains; }

        AController::Setpoints::Rates getRateSetpoint() { return _rates_sp; }

        Vector3f getTauSetpoint() { return _tau_sp; }

        Vector<float, 4> getCompensatedVoltageSetpoint() { return _compensated_V_sp; }

        AController::Outputs getMixedControlOutput() { return _mixed_u; }

        AController::ControllerStatus getControllerStatus() { return _ctrl_status; }

    private:

        /* States */
        AController::States _state{};

        /* Setpoints - Not necessary provided together in the controller configuration */
        AController::Setpoints::Attitude _att_sp{};
        AController::Setpoints::Rates _rates_sp{};

        /* Control Intermediate Outputs */
        Vector3f _tau_sp{};
        Vector<float, 4> _compensated_V_sp{};

        /* Control Gains,  Limits and Saturation */
        AController::Gains _gains{};
        AController::Limits _limits{};
        AController::Constants _constants{};
        math::LowPassFilter2p *_filters; // TODO: Share ptr for now, but break dependency for complete abstraction
        AController::Saturation _saturation{};
        static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
        AController::ControlMode _mode{};

        /* Control Variables */
        AController::ControllerStatus _ctrl_status;

        /* Control Output */
        AController::Outputs _mixed_u{};

        /* Internal Routines */
        void compensateThrusterDynamics();

        void mixOutput();

        Vector3f pid_attenuations(float tpa_breakpoint, float tpa_rate);

        /**
        * Rotor Mixing scales
        */
        static const int _rotor_count{4};
        static const int _mix_length{5};
        struct rotors_vector {
            float roll_scale;        /**< scales roll for this rotor */
            float pitch_scale;    /**< scales pitch for this rotor */
            float yaw_scale;        /**< scales yaw for this rotor */
            float thrust_scale;     /**< scales Thrust for this rotor */
            float out_scale;        /**< scales total out for this rotor */
        } _rotors[_rotor_count];

        /** Mixing Table. Order: Rollscale, PitchScale, YawScale, ThrustScale, OutScale */
//    static constexpr float _dolphin_x_table[_rotor_count][_mix_length] = {
//            { -1.000000,  0.707107,  -0.707107, 1.000000, 1.000000 },
//            { -1.000000,  -0.707107,  0.707107, 1.000000, 1.000000 },
//            { 1.000000, 0.707107,   0.707107, 1.000000, 1.000000 },
//            { 1.000000, -0.707107, -0.707107, 1.000000, 1.000000 },
//    };
//        float _dolphin_x_table[_rotor_count][_mix_length] = {
//                {-1.000000, -0.5,  -0.5, 1.000000, 1.000000},
//                {-1.000000, 0.5, 0.5,  1.000000, 1.000000},
//                {1.000000,  -0.5,  0.5,  1.000000, 1.000000},
//                {1.000000,  0.5, -0.5, 1.000000, 1.000000},
//        };
                float _dolphin_x_table[_rotor_count][_mix_length] = {
                {1.000000, -1.000,  -1.0000, 1.000000, 1.000000},
                {1.000000, 1.0000, 1.0000,  1.000000, 1.000000},
                {-1.000000,  -1.0000,  1.0000,  1.000000, 1.000000},
                {-1.000000,  1.000, -1.0000, 1.000000, 1.000000},
        };

    };
};



