//
// Created by alsaibie on 5/18/18.
//

/**
 * @file PositionController.hpp
 *
 * @inputs: Position Setpoint
 * @outputs: Attitude Setpoint
 *
 */

#pragma once

#include <matrix/matrix/math.hpp>




namespace PController {
    using namespace matrix;
    struct States {
        struct Position {
            matrix::Vector3f p;
        } pos;

        struct Attitude {
            matrix::Vector3f orientation;
            matrix::Quatf orientation_q;
        } att;

        struct Velocity {
            matrix::Vector<float, 6> v;
        } vel;

        struct Power {
            float bat_scale_mA;
        } power;
    };

    struct Setpoints {

        struct Position {
            matrix::Vector3f p;
        };

        struct Attitude {
            float thrust;
            matrix::Vector3f orientation;
            matrix::Quatf orientation_q;
        };

        struct Velocity {
            matrix::Vector<float, 6> v;
        };
    };

    struct Gains {
        matrix::Vector3f position_p;
        matrix::Vector3f velocity_p;
    };

    struct Limits {
        matrix::Vector3f max_att_angle;
    };

    struct Constraints {
    };

    struct Outputs {
        float thrust;
        matrix::Vector3f orientation;
        matrix::Quatf orientation_q;
    };

    typedef enum {
        Manual = 0,
        Velocity,
        Position
    } CONTROL_MODE;

    struct ControlMode {
        uint8_t mode;
        bool is_armed;
    };

    struct ControllerStatus {

    };


    class PositionController {

    public:
        PositionController();

        ~PositionController() {};

        /* Input Interfaces */
        void updateStates(const PController::States &state) { _state = state; }

        void updateAttitudeSetpoint(const PController::Setpoints::Attitude &att_sp) { _att_sp = att_sp; }

        void updatePositionSetpoint(const PController::Setpoints::Position &pos_sp) { _pos_sp = pos_sp; }

        void updateVelocitySetpoint(const PController::Setpoints::Velocity &vel_sp) { _vel_sp = vel_sp; }

        void updateGains(const PController::Gains &gains) { _gains = gains; }

        void updateLimits(const PController::Limits &limits) { _limits = limits; }

        void updateSaturations(const PController::Constraints &constraints) { _constraints = constraints; }

        void updateControlMode(const PController::ControlMode mode) { _mode = mode; }

        void resetSetpoints();

        void resetReferenceState() { _qref = _state.att.orientation_q; }

        /* Controller Executions */
        void controlAttitude(const float &dt);

        void controlVelocity(const float &dt);

        void controlPosition(const float &dt);

        /* Output Interfaces */
        PController::Limits getLimits() { return _limits; }

        PController::Gains getGains() { return _gains; }

        PController::Setpoints::Position getPositionSetpoint() { return _pos_sp; }

        PController::Setpoints::Velocity getVelocitySetpoint() { return _vel_sp; }

        PController::Outputs getDesiredAttitude() { return _att_d; }

        PController::ControllerStatus getControllerStatus() { return _ctrl_status; }

    private:

        /* States */
        PController::States _state{};

        /* Setpoints - Not necessary provided together in the controller configuration */
        PController::Setpoints::Position _pos_sp{};
        PController::Setpoints::Velocity _vel_sp{};
        PController::Setpoints::Attitude _att_sp{};

        /* Control Intermediate Outputs */

        /* Control Gains,  Limits and Saturation */
        PController::Gains _gains{};
        PController::Limits _limits{};
        PController::Constraints _constraints{};
        PController::ControlMode _mode{};

        /* Control Variables */
        PController::ControllerStatus _ctrl_status{};
        Quatf _qref;
        /* Control Output */
        PController::Outputs _att_d{};

        /* Internal Routines */

    };


};
