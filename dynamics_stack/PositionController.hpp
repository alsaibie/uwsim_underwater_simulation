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


using namespace matrix;

namespace Controller
{
    struct States {
        struct Position{
            Vector3f p;
        } pos;

        struct Attitude{
            Vector3f orientation;
            Quatf orientation_q;
        } att;

        struct Velocity {
            Vector<float, 6> v;
        } vel;

        struct Power{
            float bat_scale_mA;
        } power;
    };

    struct Setpoints {

        struct Position{
            Vector3f p;
        };

        struct Attitude{
            float thrust;
            Vector3f orientation;
            Quatf orientation_q;
        };

        struct Velocity {
            Vector<float, 6> v;
        };
    };

    struct Gains {
        Vector3f position_p;
        Vector3f velocity_p;
    };

    struct Limits {
        Vector3f max_att_angle;
    };

    struct Constraints {
    };

    struct Outputs {
        float thrust;
        Vector3f orientation;
        Quatf orientation_q;
    };

    typedef enum {
        Manual = 0,
        Velocity,
        Position
    } CONTROL_MODE;

    struct ControlMode{
        uint8_t mode;
        bool is_armed;
    };

    struct ControllerStatus {

    };
}

class PositionController {

public:
    PositionController();
    ~PositionController() {};

    /* Input Interfaces */
    void updateStates(const Controller::States &state){ _state = state; }
    void updateAttitudeSetpoint(const Controller::Setpoints::Attitude &att_sp){ _att_sp = att_sp;}
    void updatePositionSetpoint(const Controller::Setpoints::Position &pos_sp){ _pos_sp = pos_sp;}
    void updateVelocitySetpoint(const Controller::Setpoints::Velocity &vel_sp){ _vel_sp = vel_sp;}
    void updateGains(const Controller::Gains &gains) {_gains = gains;}
    void updateLimits(const Controller::Limits &limits) {_limits = limits; }
    void updateSaturations(const Controller::Constraints &constraints) {_constraints = constraints;}
    void updateControlMode(const Controller::ControlMode mode){_mode = mode;}
    void resetSetpoints();
    void resetReferenceState() {_qref = _state.att.orientation_q;}

    /* Controller Executions */
    void controlAttitude(const float &dt);
    void controlVelocity(const float &dt);
    void controlPosition(const float &dt);

    /* Output Interfaces */
    Controller::Limits              getLimits() {return _limits;}
    Controller::Gains               getGains() {return _gains;}
    Controller::Setpoints::Position getPositionSetpoint() { return _pos_sp; }
    Controller::Setpoints::Velocity getVelocitySetpoint() { return _vel_sp; }
    Controller::Outputs             getDesiredAttitude() { return _att_d;}
    Controller::ControllerStatus    getControllerStatus() {return _ctrl_status;}

private:

    /* States */
    Controller::States _state {};

    /* Setpoints - Not necessary provided together in the controller configuration */
    Controller::Setpoints::Position  _pos_sp {};
    Controller::Setpoints::Velocity  _vel_sp {};
    Controller::Setpoints::Attitude  _att_sp {};

    /* Control Intermediate Outputs */

    /* Control Gains,  Limits and Saturation */
    Controller::Gains _gains {};
    Controller::Limits _limits {};
    Controller::Constraints _constraints {};
    Controller::ControlMode _mode {};

    /* Control Variables */
    Controller::ControllerStatus _ctrl_status {};
    Quatf _qref;
    /* Control Output */
    Controller::Outputs _att_d {};

    /* Internal Routines */

};



