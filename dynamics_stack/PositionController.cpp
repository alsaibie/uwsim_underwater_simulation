//
// Created by alsaibie on 5/22/18.
//

#include "PositionController.hpp"
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace PController;

PositionController::PositionController() {

    _state.pos.p.zero();
    _state.vel.v.zero();
    _state.att.orientation.zero();
    _state.att.orientation_q.zero();
    _state.att.orientation_q(0) = 1.0;
    _state.power.bat_scale_mA = 0;

    _vel_sp.v.zero();
    _pos_sp.p.zero();
    _att_sp.thrust = 0;
    _att_sp.orientation.zero();
    _att_sp.orientation_q.zero();
    _att_sp.orientation_q(0) = 1.0;

    _att_d.thrust = 0;
    _att_d.orientation_q.zero();
    _att_d.orientation_q(0);

    _qref = Quatf(1.0f, 0.0f, 0.0f, 0.0f);

}

void
PositionController::resetSetpoints() {

    _vel_sp.v.zero();
    _pos_sp.p.zero();
    _att_sp.thrust = 0;
    _att_sp.orientation.zero();
    _att_sp.orientation_q.zero();
    _att_sp.orientation_q(0) = 1.0;
}

void
PositionController::controlAttitude(const float &dt) {



    if(_mode.mode == PController::Manual){


        _att_d.orientation = _att_sp.orientation.emult(_limits.max_att_angle);
//        PX4_INFO("euler d %f, %f, %f, %f", (double)_att_d.orientation(0), (double)_att_d.orientation(1),
//                 (double)_att_d.orientation(2));

        _att_d.orientation_q = matrix::Eulerf(_att_d.orientation(0), _att_d.orientation(1), _att_d.orientation(2));
//        PX4_INFO("att d %f, %f, %f, %f", (double)_att_d.orientation_q(0), (double)_att_d.orientation_q(1),
//                 (double)_att_d.orientation_q(2), (double)_att_d.orientation_q(3));

        /* Desired attitude is w.r.t to reference position */
//        _att_d.orientation_q =    _qref * _att_d.orientation_q.inversed();
        _att_d.orientation_q =    _qref.inversed() * _att_d.orientation_q;
        _att_d.orientation_q *= math::signNoZero(_att_d.orientation_q(0));
        _att_d.orientation_q(0) = math::constrain(_att_d.orientation_q(0), -1.f, 1.f);
        _att_d.orientation_q(1) = math::constrain(_att_d.orientation_q(1), -1.f, 1.f);
//        PX4_INFO("att ref %f, %f, %f, %f", (double)_qref(0), (double)_qref(1),
//                 (double)_qref(2), (double)_qref(3));
//
//        PX4_INFO("att d comp %f, %f, %f, %f", (double)_att_d.orientation_q(0), (double)_att_d.orientation_q(1),
//                 (double)_att_d.orientation_q(2), (double)_att_d.orientation_q(3));
        _att_d.thrust = _att_sp.thrust;
    }
}

void
PositionController::controlVelocity(const float &dt) {

}

void
PositionController::controlPosition(const float &dt) {

}



