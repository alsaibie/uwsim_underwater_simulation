//
// Created by alsaibie on 5/30/18.
//

#include "AUVDynamics.hpp"
#include <iostream>
using namespace Dynamics;

AUVDynamics::AUVDynamics() {
    /* Zero States */
    _state.pos.setZero();
    _state.pos_dot.setZero();
    _state.vel.setZero();
    _state.vel_dot.setZero();
    _state.att.orientation.setZero();
    _state.att.orientation_q.setIdentity();
    _state.power.bat_percent_remaining = 100.0;
    _state.power.bat_vnom = 8.4;
    _state.power.bat_I_mA = 0;
}

void AUVDynamics::initialize(void){

    /* Initialize Dynamics - only after setting constants */
    _dparam.Mv = mass_matrix();
    _dparam.Mv_inv = _dparam.Mv.inverse();

    /** x forward z downward convention (y right)
    * Thrust on x axis, roll around x axis, pitch around y axis and yaw around z axis
    * -- 3  CW -- 1 CCW --
    * ---------\/---------
    * ---------/\---------
    * -- 2 CCW -- 4  CW --
    * The direction should be set by the mixer and the thrust / torque should be calculated in the generalized
    * force call, this matrix should be the geometric allocation matrix tau (6x1)= Bu (6x8)(8x1)
    * where u = [T1 T2 T3 T4 Tor1 Tor2 Tor3 Tor4]^T
    * Negative sign to account for reversed motors 3 & 4, motor speed sign should determine flow direction.
    * Positive speed sign means the thrust is pushing the vehicle in positive surge direction. This implies
    * that 2 motors rotate in the opposite direction and with mirrored impellers. The torque direction must be
    * carried in the B matrix, so we need to know the rot direction there as well to compute the roll torque
    **/

    // TODO: Move this matrix to the yaml file, since it's based on pure geometric configuration
    VectorXi inv_ = _constants.actuator.dir_inversion;
    double L = _constants.actuator.L;
    _dparam.B = MatrixXd::Zero(6, _constants.actuator.count * 2);
    _dparam.B << 1, 1, 1, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, inv_(0), inv_(1), inv_(2), inv_(3),
            -L, L, -L, L, 0, 0, 0, 0,
            -L, L, L, -L, 0, 0, 0, 0;
    _dparam.B_T = _dparam.B.bottomLeftCorner(2,4);
    std::cout << "Mass Matrix" << std::endl << _dparam.Mv << std::endl;
    std::cout << "B Matrix" << std::endl << _dparam.B << std::endl;
    std::cout << "Sub B Matrix" << std::endl << _dparam.B_T << std::endl;
    std::cout <<" Unity Dvv Vector: " <<  std::endl << damping_vector(VectorXd::Constant(6, 1)) << endl;
    std::cout <<" Unity Cv Matrix: " <<  std::endl << coriolis_matrix(VectorXd::Constant(6, 1)) << endl;
    std::cout <<" Unity Cvv Vector: " <<  std::endl << coriolis_matrix(VectorXd::Constant(6, 1)) * VectorXd::Constant(6, 1) << endl;

//    _w_hz_sp.setZero(_constants.actuator.count);
    _w_hz_sp.setZero(4);

}

Matrix6d AUVDynamics::mass_matrix() {
    /**
     * TODO: add note
     */

    /* Mass: Rigid Body */
    double m_ = _constants.inertia.mass;
    Matrix6d M_ = MatrixXd::Zero(6,6);
    M_.topLeftCorner(3, 3) <<   m_, 0.0, 0.0,
                                0.0, m_, 0.0,
                                0.0, 0.0, m_;
    M_.bottomRightCorner(3,3) = _constants.inertia.tensor;

    //  Tensor is precalculated and provided
    //  Ix = self.mass*(pow(b,2)+pow(c,2))/5
    //  Iy = self.mass*(pow(a,2)+pow(c,2))/5
    //  Iz = self.mass*(pow(a,2)+pow(b,2))/5

    /* Added Mass */
    auto a_ = _constants.inertia.radius(0);
    auto b_ = _constants.inertia.radius(1);
    auto c_ = _constants.inertia.radius(2);

    double ecc = 1 - pow(b_ / a_, 2);
    double dummy = log((1 + ecc) / (1 - ecc));
    double alpha_0 = (2 * (1 - pow(ecc, 2)) * (.5 * dummy - ecc)) / (pow(ecc, 3));
    double beta_0 = 1 / (pow(ecc, 2)) - ((1 - pow(ecc, 2)) * dummy) / (2 * pow(ecc, 3));
    double Xud_ = -m_ * (alpha_0 / (2 - alpha_0));
    double Yvd_ = -m_ * (beta_0 / (2 - beta_0));
    double Zwd_ = Yvd_;

    double dummy1 = (pow(b_, 2) - pow(a_, 2));
    double dummy2 = (alpha_0 - beta_0);
    double dummy3 = (pow(b_, 2) + pow(a_, 2));
    double Mqd_ = (-m_ / 5) * ((pow(dummy1, 2) * dummy2) / (2 * dummy1 - dummy3 * dummy2));
    double Nrd_ = Mqd_;
    double Kpd_ = Mqd_;

    Vector6d m_a_; m_a_ << -Xud_, -Yvd_, -Zwd_, -Kpd_, -Mqd_, -Nrd_;
    Matrix6d Ma_ = m_a_.asDiagonal();

    M_ += Ma_;

    return M_;
}

Vector6d AUVDynamics::damping_vector(const Vector6d &v) {
    /**
     *  The damping force vector computed here is based on CFD analysis.
     *  The sign of the elements correspond to the sign of the velocity element.
     */

    Vector6d Dv; Dv <<   poly_abs_discontinuos_positive(_constants.damping.ksurge, v[0]),
                         poly_abs_discontinuos_positive(_constants.damping.ksway,  v[1]),
                         poly_abs_discontinuos_positive(_constants.damping.kheave, v[2]),
                         poly_abs_discontinuos_positive(_constants.damping.kroll,  v[3]),
                         poly_abs_discontinuos_positive(_constants.damping.kpitch, v[4]),
                         poly_abs_discontinuos_positive(_constants.damping.kyaw,   v[5]);

    return Dv;

}



Matrix6d AUVDynamics::coriolis_matrix(const Vector6d &v) {
    /**
     * TODO: add note
     */
    static Matrix3d Mv11_ = _dparam.Mv.block<3,3>(0,0), Mv12_ = _dparam.Mv.block<3,3>(0,3),
            Mv21_ = _dparam.Mv.block<3,3>(3,0), Mv22_ = _dparam.Mv.block<3,3>(3,3);

    Vector3d vTranslate_ =  v.head<3>();
    Vector3d vRotate_ =  v.tail<3>();
    Vector3d dot1 = Mv11_ * vTranslate_ + Mv12_ * vRotate_;
    Vector3d dot2 = Mv21_ * vTranslate_ + Mv22_ * vRotate_;

    Matrix6d Cv_ = Matrix6d::Zero();
    Cv_.topRightCorner(3, 3)    = -skew_m(dot1);
    Cv_.bottomLeftCorner(3, 3)  = -skew_m(dot1);
    Cv_.bottomRightCorner(3, 3) = -skew_m(dot2);


    return Cv_;
}

Vector6d AUVDynamics::gravity_vector(const Quaterniond &q) {
    /** Computes the gravity and buoyancy forces. For a spheroid - this only matters if the mAUV is not neutrally
    * buoyant
     * **/
    /* Weight and Floatability */
    static auto W_ = _constants.inertia.mass * _constants.inertia.g;  // [Kg]

    /* Assume the vehicle is always submerged */
    static auto r = _constants.inertia.radius;
    auto qw_ = q.w(), qx_ = q.x(), qy_ = q.y(), qz_ = q.z();

    // TODO: Incorporate the displaced water volume in here to compute B, for now assume neutral buoyancy
    // B = ((4 * math.pi * r[0] * r[1] * r[2]) / 3) * _density * _g
    static auto B_ = W_;
    /* gravity center position in the robot fixed frame (x',y',z') [m]  see equation 2.48 */
    static auto xg_ = _constants.inertia.rG(0), yg_ = _constants.inertia.rG(1), zg_ = _constants.inertia.rG(2);
    static auto xb_ = _constants.inertia.rB(0), yb_ = _constants.inertia.rB(1), zb_ = _constants.inertia.rB(2);

    auto qw_2_ = pow(qw_, 2);
    auto qx_2_ = pow(qx_, 2);
    auto qy_2_ = pow(qy_, 2);
    auto qz_2_ = pow(qz_, 2);
    auto qxqz_ = qx_ * qz_;
    auto qyqz_ = qy_ * qz_;
    auto qwqx_ = qw_ * qx_;
    auto qwqy_ = qw_ * qy_;
    auto term1 = (qwqy_ - qxqz_);
    auto term2 = (qwqx_ + qyqz_);
    auto term3 = (-qw_2_ + qx_2_ + qy_2_ - qz_2_);

    Vector6d g_; g_ << 2 * term1 * (W_ - B_),
                        -2 * term2 * (W_ - B_),
                        term3 * (W_ - B_),
                        term3 * (yg_ * W_ - yb_ * B_) + 2 * term2 * (zg_ * W_ - zb_ * B_),
                        -term3 * (xg_ * W_ - xb_ * B_) + 2 * term1 * (zg_ * W_ - zb_ * B_),
                        -2 * term2 * (xg_ * W_ - xb_ * B_) - 2 * term1 * (yg_ * W_ - yb_ * B_);
    return g_;

}

VectorXd AUVDynamics::motors_transient_dynamics(const VectorXd &w_hz_sp, const double &dt) {
    /** This captures the motor transients:
    *   Difference eq from diffq y[k] = (Kp*u *dt + y[k-1] *tau) / ( tau + dt) : period is too low that this becomes
    *   accurate compared to difference from z-transform
    **/
    VectorXd w_hz_tr_ = VectorXd::Zero(w_hz_sp.rows());
    static VectorXd w_hz_tr_1_ = w_hz_tr_;

    for(int k = 0; k < _constants.actuator.count; k++) {
        w_hz_tr_(k) = (dt * w_hz_sp(k) + _constants.actuator.tconst * w_hz_tr_1_(k)) / (dt + _constants.actuator.tconst);
        w_hz_tr_1_(k) = w_hz_tr_(k);
    }

    return w_hz_tr_;
}

VectorXd AUVDynamics::thrust_n(const VectorXd &w_hz_tr) {
    /** Compute the Thrust in N given speed of rotor in Hz
    *   Apply the thruster dead-zone truncation here
    *   Note: Positive motor speed means thrust in forward Surge direction,
     *   Rotation Direction / Torque sign is carried in B Matrix
    **/
    static VectorXd thrust_n_ = VectorXd::Zero(w_hz_tr.rows());
    static VectorXd v_t = VectorXd::Zero(w_hz_tr.rows());

    thrust_n_.setZero();
    // TODO: ADD rotational effects to speed
    v_t << _state.vel(0) * VectorXd::Constant(4,1) + _dparam.B_T.transpose() *_state.vel.bottomRows(2);
    for(int k = 0; k < _constants.actuator.count; k++) {
        thrust_n_(k) = poly_abs_discontinuos_positive(_constants.actuator.kf, w_hz_tr(k));

        double J = 0.00;
        /* We assume bidirectional symmetry advance speed effect on thrust */
        if (fabs(w_hz_tr(k)) > 10.0) {
            J = v_t(k) / (_constants.actuator.impeller_D * w_hz_tr(k) * 2.00 * M_PI);
            if ((w_hz_tr(k) * v_t(k)) > 0) {
                thrust_n_(k) *= max((1 +  _constants.actuator.NKT_Coefficient * J * J), 0.0);
            }
            else if ((w_hz_tr(k) * v_t(k)) < 0) {
                thrust_n_(k) *= max((1 - _constants.actuator.NKT_Coefficient * J * J), 0.0);
            }
        }
    }

    return thrust_n_ ;
}

VectorXd AUVDynamics::torque_n(const VectorXd &w_hz_tr) {
    /** Compute the Torque in N.m given speed of rotor in Hz TODO: Check units
    * Apply the thruster dead-zone truncation here
    **/
    static VectorXd torque_n_ = VectorXd::Zero(w_hz_tr.rows());

    torque_n_.setZero();

    for(int k = 0; k < _constants.actuator.count; k++) {
        torque_n_(k) = poly_abs_discontinuos_positive(_constants.actuator.km, w_hz_tr(k));
    }

    return torque_n_ ;
}

Vector6d AUVDynamics::generalized_force(const VectorXd &w_hz_tr) {
    /**
     * Apply Allocation Matrix
     */

    static VectorXd u_ = VectorXd::Zero(_constants.actuator.count * 2);

    u_.head(_constants.actuator.count) =  thrust_n(w_hz_tr);
    u_.tail(_constants.actuator.count) =  torque_n(w_hz_tr);

    return _dparam.B * u_;
}

void AUVDynamics::inverse_dynamics(const double &dt) {
    /** Given the setpoint for each thruster, the previous velocity and the
     * previous position computes the v_dot
     */
    static Vector6d CDvv_;

    if(DEBUG_OUTPUT) {
        cout << "vel: " << _state.vel << endl;
        cout << "whzsp: " << _w_hz_sp << endl;
    }

    _dparam.Dvv   = damping_vector(_state.vel);
    _dparam.Cv   = coriolis_matrix(_state.vel);
    _dparam.Cvv  = _dparam.Cv * _state.vel;
    _dparam.gRB  = gravity_vector(_state.att.orientation_q);
    _w_hz_tr     = motors_transient_dynamics(_w_hz_sp, dt);  // Apply First Order Delay
    _dparam.tau  = generalized_force(_w_hz_tr);  // Apply thruster dynamics and allocation matrix

    if(DEBUG_OUTPUT) {
        cout << "Dvv: " << _dparam.Dvv << endl;
        cout << "Cvv: " << _dparam.Cvv << endl;
        cout << "gRB: " << _dparam.gRB << endl;
        cout << "whztr: " << _w_hz_tr << endl;
        cout << "tau: " << _dparam.tau << "\n" << endl;
    }

//    _state.vel_dot = _dparam.Mv_inv * (_dparam.tau - _dparam.Dvv - _dparam.gRB); // inv(M)*(tau-cdvv-g+collisionForce)
    _state.vel_dot = _dparam.Mv_inv * (_dparam.tau - _dparam.Dvv - _dparam.Cvv - _dparam.gRB); // inv(M)*(tau-cdvv-g+collisionForce)

//     self.collision_force = [0, 0, 0, 0, 0, 0]   # TODO: Incorporate Collision Dynamics

    if (!_inverse_dynamics_initialized) _inverse_dynamics_initialized = true;

    _state.vel = state_integral(_state.vel_dot, _state.vel, dt);
}

void AUVDynamics::inverse_kinematics(const double &dt) {
    /**
     *     Transform body fixed velocity to inertial reference frame,
     *     rpy_rate need to be expressed in inertial f. to integrate ref Antonelli eq2.21
     */

    /* Calculate p_dot_translation with a standard SO(3) rotation matrix */
    static Matrix3d Rinv_;
    static Quaterniond expq_;

    /* Translation */
    Rinv_ = iRb_q(_state.att.orientation_q);
    _state.pos_dot = Rinv_ * _state.vel.head(3);
    _state.pos = state_integral(_state.pos_dot, _state.pos, dt);

    /* Rotation */
    expq_ = exp_wq(_state.vel.tail(3), dt);
    _state.att.orientation_q = _state.att.orientation_q * expq_;
    _state.att.orientation   =  Quaternion2eulerd(_state.att.orientation_q);
    /* Discontinuity Cleanup TODO: Use a better approach*/
    if(fabs(_state.att.orientation(0)) > (M_PI - 0.05f)){
        _state.att.orientation(0) = (_state.att.orientation(0) > 0.0 ? (M_PI -_state.att.orientation(0)) :
                                     - (M_PI - fabs(_state.att.orientation(0))));
    }
    if(fabs(_state.att.orientation(1)) > (M_PI - 0.05f)){
//        _state.att.orientation(1) = (_state.att.orientation(1) > 0.0 ? (M_PI -_state.att.orientation(1)) :
//                                     - (M_PI - fabs(_state.att.orientation(1))));
          _state.att.orientation(1) = fabs(_state.att.orientation(1));
    }
    if(fabs(_state.att.orientation(2)) > (M_PI - 0.05f)){
        _state.att.orientation(2) = (_state.att.orientation(2) > 0.0 ? (M_PI -_state.att.orientation(2)) :
                                     - (M_PI - fabs(_state.att.orientation(2))));
    }

    if(DEBUG_OUTPUT){
        cout << " Pos: " << _state.pos << endl;
        cout << " Att Euler: " << _state.att.orientation << endl;
        printf("Att Q: %f, %f, %f, %f", _state.att.orientation_q.w(), _state.att.orientation_q.x(),
               _state.att.orientation_q.y(), _state.att.orientation_q.z());
        cout << "\n" << endl;
    }
}

void AUVDynamics::vnom_battery_dynamics(const VectorXd &w_hz_tr, const double &dt) {
    /** compute current draw rate based on motor dynamics - must be called at T = self.diffq_period **/
    static double bat_mAh_consumed_ = 0.0f;

    _state.power.bat_I_mA = 0.0;
    for(int k = 0; k < _constants.actuator.count; k++) {
        /* Total draw from all actuators */
        _state.power.bat_I_mA += poly_abs(_constants.battery.ki, w_hz_tr(k));
    }

    bat_mAh_consumed_ = bat_mAh_consumed_ + (_state.power.bat_I_mA * dt) / 3600.0;

    /* Update remaining charge voltage based on consumed mAh and instantaneous current draw */
    double mAh_consumed_percentage_ = bat_mAh_consumed_ * 100 / _constants.battery.mAh;
    _state.power.bat_percent_remaining = 100 - mAh_consumed_percentage_;

    double mV_ = _constants.battery.cell_n * (poly_abs(_constants.battery.kBd,mAh_consumed_percentage_) -
                                              _state.power.bat_I_mA * _constants.battery.esr / 1000.0);
    _state.power.bat_vnom =  mV_ / 1000.0;
}

void AUVDynamics::setInitialState(const States &initial_state){
    _state = initial_state;}

void AUVDynamics::setConstants(const Constants &constants){
    _constants = constants;
    _actuator_count = _constants.actuator.count;}


void AUVDynamics::setInput(const Inputs &input){

    /** Receives the control input, the control input should be in PWM format, ranging from 1100 to 1900 with 1500
    *as zero, 1100 as max negative, 1900 as max positive. This is the output of the pwm mixer, which should take
    *care of any saturation and gain setting, Here it is scaled based on the thruster model to output w in
    *Hz
    **/

    /** Convert PWM to Voltage TODO: Update the model such that it captures voltage drop over time for a LiPO,
    * to do that I need to capture current consumption dynamics which I have the coefficients for I(n): Amp(Hz) -
    * which should be a power consumption cumulative function that updates at a given sample time then updates
    * Vactual which is then used here instead. And not only here, but fed back to the controller so that the
    * voltage sent is adjusted accordingly.
    **/

    Inputs inputs_ = input;
    double min_pwm_ = _constants.actuator.pwm_range(0), mid_pwm_ = _constants.actuator.pwm_range(1),
            max_pwm_ = _constants.actuator.pwm_range(2);

    if(DEBUG_OUTPUT) cout << "w_pwm: " << input.w_pwm << endl;

    for (int k = 0; k <_constants.actuator.count; k++){

        double w_voltage_ = 2.0 * _state.power.bat_vnom * (inputs_.w_pwm(k) - mid_pwm_) / (max_pwm_ - min_pwm_);
        if(DEBUG_OUTPUT) cout << "voltage: " << w_voltage_ << endl;
        _w_hz_sp(k) = poly_abs_discontinuos_positive(_constants.actuator.kv, w_voltage_) ;
    }

}

void AUVDynamics::iterate(const double &dt) {
    inverse_dynamics(dt);
    inverse_kinematics(dt);
    vnom_battery_dynamics(_w_hz_tr, dt);
    // collision_dynamics(dt); //TODO
}