//
// Created by alsaibie on 5/30/18.
//
#pragma once

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace Dynamics_Math{
    using namespace Eigen;

    inline double poly2n_abs(const Vector3d &k, const double x) {
        /** Simple 2nd degree polynomial - positive x */
        return k(0) * x * x + k(1) * fabs(x) + k(2);
    }

    inline Vector6d state_integral(Vector6d &xdot, Vector6d &x, double dt) {
        Vector6d integ = dt * xdot;
        return integ + x;
    }

    inline Vector3d state_integral(Vector3d &xdot, Vector3d &x, double dt) {
        Vector3d integ = dt * xdot;
        return integ + x;
    }


    inline Matrix6d inverse_jacobian_euler(const Vector3d &att) {
        /** Generate the inverse Jacobian for calculating p_dot = Je_inv*v
         * Generated using the awesome SymPy, Je = [[bRi 0], [0 Jko]] and this is the inverse of that
         */

        double roll_ = att(0), pitch_ = att(1), yaw_ = att(2);

        Matrix6d j;
        j << cos(pitch_) * cos(yaw_), sin(pitch_) * sin(roll_) * cos(yaw_) - sin(yaw_) * cos(roll_),
                sin(pitch_) * cos(roll_) * cos(yaw_) + sin(roll_) * sin(yaw_), 0.0, 0.0, 0.0,
                sin(yaw_) * cos(pitch_), sin(pitch_) * sin(roll_) * sin(yaw_) + cos(roll_) * cos(yaw_),
                sin(pitch_) * sin(yaw_) * cos(roll_) - sin(roll_) * cos(yaw_), 0.0, 0.0, 0.0,
                -sin(pitch_), sin(roll_) * cos(pitch_), cos(pitch_) * cos(roll_), 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1, sin(roll_) * tan(pitch_), cos(roll_) * tan(pitch_),
                0.0, 0.0, 0.0, 0.0, cos(roll_), -sin(roll_),
                0.0, 0.0, 0.0, 0.0, sin(roll_) / cos(pitch_), cos(roll_) / cos(pitch_);

        return j;
    }

    inline Matrix6d inverse_jacobian_q(const Quaterniond &att_q) {
        /** Generate the inverse Jacobian using quaternions for calculating p_dot_q = Je_inv*v */
        double qr = att_q.w(), qi = att_q.x(), qj = att_q.y(), qk = att_q.z();

        Matrix6d j;
        j << -2.0 * pow(qj, 2) - 2.0 * pow(qk, 2) + 1, 2.0 * qi * qj - 2.0 * qk * qr, 2.0 * qi * qk +
                                                                                      2.0 * qj * qr, 0.0, 0.0, 0.0,
                2.0 * qi * qj + 2.0 * qk * qr, -2.0 * pow(qi, 2) - 2.0 * pow(qk, 2) + 1, -2.0 * qi * qr +
                                                                                         2.0 * qj * qk, 0.0, 0.0, 0.0,
                2.0 * qi * qk - 2.0 * qj * qr, 2.0 * qi * qr + 2.0 * qj * qk, -2.0 * pow(qi, 2) - 2.0 * pow(qj, 2) +
                                                                              1, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.5 * qr, -0.5 * qk, 0.5 * qj,
                0.0, 0.0, 0.0, 0.5 * qk, 0.5 * qr, -0.5 * qi,
                0.0, 0.0, 0.0, -0.5 * qj, 0.5 * qi, 0.5 * qr,
                0.0, 0.0, 0.0, -0.5 * qi, -0.5 * qj, -0.5 * qk;

        return j;
    }

    inline Matrix3d iRb_q(const Quaterniond &att_q) {
        double qr = att_q.w(), qi = att_q.x(), qj = att_q.y(), qk = att_q.z();

        Matrix3d r;
        r << -2.0 * pow(qj, 2) - 2.0 * pow(qk, 2) + 1, 2.0 * qi * qj - 2.0 * qk * qr, 2.0 * qi * qk + 2.0 * qj * qr,
                2.0 * qi * qj + 2.0 * qk * qr, -2.0 * pow(qi, 2) - 2.0 * pow(qk, 2) + 1, -2.0 * qi * qr + 2.0 * qj * qk,
                2.0 * qi * qk - 2.0 * qj * qr, 2.0 * qi * qr + 2.0 * qj * qk, -2.0 * pow(qi, 2) - 2.0 * pow(qj, 2) + 1;
    }

    inline Quaterniond exp_wq(const Vector3d &v, const double dt) {
        /** Compute the exponential e^(Adt/2)=Exp{w dt}=q{w dt} */
        double vn = v.norm();

        // This can be singular, but we know when
        if (vn < 0.0001) {
            return Quaterniond::Identity();
        } else {
            auto cs = cos(vn * dt / 2);
            auto sn_n = sin(vn * dt / 2) / vn;
            return Quaterniond(cs, v(0) * sn_n, v(1) * sn_n, v(2) * sn_n);
        }
    }

    inline Quaterniond euler2Quaterniond(const Vector3d &v) {
        /** Compose Quaternion from Euler Angles using XYZ rot sequence */
        return AngleAxisd(v[0], Eigen::Vector3d::UnitX()) *
               AngleAxisd(v[1], Eigen::Vector3d::UnitY()) *
               AngleAxisd(v[2], Eigen::Vector3d::UnitZ());
    }

    inline Vector3d Quaternion2eulerd(const Quaterniond &q) {
        /** Convert Quaternion to Euler Angles using XYZ rot sequence */
        return q.toRotationMatrix().eulerAngles(0, 1, 2);
    }

    inline Quaterniond QuaternionVectord(const Vector3d &v) {
        /** Compose unit vector as quaternion */
        Quaterniond q;
        q.w() = 0;
        q.vec() = v;
        return q;
    }

    inline Matrix3d skew_m(Vector3d &x) {
        Matrix3d m;
        m << 0.0f, -x[2], x[1],
                x[2], 0.0f, -x[0],
                -x[1], x[0], 0.0f;
        return m;
    }
}