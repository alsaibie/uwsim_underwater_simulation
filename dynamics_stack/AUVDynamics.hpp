//
// Created by alsaibie on 5/30/18.
//

#pragma once
#include <Eigen/Dense>
#include "dynamics_math_helper.hpp"

using namespace std;
using namespace Eigen;
using namespace Dynamics_Math;

namespace Dynamics {

    struct States {

        Vector3d pos;
        Vector3d pos_dot;
        Vector6d vel;
        Vector6d vel_dot;

        struct Attitude {
            Vector3d orientation;
            Quaterniond orientation_q;
        } att;

        struct Power {
            double bat_vnom;
            double bat_I;
            double bat_percent_remaining;
        } power;
    };

    struct DynamicParameters {
        Matrix6d Mv;
        Matrix6d Mv_inv;
        Matrix6d Dv;
        Matrix6d Cv;
        Vector6d gRB;
        MatrixXd B;
        Vector6d tau;
    };

    struct Inputs{
        VectorXd w_pwm;
    };

    struct Constants{

        struct Inertia{
            double mass;
            Vector3d rG;
            Vector3d rB;
            double g;
            Vector3d radius;
            Matrix3d tensor;
        }inertia;

        struct Damping{
            Vector6d damping {};
            Vector6d q_damping {};
            double dzv {};
            double dv {};
            double dh {};
        }damping;

        struct Actuator{
            int count {};
            Vector3d kf {}; /* Thrust(n_Hz) coefficients*/
            Vector3d km {}; /* Torque(n_Hz) coefficients*/
            Vector3d kv {}; /* n_Hz(voltage) coefficients*/
            double L {};
            double tconst {};
            double maxsat {};
            double minsat {};
            Matrix<int, 4, 1> dir_inversion {};
            Vector3i pwm_range {};

        }actuator;

        struct Environment {
            double density;
        }environment;

        struct Battery{
            Vector3d ki {};

            Vector3d kBd {};
            double c_vmax {};
            double c_vcut {};
            double esr {};
            double cell_n {};
            int mAh {};
        }battery;


    };

}


using namespace Dynamics;

class AUVDynamics{

    public:

    AUVDynamics();

    ~AUVDynamics() {};

    void iterate(const double &dt);

    void setInput(const Inputs &input);

    void setInitialState(const States &initial_state){ _state = initial_state;}

    void setConstants(const Constants &constants){
        _constants = constants;
        _actuator_count = _constants.actuator.count;}

    States getStates(void) {return _state;}

    DynamicParameters getDynamicParameters(void) {return _dparam;}

private:

    /** Vehicle Dynamics */
    void inverse_dynamics(const double &dt);

    void inverse_kinematics(const double &dt);

    Matrix6d mass_matrix();

    Matrix6d damping_matrix(const Vector6d &v);

    Matrix6d coriolis_matrix(const Vector6d &v);

    Vector6d gravity_vector(const Quaterniond &q);

    Vector6d generalized_force(const VectorXd &w_hz_tr);

    VectorXd motors_transient_dynamics(const VectorXd &w_hz_sp, const double &dt);

    VectorXd thrust_n(const VectorXd &w_hz_tr);

    VectorXd torque_n(const VectorXd &w_hz_tr);

    void vnom_battery_dynamics(const VectorXd &w_hz_tr, const double &dt);

    /** dynamics variables */

    DynamicParameters _dparam {};
    States _state {};
    Constants _constants;

    VectorXd _w_hz_sp {};
    VectorXd _w_hz_tr {};

    int  _actuator_count {4};
    bool _inverse_dynamics_initialized {false};


};
