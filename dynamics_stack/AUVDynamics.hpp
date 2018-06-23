//
// Created by alsaibie on 5/30/18.
//

#pragma once
#include <Eigen/Dense>
#include "dynamics_math_helper.hpp"



namespace Dynamics {

    using namespace std;
    using namespace Eigen;
    using namespace Dynamics_Math;

    struct States {

        Vector3d pos{};
        Vector3d pos_dot{};
        Vector6d vel{};
        Vector6d vel_dot{};

        struct Attitude {
            Vector3d orientation{};
            Quaterniond orientation_q{};
        } att;

        struct Power {
            double bat_vnom{};
            double bat_I_mA{};
            double bat_percent_remaining{};
        } power;
    };

    struct DynamicParameters {
        Matrix6d Mv{}; // N / (m/s^s)
        Matrix6d Mv_inv{};
        Vector6d Dvv{}; // N or Nm
        Matrix6d Cv{}; // N / (m/s) or Nm / (rad/s)
        Vector6d Cvv{}; // N  or Nm
        Vector6d gRB{}; // N or Nm
        MatrixXd B{}; // Dimensionless
        Vector6d tau{}; // N or Nm
    };

    struct Inputs {
//        VectorXd w_pwm{};
        Matrix<double, 4, 1> w_pwm{};
    };

    struct Constants {

        struct Inertia {
            double mass{};
            Vector3d rG{};
            Vector3d rB{};
            double g{};
            Vector3d radius{};
            Matrix3d tensor{};
        } inertia;

        struct Damping {
            Vector3d ksurge{};
            Vector3d ksway{};
            Vector3d kheave{};
            Vector3d kroll{};
            Vector3d kpitch{};
            Vector3d kyaw{};
        } damping;

        struct Actuator {
            int count{};
            Vector3d kf{}; /* Thrust(n_Hz) coefficients*/
            Vector3d km{}; /* Torque(n_Hz) coefficients*/
            Vector3d kv{}; /* n_Hz(voltage) coefficients*/
            double L{};
            double tconst{};
            double maxsat{};
            double minsat{};
            Matrix<int, 4, 1> dir_inversion{};
            Vector3i pwm_range{};

        } actuator;

        struct Environment {
            double density{};
        } environment;

        struct Battery {
            Matrix<double, 4, 1> ki{};

            Matrix<double, 6, 1> kBd{};
            double c_vmax{};
            double c_vcut{};
            double esr{};
            double cell_n{};
            int mAh{};
        } battery;


    };


    class AUVDynamics {


    public:

        AUVDynamics();

        ~AUVDynamics() {};

        void initialize(void);

        void iterate(const double &dt);

        void setInput(const Inputs &input);

        void setInitialState(const States &initial_state);

        void setConstants(const Constants &constants);

        States getStates(void) { return _state; }

        DynamicParameters getDynamicParameters(void) { return _dparam; }

    private:

        /** Vehicle Dynamics */
        void inverse_dynamics(const double &dt);

        void inverse_kinematics(const double &dt);

        Matrix6d mass_matrix();

        Vector6d damping_vector(const Vector6d &v);

        Matrix6d coriolis_matrix(const Vector6d &v);

        Vector6d gravity_vector(const Quaterniond &q);

        Vector6d generalized_force(const VectorXd &w_hz_tr);

        VectorXd motors_transient_dynamics(const VectorXd &w_hz_sp, const double &dt);

        VectorXd thrust_n(const VectorXd &w_hz_tr);

        VectorXd torque_n(const VectorXd &w_hz_tr);

        void vnom_battery_dynamics(const VectorXd &w_hz_tr, const double &dt);

        /** dynamics variables */

        DynamicParameters _dparam{};
        States _state{};
        Constants _constants{};

        VectorXd _w_hz_sp{};
        VectorXd _w_hz_tr{};

        int _actuator_count{4};

        bool _inverse_dynamics_initialized{false};


    };

};