//
// Created by alsaibie on 5/30/18.
//

#pragma once
#include <random>
#include <Eigen/Dense>
#include "dynamics_math_helper.hpp"



namespace Sensor {
    using namespace Dynamics_Math;
    using namespace Eigen;
    // TODO: separate into input and output states
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

        struct IMU {
            Vector3d acc{};
            Vector3d gyro{};
            Vector3d mag{};
        } imu;

        struct Pressure {
            double abs{};
            double differential{};
            double altitude{};
        } pressure;

        struct Temperature {
            double Celcius{};
        } temp;

    };

    struct Parameters {

        struct Accelerometer {
            double std{};
            double noise_density{};
            double bias_diffusion{};
        } acc;

        struct Gyro {
            double std{};
            double noise_density{};
            double bias_diffusion{};
        } gyro;

        struct Magnetometer {
            double std{};
            double inclination{};
            double declination{};
        } mag;

        struct Temperature {
            double std{};
            double ref{};
        } temp;

        struct Pressure {
            double std{};
            double ref{};
        } pressure;
    };


    class SensorDynamics {
    public:
        SensorDynamics();

        ~SensorDynamics() {};

        /* Interface */
        void Initialize(void);

        void Iterate(const double &dt);

        void setInput(Sensor::States &input_state) { _input_state = input_state; }

        void setParameters(Sensor::Parameters &param) { _param = param; }

        Sensor::States getOutputStates(void) { return _output_state; }

        Sensor::States getInputStates(void) { return _input_state; }

    private:


        /* Signal deprocessing */
        void acclerometer_real(Vector3d &acc, double dt);

        void gyroscope_real(Vector3d &gyro, double dt);

        void magnetometer_real(Vector3d &mag);

        /* stochastics */
        std::default_random_engine _rand_generator;
        std::normal_distribution<double> _sen_acc_dist;
        std::normal_distribution<double> _sen_gyro_dist;
        std::normal_distribution<double> _sen_mag_dist;
        std::normal_distribution<double> _sen_pressure_dist;
        std::normal_distribution<double> _sen_temp_dist;

        Sensor::States _input_state;
        Sensor::States _output_state;
        Sensor::Parameters _param;

        /* rotation services */
        Quaterniond qmag_fu; // Magnetic inclination-declination - expressed in NED frame u

    };

};