//
// Created by alsaibie on 5/30/18.
//
#pragma once

#include "AUVDynamics.hpp"
#include "Sensor.hpp"
#include "AttitudeController.hpp"
#include "PositionController.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <string>
#include "dynamics_math_helper.hpp"
using namespace std;

template <class T>
using param_pair = pair<string, T>;

namespace uwsim {

    template<typename T>
    inline void get_param(YAML::Node &yn, pair<string, T> &param_pair_) {
        param_pair_.second = yn[string(param_pair_.first)].as<T>();
    }

    // TODO: Move to print helper header

    const static Eigen::IOFormat CSVFormat_br(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "\n");

    const static Eigen::IOFormat CSVFormat_c(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", ",");

    inline void writeToCSVfile(ofstream &stream, Eigen::MatrixXd matrix, const Eigen::IOFormat &ioFormat) {
        stream << matrix.format(ioFormat);
    }

    inline void print_numerated_header(ofstream &stream, string prefix, int count) {

        stream << "time,";
        for (int k = 1; k < count; k++) {
            stream << prefix + to_string(k) + ",";
        }
        stream << prefix + to_string(count) + "\n";
    }


    class unit_sim {
    public:

        unit_sim();

        ~unit_sim() {};

        void start();

        void reset();

    private:

        /**
         *
         * AUV Dynamics
         *
         */

        Dynamics::AUVDynamics *_auv_dyn;

        /* node specific param */
        std::string _vehicle_name{};

        /** */
        param_pair<double> _diffq_period{"dynamics/diffq_period", {}};
        param_pair<double> _pub_period{"dynamics/pub_period", {}};

        /* simulation specific param */
        /* inertia param */
        param_pair<double> _mass{"dynamics/mass", {}};
        param_pair<vector<double>> _rG{"dynamics/gravity_center", {{}}};
        param_pair<vector<double>> _rB{"dynamics/buoyancy_center", {{}}};
        param_pair<double> _g{"dynamics/g", {}};
        param_pair<vector<double>> _radius{"dynamics/radius", {{}}};
        param_pair<vector<double>> _tensor{"dynamics/tensor", {{}}};

        /* damping param */
        param_pair<vector<double>> _ksurge_coefficients{"dynamics/damping_ksurge", {{}}};
        param_pair<vector<double>> _ksway_coefficients{"dynamics/damping_ksway", {{}}};
        param_pair<vector<double>> _kheave_coefficients{"dynamics/damping_kheave", {{}}};
        param_pair<vector<double>> _kroll_coefficients{"dynamics/damping_kroll", {{}}};
        param_pair<vector<double>> _kpitch_coefficients{"dynamics/damping_kpitch", {{}}};
        param_pair<vector<double>> _kyaw_coefficients{"dynamics/damping_kyaw", {{}}};

        param_pair<double> _density{"dynamics/density", {}};

        /* actuator param */
        param_pair<int> _num_actuators{"dynamics/num_actuators", {}};
        param_pair<vector<double>> _kF_coefficients{"dynamics/actuators_kf", {{}}};
        param_pair<vector<double>> _kM_coefficients{"dynamics/actuators_km", {{}}};
        param_pair<vector<double>> _kV_coefficients{"dynamics/actuators_kV", {{}}};
        param_pair<vector<double>> _kI_coefficients{"dynamics/actuators_kI", {{}}};
        param_pair<double> _actuators_r{"dynamics/actuators_radius", {}};
        param_pair<double> _actuators_tconst{"dynamics/actuators_tconst", {}};
        param_pair<double> _actuators_maxsat{"dynamics/actuators_maxsat", {}};
        param_pair<double> _actuators_minsat{"dynamics/actuators_minsat", {}};
        param_pair<vector<int>> _actuators_dir_inversion{"dynamics/actuators_inversion", {{}}};
        param_pair<vector<int>> _actuators_pwm_range{"dynamics/actuators_pwm", {{}}};

        /* battery param */
        param_pair<vector<double>> _kBd_coefficients{"battery/discharge_K", {{}}};
        param_pair<double> _bat_c_vmax{"battery/cell_Vmax", {}};
        param_pair<double> _bat_c_vcut{"battery/cell_Vcut", {}};
        param_pair<double> _bat_esr{"battery/ESR", {}};
        param_pair<double> _bat_cell_n{"battery/cell_n", {}};
        param_pair<int> _bat_mAh{"battery/capacity_mAh", {}};

        /* initial state param */
        param_pair<vector<double>> _p_initial{"dynamics/initial_pose", {{}}};
        param_pair<vector<double>> _v_initial{"dynamics/initial_velocity", {{}}};



        void get_set_dynamics_parameters(void);

        /**
         *
         * Sensor Dynamics
         *
         */

        Sensor::SensorDynamics *_sensor_dyn;

        param_pair<double> _hil_sensor_period_sec{"hil_sensor_period", {}};
        param_pair<double> _hil_quaternion_period_sec{"hil_quaternion_period", {}};
        param_pair<double> _hil_battery_period_sec{"hil_battery_period", {}};

        param_pair<float> _sen_accelerometer_std{"sensor/accelerometer_std", {}};
        param_pair<float> _sen_acc_noise_density{"sensor/accelerometer_noise_density", {}};
        param_pair<float> _sen_acc_bias_diffusion{"sensor/accelerometer_bias_diffusion", {}};
        param_pair<float> _sen_gyro_std{"sensor/gyro_std", {}};
        param_pair<float> _sen_gyro_noise_density{"sensor/gryo_noise_density", {}};
        param_pair<float> _sen_gyro_bias_diffusion{"sensor/gyro_bias_diffusion", {}};
        param_pair<float> _sen_mag_std{"sensor/mag_std", {}};
        param_pair<float> _sen_mag_inclination{"sensor/mag_inclination", {}};
        param_pair<float> _sen_mag_declination{"sensor/mag_declination", {}};
        param_pair<float> _sen_pressure_ref{"sensor/pressure_ref", {}};
        param_pair<float> _sen_pressure_std{"sensor/pressure_std", {}};
        param_pair<float> _sen_temp_ref{"sensor/temp_ref", {}};
        param_pair<float> _sen_temp_std{"sensor/temp_std", {}};

        param_pair<float> _att_quaternion_std{"att/quaternion_std", {}};
        param_pair<float> _att_omega_std{"att/omega_std", {}};
        param_pair<float> _att_acceleration_std{"att/acceleration_std", {}};


        void get_set_sensor_parameters(void);

        /**
         *
         * Position Controller
         *
         */
        PController::PositionController *_pos_control;

        param_pair<float> _thrust_idle{"Control/DPC_THR_IDLE", {}};
        param_pair<int>   _speed_ctrl_mode{"Control/DPC_SP_CTRL_MODE", {}};
        param_pair<float> _max_tilt_angle{"Control/DPC_MAX_TILT", {}};
        param_pair<float> _max_roll_angle{"Control/DPC_MAX_ROLL", {}};

        void get_set_pos_controller_parameters(void);

        /**
         *
         * Attitude Controller
         *
         */

        AController::AttitudeController *_att_control;

        param_pair<float> _roll_p{"Control/DP_ROLL_P",{}};
        param_pair<float> _roll_rate_p{"Control/DP_ROLLRATE_P",{}};
        param_pair<float> _roll_rate_i{"Control/DP_ROLLRATE_I",{}};
        param_pair<float> _roll_rate_integ_lim{"Control/DP_RR_INT_LIM",{}};
        param_pair<float> _roll_rate_d{"Control/DP_ROLLRATE_D",{}};
        param_pair<float> _roll_rate_ff{"Control/DP_ROLLRATE_FF",{}};
        param_pair<float> _roll_ff{"Control/DP_ROLL_FF",{}};				    /**< roll control feed-forward */
        param_pair<float> _roll_max_tau_sp{"Control/DP_RTAU_MAXSP",{}};

        param_pair<float> _pitch_p{"Control/DP_PITCH_P",{}};
        param_pair<float> _pitch_rate_p{"Control/DP_PITCHRATE_P",{}};
        param_pair<float> _pitch_rate_i{"Control/DP_PITCHRATE_I",{}};
        param_pair<float> _pitch_rate_integ_lim{"Control/DP_PR_INT_LIM",{}};
        param_pair<float> _pitch_rate_d{"Control/DP_PITCHRATE_D",{}};
        param_pair<float> _pitch_rate_ff{"Control/DP_PITCHRATE_FF",{}};
        param_pair<float> _pitch_max_tau_sp{"Control/DP_PTAU_MAXSP",{}};

        param_pair<float> _yaw_p{"Control/DP_YAW_P",{}};
        param_pair<float> _yaw_rate_p{"Control/DP_YAWRATE_P",{}};
        param_pair<float> _yaw_rate_i{"Control/DP_YAWRATE_I",{}};
        param_pair<float> _yaw_rate_integ_lim{"Control/DP_YR_INT_LIM",{}};
        param_pair<float> _yaw_rate_d{"Control/DP_YAWRATE_D",{}};
        param_pair<float> _yaw_rate_ff{"Control/DP_YAWRATE_FF",{}};
        param_pair<float> _yaw_max_tau_sp{"Control/DP_YTAU_MAXSP",{}};

        param_pair<float> _d_term_cutoff_freq{"Control/DP_DTERM_CUTOFF",{}};	/**< Cutoff frequency for the D-term filter */

        param_pair<float> _tpa_breakpoint_p{"Control/DP_TPA_BREAK_P",{}};	/**< Throttle PID Attenuation breakpoint */
        param_pair<float> _tpa_breakpoint_i{"Control/DP_TPA_BREAK_I",{}};	/**< Throttle PID Attenuation breakpoint */
        param_pair<float> _tpa_breakpoint_d{"Control/DP_TPA_BREAK_D",{}};	/**< Throttle PID Attenuation breakpoint */
        param_pair<float> _tpa_rate_p{"Control/DP_TPA_RATE_P",{}};			/**< Throttle PID Attenuation slope */
        param_pair<float> _tpa_rate_i{"Control/DP_TPA_RATE_I",{}};			/**< Throttle PID Attenuation slope */
        param_pair<float> _tpa_rate_d{"Control/DP_TPA_RATE_D",{}};			/**< Throttle PID Attenuation slope */

        param_pair<float> _roll_rate_max{"Control/DP_ROLLRATE_MAX",{}};
        param_pair<float> _pitch_rate_max{"Control/DP_PITCHRATE_MAX",{}};
        param_pair<float> _yaw_rate_max{"Control/DP_YAWRATE_MAX",{}};
        param_pair<float> _roll_auto_max{"Control/DP_ROLLRAUTO_MAX",{}};
        param_pair<float> _acro_roll_max{"Control/DP_ACRO_R_MAX",{}};
        param_pair<float> _acro_pitch_max{"Control/DP_ACRO_P_MAX",{}};
        param_pair<float> _acro_yaw_max{"Control/DP_ACRO_Y_MAX",{}};
        param_pair<float> _acro_expo_py{"Control/DP_ACRO_EXPO",{}};			/**< expo stick curve shape (pitch & yaw) */
        param_pair<float> _acro_expo_r{"Control/DP_ACRO_EXPO_R",{}};			/**< expo stick curve shape (roll) */
        param_pair<float> _acro_superexpo_py{"Control/DP_ACRO_SUPEXPO",{}};		/**< superexpo stick curve shape (pitch & yaw) */
        param_pair<float> _acro_superexpo_r{"Control/DP_ACRO_SUPEXPOR",{}};		/**< superexpo stick curve shape (roll) */

        param_pair<float> _rattitude_thres{"Control/DP_RATT_TH",{}};
        param_pair<int>   _bat_scale_en{"Control/DP_BAT_SCALE_EN",{}};

        /* Constants */
        param_pair<float> _c0_nThrust{"Control/DP_HZ_THRST_C0", {}};
        param_pair<float> _c1_nThrust{"Control/DP_HZ_THRST_C1", {}};
        param_pair<float> _c2_nThrust{"Control/DP_HZ_THRST_C2", {}};
        param_pair<float> _c2_nTorque{"Control/DP_HZ_TORQ_C2", {}};
        param_pair<float> _c0_Vn{"Control/DP_V_HZ_C0", {}};
        param_pair<float> _c1_Vn{"Control/DP_V_HZ_C1", {}};
        param_pair<float> _c2_Vn{"Control/DP_V_HZ_C2", {}};
        param_pair<float> _c0_Vn_norm{"Control/DP_V_HZ_N_C0", {}};
        param_pair<float> _c1_Vn_norm{"Control/DP_V_HZ_N_C1", {}};
        param_pair<float> _c2_Vn_norm{"Control/DP_V_HZ_N_C2", {}};
        param_pair<float> _rotor_radius{"Control/DP_ROTOR_R", {}};

        math::LowPassFilter2p _shared_lp_filters_d[3];                      /**< low-pass filters for D-term (roll, pitch & yaw) */
        static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
        float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

        void get_set_att_controller_parameters(void);

        string _config_path;
        string _data_path;

    };
};
