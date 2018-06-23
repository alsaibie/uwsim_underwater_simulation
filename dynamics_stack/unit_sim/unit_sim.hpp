//
// Created by alsaibie on 5/30/18.
//
#pragma once
#include "dynamics_math_helper.hpp"
#include "AUVDynamics.hpp"
//#include "Sensor.hpp"
//#include "AttitudeController.hpp"
//#include "PositionController.hpp"
#include "yaml-cpp/yaml.h"

#include <string>

using namespace std;

template <class T>
using param_pair = pair<string, T>;

namespace uwsim{

    template<typename T>
    inline void get_param(YAML::Node &yn, pair<string, T> &param_pair_) {
        param_pair_.second = yn[string(param_pair_.first)].as<T>();
    }
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

    AUVDynamics *_auv_dyn;

    /* node specific param */
    std::string _vehicle_name {};

    /** */
    param_pair <double>             _diffq_period {"dynamics/diffq_period", {}};
    param_pair <double>             _pub_period {"dynamics/pub_period", {}};

    /* simulation specific param */
    /* inertia param */
    param_pair <double>             _mass {"dynamics/mass",{}};
    param_pair <vector<double>>     _rG {"dynamics/gravity_center",{{}}};
    param_pair <vector<double>>     _rB {"dynamics/buoyancy_center",{{}}};
    param_pair <double>             _g {"dynamics/g",{}};
    param_pair <vector<double>>     _radius {"dynamics/radius",{{}}};
    param_pair <vector<double>>     _tensor {"dynamics/tensor",{{}}};

    /* damping param */
    param_pair <vector<double>>     _damping {"dynamics/damping",{{}}};
    param_pair <vector<double>>     _quadratic_damping {"dynamics/quadratic_damping",{{}}};

    param_pair <double>             _dzv {"dynamics/dzv",{}};
    param_pair <double>             _dv {"dynamics/dv",{}};
    param_pair <double>             _dh {"dynamics/dh",{}};
    param_pair <double>             _density {"dynamics/density",{}};

    /* actuator param */
    param_pair <int>                _num_actuators {"dynamics/num_actuators",{}};
    param_pair <vector<double>>     _kF_coefficients {"dynamics/actuators_kf",{{}}};
    param_pair <vector<double>>     _kM_coefficients {"dynamics/actuators_km",{{}}};
    param_pair <vector<double>>     _kV_coefficients {"dynamics/actuators_kV",{{}}};
    param_pair <vector<double>>     _kI_coefficients {"dynamics/actuators_kI",{{}}};
    param_pair <double>             _actuators_r {"dynamics/actuators_radius",{}};
    param_pair <double>             _actuators_tconst {"dynamics/actuators_tconst",{}};
    param_pair <double>             _actuators_maxsat {"dynamics/actuators_maxsat",{}};
    param_pair <double>             _actuators_minsat {"dynamics/actuators_minsat",{}};
    param_pair <vector<int>>        _actuators_dir_inversion {"dynamics/actuators_inversion",{{}}};
    param_pair <vector<int>>        _actuators_pwm_range {"dynamics/actuators_pwm",{{}}};

    /* battery param */
    param_pair <vector<double>>     _kBd_coefficients {"battery/discharge_K",{{}}};
    param_pair <double>             _bat_c_vmax {"battery/cell_Vmax",{}};
    param_pair <double>             _bat_c_vcut {"battery/cell_Vcut",{}};
    param_pair <double>             _bat_esr {"battery/ESR",{}};
    param_pair <double>             _bat_cell_n {"battery/cell_n",{}};
    param_pair <int>                _bat_mAh {"battery/capacity_mAh",{}};

    /* initial state param */
    param_pair <vector<double>>     _p_initial {"dynamics/initial_pose",{{}}};
    param_pair <vector<double>>     _v_initial {"dynamics/initial_velocity",{{}}};

    void get_set_dynamics_parameters(void);

    /**
     *
     * Sensor Dynamics
     *
     */


    /**
     *
     * Attitude Controller
     *
     */


    /**
     *
     * Position Controller
     *
     */

    string _config_path;
    string _data_path;

    double time_scale {20.0}; // Simulation Speed up

};
