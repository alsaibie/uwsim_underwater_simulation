# Full State Vector x = [p_q pdot v vdot]
std_msgs/Header header
float64[7] p_q    # Quaternion Attitude w.r.t I Frame
float64[3] p_lin_dot # Linear Velocity w.r.t I Frame
float64[6] v    # Velocity w.r.t B Frame
float64[6] v_dot # Acceleration w.r.t B Frame
