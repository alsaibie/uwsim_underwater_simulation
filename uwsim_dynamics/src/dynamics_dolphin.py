#!/usr/bin/env python

from numpy import *
import tf
# Basic ROS imports
import roslib
import rospy
import PyKDL
import sys
import rosbag
from scipy import linalg
# import msgs
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from uwsim_msgs.msg import dynamics_param
from uwsim_msgs.msg import full_state

# import services
from std_srvs.srv import Empty

roslib.load_manifest('uwsim_dynamics')


class Dynamics:
    # Utility Functions
    def s(self, x):
        """ Given a 3D vector computes the 3x3 antisymetric matrix (The cross product) """
        # rospy.loginfo("s(): \n %s", x)
        return array([[0.0, -x[2], x[1]], [x[2], 0.0, -x[0]], [-x[1], x[0], 0.0]])

    def integral(self, x_dot, x, t):
        """ Computes the integral of x dt """
        # print("shepes of xdot t and x", shape(x_dot), shape(t), shape(asarray(x)))
        # print("return shape", shape(multiply(x_dot, t) + array(x).T))
        integral = multiply(x_dot, t) + array(x).T
        # Prevent integral wind up #TODO: Check implementation, do rotational values need a different threshold?
        #integral[abs(integral) < 0.01] = 0.0
        return integral

    def mass_matrix(self):
        """ Computes the Mass Matrix = MRB + MA """
        # Inertia Tensor. Principal moments of inertia, and products of inertia [kg*m*m]
        m_br = self.tensor
        r = self.radius
        a = r[0]
        b = r[1]
        c = r[2]
        # TODO: Check why these are here?
        # Ix = self.mass*(pow(b,2)+pow(c,2))/5
        # Iy = self.mass*(pow(a,2)+pow(c,2))/5
        # Iz = self.mass*(pow(a,2)+pow(b,2))/5

        m_zeros = zeros((3, 3), float)
        m_tl = m_zeros.copy()
        fill_diagonal(m_tl, self.mass)
        # Reassign to matrix - just for clarity
        print("Sizes of m_br, m_zeros, m_tl", shape(m_br), shape(m_zeros), shape(m_tl))
        MRB = vstack((hstack((m_tl, m_zeros)), hstack((m_zeros, m_br))))
        set_printoptions(linewidth=160)
        # compute the added mass matrix for a spheroid
        # semi axes of the ellipsoid as prolate spheroid with a along x [m]
        # x^2/a^2 + y^2/b^2 + z^2/c^2 = 1

        ecc = 1 - pow(b / a, 2)
        dummy = log((1 + ecc) / (1 - ecc))
        alpha_0 = (2 * (1 - pow(ecc, 2)) * (.5 * dummy - ecc)) / (pow(ecc, 3))
        beta_0 = 1 / (pow(ecc, 2)) - ((1 - pow(ecc, 2)) * dummy) / (2 * pow(ecc, 3))
        m = self.mass
        Xud = -m * (alpha_0 / (2 - alpha_0))
        Yvd = -m * (beta_0 / (2 - beta_0))
        Zwd = Yvd
        Kpd = 0
        dummy1 = (pow(b, 2) - pow(a, 2))
        dummy2 = (alpha_0 - beta_0)
        dummy3 = (pow(b, 2) + pow(a, 2))
        Mqd = (-m / 5) * ((pow(dummy1, 2) * dummy2) / (2 * dummy1 - dummy3 * dummy2))
        Nrd = Mqd

        # TODO: Check size, Ma it should be 6x6
        Ma = diag([-Xud, -Yvd, -Zwd, -Kpd, -Mqd, -Nrd])
        # print("Ma", Ma)
        # return MRB
        return Ma + MRB

    def damping_matrix(self):
        # Linear Hydrodynamic Damping Coefficients
        Xu = self.damping[0]
        Yv = self.damping[1]
        Zw = self.damping[2]
        Kp = self.damping[3]
        Mq = self.damping[4]
        Nr = self.damping[5]

        # Quadratic Hydrodynamic Damping Coefficients
        a = self.radius[0]
        b = self.radius[1]
        Xuu = self.quadratic_damping[0]  # [Kg/m]
        Yvv = Xuu * a / b  # [Kg/m]
        Zww = Xuu * a / b  # [Kg/m]
        Kpp = self.quadratic_damping[3]  # [Kg*m*m]
        Mqq = self.quadratic_damping[4]  # [Kg*m*m]
        Nrr = self.quadratic_damping[5]  # [Kg*m*m]
        # rospy.logdebug(shape(self.v))
        d = diag([-Xu - Xuu * abs(self.v[0]),
                  -Yv - Yvv * abs(self.v[1]),
                  -Zw - Zww * abs(self.v[2]),
                  -Kp - Kpp * abs(self.v[3]),
                  -Mq - Mqq * abs(self.v[4]),
                  -Nr - Nrr * abs(self.v[5])])
        return d

    def coriolis_matrix(self):
        """ Compute the coriolis matrix C = CA + CRB """
        # TODO: Fix syntax and use built in cross product
        dot1 = squeeze(asarray(dot(self.M[0:3, 0:3], self.v[0:3]) + dot(self.M[0:3, 3:6], self.v[3:6])))
        dot2 = squeeze(asarray(dot(self.M[3:6, 0:3], self.v[0:3]) + dot(self.M[3:6, 3:6], self.v[3:6])))
        s1 = self.s(dot1)
        s2 = self.s(dot2)
        c = zeros((6, 6))
        c[0:3, 3:6] = -s1
        c[3:6, 0:3] = -s1
        c[3:6, 3:6] = -s2
        return c

    def gravity(self):
        """ Computes the gravity and buoyancy forces. For a spheroid - this only matters if the mAUV is not neutrally
         buoyant """
        # Weight and Flotability
        W = self.mass * self.g  # [Kg]
        # Assume the vehicle is always submerged
        r = self.radius
        [q1, q2, q3, q4] = self.p_q[3:7] #TODO: change to qi qj qk qr

        #      or define common models and let the user choose one by the name
        #      Eventually let this part to bullet inside uwsim (HfFluid)
        # TODO: Incorporate the displaced water volume in here to compute B, for now assume neutral buoyancy
        # B = ((4 * math.pi * r[0] * r[1] * r[2]) / 3) * self.density * self.g
        B = W
        # gravity center position in the robot fixed frame (x',y',z') [m]
        # See equation 2.48
        xg = self.rG[0]
        yg = self.rG[1]
        zg = self.rG[2]
        xb = self.rB[0]
        yb = self.rB[1]
        zb = self.rB[2]

        q1_2 = power(q1, 2)
        q2_2 = power(q2, 2)
        q3_2 = power(q3, 2)
        q4_2 = power(q4, 2)
        q1q3 = q1 * q3
        q2q3 = q2 * q3
        q4q1 = q4 * q1
        q4q2 = q4 * q2
        term1 = (q4q2 - q1q3)
        term2 = (q4q1 + q2q3)
        term3 = (-q4_2 + q1_2 + q2_2 - q3_2)
        g = array([
            - 2 * term1 * (W - B),
            2 * term2 * (W - B),
            -term3 * (W - B),
            -term3 * (yg * W - yb * B) - 2 * term2 * (zg * W - zb * B),
            +term3 * (xg * W - xb * B) - 2 * term1 * (zg * W - zb * B),
            +2 * term2 * (xg * W - xb * B) + 2 * term1 * (yg * W - yb * B)
        ])

        return g

    def battery_discharge(self):
        # Update remaining charge voltage based on consumed mAh and instantaneous current draw
        mAh_consumed_percentage = self.bat_mAh_consumed * 100 / self.bat_mAh
        mV_ = self.bat_cell_n * (self.bat_discharge_Vd_poly(mAh_consumed_percentage) - self.bat_mA_draw * self.bat_esr / 1000)
        self.bat_Vnom = float(mV_) / 1000

    def battery_dynamics(self):
        """ compute current draw rate based on motor dynamics - must be called at T = self.diffq_period """
        self.bat_mA_draw = sum(self.current_draw_In_poly(abs(self.w_hz_tr)))  # sum total current draw from all actuators
        self.bat_mAh_consumed = self.bat_mAh_consumed + (self.bat_mA_draw * self.diffq_period) / 3600
        # print(self.bat_mAh_consumed)
        self.battery_discharge()

    def motors_transient_dynamics(self, w_hz_sp):
        """ This captures the motor transients:
        Difference eq from diffq y[k] = (Kp*u *dt + y[k-1] *tau) / ( tau + dt) : period is too low that this becomes
        accurate compared to difference from z-transform"""
        w_hz_tr = zeros(size(w_hz_sp))
        for i in range(size(w_hz_sp)):
            w_hz_tr[i] = (self.diffq_period * w_hz_sp[i] + self.actuators_tconst * self.w_hz_tr[i]) / (self.diffq_period + self.actuators_tconst)
        self.w_hz_tr_1 = w_hz_tr
        # print("Motor speed after transients", y)
        return w_hz_tr

    def thrust_n(self, w_hz_tr):
        """ Compute the Thrust in N given speed of rotor in Hz
        Return a column vector
        Apply the thruster dead-zone truncation here
        """
        kf2, kf1, kf0 = self.kF_coefficients
        tn = zeros(len(w_hz_tr), dtype=float)
        # TODO: use poly function
        for k, u in enumerate(w_hz_tr):
            if u >= 0:
                tn[k] = kf2 * u * u + kf1 * u + kf0
                if tn[k] < 0.0:
                    tn[k] = 0.0
            else:
                u = -u
                tn[k] = -(kf2 * u * u + kf1 * u + kf0)
                if tn[k] > 0.0:
                    tn[k] = 0.0
        # Convert N/1000 to N
        return tn / 1000

    def torque_n(self, w_hz_tr):
        """ Compute the Torque in N.mm given speed of rotor in Hz
        Return a column vector
        Apply the thruster dead-zone truncation here
        """
        km2, km1, km0 = self.kF_coefficients
        tn = zeros(len(w_hz_tr), dtype=float)
        # TODO: use poly function
        for k, u in enumerate(w_hz_tr):
            if u >= 0:
                tn[k] = km2 * u * u + km1 * u + km0
                if tn[k] < 0.0:
                    tn[k] = 0.0
            else:
                u = -u
                tn[k] = -(km2 * u * u + km1 * u + km0)
                if tn[k] > 0.0:
                    tn[k] = 0.0
        # Convert Nmm / 1000 to Nm
        return tn / (1000 * 1000)

    def generalized_force(self, w_hz_tr):
        """ Computes the generalized force as B*Tn, and Tn = [T(u) Torque(u)] being B the allocation matrix,
        u the control input
        Return a 6x1 matrix"""
        # Compute the Torque/Thrust Array - an 8x1 array

        T_n = hstack((self.thrust_n(w_hz_tr), self.torque_n(w_hz_tr)))  # 2nx1
        # Apply the thruster allocation matrix B to compute tau  - a 6x1 array
        return self.B.dot(T_n)

    # Update callbacks
    def update_thrusters(self, thrusters):
        """Receives the control input, the control input should be in PWM format, ranging from 1100 to 1900 with 1500
         as zero, 1100 as max negative, 1900 as max positive. This is the output of the pwm mixer, which should take
         care of any saturation and gain setting, Here it is scaled based on the thruster model to output w in
         Hz
        """
        w_pwm = array(thrusters.data)

        # Convert PWM to Voltage TODO: Update the model such that it captures voltage drop over time for a LiPO,
        # to do that I need to capture current consumption dynamics which I have the coefficients for I(n): Amp(Hz) -
        #  which should be a power consumption cumulative function that updates at a given sample time then updates
        # Vactual which is then used here instead. And not only here, but fed back to the controller so that the
        # voltage sent is adjusted accordingly.
        min_pwm, mid_pwm, max_pwm = self.actuators_pwm
        kv2, kv1, kv0 = self.kV_coefficients
        w_hz = array(zeros(len(w_pwm)))
        for k, w_ in enumerate(w_pwm):
            w_voltage = 2 * self.bat_Vnom * (w_ - mid_pwm) / (max_pwm - min_pwm)
            # Voltage to Hz n(V)
            if w_voltage >= 0:
                w_hz[k] = (kv2 * w_voltage * w_voltage + kv1 * w_voltage + kv0) / 1000
            else:
                w_hz[k] = - (kv2 * w_voltage * w_voltage - kv1 * w_voltage + kv0) / 1000
        self.w_hz_sp = w_hz

    def inverse_dynamics(self):
        """ Given the setpoint for each thruster, the previous velocity and the
            previous position computes the v_dot """

        self.d_ = self.damping_matrix()
        self.c_ = self.coriolis_matrix()
        self.g_ = self.gravity()
        self.w_hz_tr = self.motors_transient_dynamics(self.w_hz_sp)  # Apply First Order Delay
        self.tau_ = self.generalized_force(self.w_hz_tr)  # Apply thruster dynamics and allocation matrix
        self.c_v_ = dot(self.c_ + self.d_, self.v).T
        # self.c_v_ = dot(self.d_, self.v).T
        # v_dot = dot(self.invM, (self.tau_ - c_v - self.g_ + self.collision_force))  # inv(M)*(tau-c_v-g+collisionForce)
        # print("Shape invM", shape(self.invM))
        # print("Shape tau", shape(self.tau_))
        # print("Shape c_v_", shape(self.c_v_))
        # print("Shape g_", shape(self.g_))
        v_dot = dot(self.invM, (self.tau_ + self.c_v_ + self.g_).T) # inv(M)*(tau-c_v-g+collisionForce)
        # v_dot = dot(self.invM, (self.tau_ + self.c_v_).T) # inv(M)*(tau-c_v-g+collisionForce)

        self.v_dot = asarray(v_dot)  # Transforms a matrix into an array
        # self.collision_force = [0, 0, 0, 0, 0, 0]   # TODO: Fix. Why is this zeroed here?

        if self.invers_dyanmics_intialized is False:
            self.invers_dyanmics_intialized = True
        self.v = self.integral(self.v_dot, self.v, self.diffq_period)

        self.state24_msg.v = self.v
        self.state24_msg.v_dot = self.v_dot

        return self.v

    def inverse_jacobian(self, p):
        """ Generate the inverse Jacobian for calculating p_dot = Je_inv*v"""
        # Generated using the awesome SymPy, Je = [[bRi 0], [0 Jko]] and this is the inverse of that
        [roll_, pitch_, yaw_] = p[3:6]
        return array([[cos(pitch_)*cos(yaw_),
                   sin(pitch_)*sin(roll_)*cos(yaw_) - sin(yaw_)*cos(roll_),
                   sin(pitch_)*cos(roll_)*cos(yaw_) + sin(roll_)*sin(yaw_), 0, 0, 0],
                  [sin(yaw_)*cos(pitch_),
                   sin(pitch_)*sin(roll_)*sin(yaw_) + cos(roll_)*cos(yaw_),
                   sin(pitch_)*sin(yaw_)*cos(roll_) - sin(roll_)*cos(yaw_), 0, 0, 0],
                  [-sin(pitch_), sin(roll_)*cos(pitch_), cos(pitch_)*cos(roll_), 0, 0,
                   0],
                  [0, 0, 0, 1, sin(roll_)*tan(pitch_), cos(roll_)*tan(pitch_)],
                  [0, 0, 0, 0, cos(roll_), -sin(roll_)],
                  [0, 0, 0, 0, sin(roll_)/cos(pitch_), cos(roll_)/cos(pitch_)]], dtype=float)

    def inverse_jacobian_q(self, p_q):
        """ Generate the inverse Jacobian using quaternions for calculating p_dot_q = Je_inv*v"""
        [qi, qj, qk, qr] = p_q[3:7]
        return array([[-2*qj**2 - 2*qk**2 + 1, 2*qi*qj - 2*qk*qr, 2*qi*qk + 2*qj*qr, 0,
                      0, 0],
                     [2*qi*qj + 2*qk*qr, -2*qi**2 - 2*qk**2 + 1, -2*qi*qr + 2*qj*qk, 0,
                      0, 0],
                     [2*qi*qk - 2*qj*qr, 2*qi*qr + 2*qj*qk, -2*qi**2 - 2*qj**2 + 1, 0,
                      0, 0],
                     [0, 0, 0, 0.5*qr, -0.5*qk, 0.5*qj],
                     [0, 0, 0, 0.5*qk, 0.5*qr, -0.5*qi],
                     [0, 0, 0, -0.5*qj, 0.5*qi, 0.5*qr],
                     [0, 0, 0, -0.5*qi, -0.5*qj, -0.5*qk]],   dtype=float)

    def iRb_q(self, p_q):
        """ Generate the SO(3) rotation matrix from body to inertial frame, using quaternion as input"""
        [qi, qj, qk, qr] = p_q[3:7]
        return array([[-2*qj**2 - 2*qk**2 + 1, 2*qi*qj - 2*qk*qr, 2*qi*qk + 2*qj*qr],
                  [2*qi*qj + 2*qk*qr, -2*qi**2 - 2*qk**2 + 1, -2*qi*qr + 2*qj*qk],
                  [2*qi*qk - 2*qj*qr, 2*qi*qr + 2*qj*qk, -2*qi**2 - 2*qj**2 + 1]], dtype=float)

    def exp_wq(self, v, dt):
        """ Compute the exponential e^(Adt/2)=Exp{w dt}=q{w dt}"""
        # Norm of v
        v_ = v[3:6]
        vn = linalg.norm(v_)
        # This can be singular, but we know when.
        # print("vn", vn)
        if vn < 0.0001:
            return array([0, 0, 0, 1])
        else:
            cs = cos(vn*dt/2)
            sn_n = sin(vn*dt/2)/vn
            return array([v_[0]*sn_n, v_[1]*sn_n, v_[2]*sn_n, cs])

    def kinematics(self):
        """ Transform body fixed velocity to inertial reference frame,
        rpy_rate need to be expressed in inertial f. to integrate ref Antonelli eq2.21 """

        # Calculate p_dot_translation with a standard SO(3) rotation matrix
        Rinv = self.iRb_q(self.p_q)
        q_dot_translation_q = dot(Rinv, self.v[0:3])
        self.p_q[0:3] = self.integral(q_dot_translation_q, self.p_q[0:3], self.diffq_period)
        # Calculate the updated position based on quaternion integration
        expq = self.exp_wq(self.v, self.diffq_period)
        self.p_q[3:7] = tf.transformations.quaternion_multiply(self.p_q[3:7], expq)
        # Convert back to euler form
        self.p_e[0:3] = self.p_q[0:3]
        self.p_e[3:6] = tf.transformations.euler_from_quaternion(self.p_q[3:7], 'rxyz')
        # print("exp", expq)

        self.state24_msg.p_q = self.p_q
        self.state24_msg.p_lin_dot = q_dot_translation_q

        return self.p_e

    def update_collision(self, force):
        self.colon_force = [force.wrench.force.x, force.wrench.force.y, force.wrench.force.z, force.wrench.torque.x,
                            force.wrench.torque.y, force.wrench.torque.z]

    def pubPose(self, event):
        pose = Pose()
        pose.position.x = self.p_e[0]
        pose.position.y = self.p_e[1]
        pose.position.z = self.p_e[2]

        pose.orientation.x = self.p_q[0]
        pose.orientation.y = self.p_q[1]
        pose.orientation.z = self.p_q[2]
        pose.orientation.w = self.p_q[3]

        self.pub_pose.publish(pose)

        # Broadcast transform
        br = tf.TransformBroadcaster()
        br.sendTransform((self.p_e[0], self.p_e[1], self.p_e[2]), self.p_q[3:7],
                         rospy.Time.now(), "world", str(self.frame_id))

        if self.invers_dyanmics_intialized is True:
            dynamics_msg = dynamics_param()
            # dynamics_msg.mass_matrix = squeeze(self.M.flatten('C')).tolist()
            dynamics_msg.mass_matrix = self.M.astype(float).flatten().tolist()
            dynamics_msg.damping_matrix = self.d_.astype(float).flatten().tolist()
            dynamics_msg.coriolis_matrix = self.c_.astype(float).flatten().tolist()
            dynamics_msg.c_v = self.c_v_.astype(float).flatten().tolist()
            dynamics_msg.gravity = self.g_.astype(float).flatten().tolist()
            dynamics_msg.motors_transient_dynamics = self.w_hz_tr.astype(float).flatten().tolist()
            dynamics_msg.tau = self.tau_.astype(float).flatten().tolist()
            dynamics_msg.v_dot = self.v_dot.astype(float).flatten().tolist()
            dynamics_msg.v = self.v.astype(float).flatten().tolist()
            dynamics_msg.p_q = self.p_q.astype(float).flatten().tolist()
            dynamics_msg.p = self.p_e.astype(float).flatten().tolist()
            dynamics_msg.mA_draw = self.bat_mA_draw
            dynamics_msg.mAh_consumed = self.bat_mAh_consumed
            dynamics_msg.Vnom = self.bat_Vnom
            dynamics_msg.time_secs = float(self.t)
            # print("mass matrix", dynamics_msg.mass_matrix)
            self.pub_dyn_param.publish(dynamics_msg)

            self.dyn_bag.write('dynamics param', dynamics_msg)

    def pubIMU(self, event):
        """ Generate Virtual IMU data, with noise and biases """
        if self.invers_dyanmics_intialized is True:
            imu_msg = Imu()
            # Orientation w.r.t to world frame - p TODO: VERIFY
            euler_noise = random.normal(0, self.imu_q_std, 3)
            p_w_noise = self.p_e[3:6] + euler_noise

            quaternion = tf.transformations.quaternion_from_euler(p_w_noise[0], p_w_noise[1], p_w_noise[2])

            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            # rospy.loginfo("Pwnoise %s", imu_msg.orientation)
            imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = \
                imu_msg.orientation_covariance[8] = pow(self.imu_q_std, 2)

            # Angular Velocity w.r.t body frame - v
            w_noise = random.normal(0, self.imu_w_std, 3)
            imu_msg.angular_velocity.x = self.v[3] + w_noise[0]
            imu_msg.angular_velocity.y = -self.v[4] + w_noise[1]
            imu_msg.angular_velocity.z = self.v[5] + w_noise[2]
            imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = \
                imu_msg.angular_velocity_covariance[8] = pow(self.imu_w_std, 2)

            # Linear Acceleration w.r.t world frame  - # TODO: Populate!
            # a_noise = random.normal(0, self.imu_a_std, 3)
            # imu_msg.linear_acceleration.x = ... + a_noise[0]
            # imu_msg.linear_acceleration.y = ..  + a_noise[1]
            # imu_msg.linear_acceleration.z = .. .+ a_noise[2]
            # imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = \
            # imu_msg.linear_acceleration_covariance[8] = pow(self.imu_a_std, 2)
            self.pub_imu.publish(imu_msg)

    def compute_tf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
        frame = PyKDL.Frame(r, v)
        return frame

    def reset(self, req):
        self.v = self.v_0
        self.p_e = self.p_0
        self.p_q = self.p_q_0
        return []

    def __init__(self):
        """ Simulates the dynamics of an AUV """

        if len(sys.argv) != 6:
            sys.exit("Usage: " + sys.argv[0] + " <namespace> <input_topic> <output_topic>")

        # Workspace arguments
        self.namespace = sys.argv[1]
        self.vehicle_name = self.namespace
        self.input_topic = sys.argv[2]
        self.output_topic = sys.argv[3]
        #  Initialize Collision
        self.collision_force = [0, 0, 0, 0, 0, 0]

        """ Load dynamic parameters """

        # time properties
        self.diffq_period = rospy.get_param(self.vehicle_name + "/dynamics" + "/diffq_period")
        self.uwsim_period = rospy.get_param(self.vehicle_name + "/dynamics" + "/uwsim_period")

        # IMU Properties
        self.output_imu_topic = self.vehicle_name + "/imu_dyn"
        self.imu_period = rospy.get_param(self.vehicle_name + "/imu" + "/period")
        self.imu_q_std = rospy.get_param(self.vehicle_name + "/imu" + "/quaternion_std")
        self.imu_w_std = rospy.get_param(self.vehicle_name + "/imu" + "/omega_std")
        self.imu_a_std = rospy.get_param(self.vehicle_name + "/imu" + "/acceleration_std")

        # Inertia Properties
        self.mass = rospy.get_param(self.vehicle_name + "/dynamics" + "/mass")
        self.rG = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/gravity_center"))
        self.rB = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/buoyancy_center"))
        self.g = rospy.get_param(self.vehicle_name + "/dynamics" + "/g")
        self.radius = rospy.get_param(self.vehicle_name + "/dynamics" + "/radius")
        self.tensor = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/tensor"), dtype=float)

        # Damping Properties
        self.damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/damping"), dtype=float)
        self.quadratic_damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/quadratic_damping"),
                                       dtype=float)

        self.dzv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dzv")
        self.dv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dv")
        self.dh = rospy.get_param(self.vehicle_name + "/dynamics" + "/dh")
        self.density = rospy.get_param(self.vehicle_name + "/dynamics" + "/density")

        # Actuator Properties
        self.num_actuators = rospy.get_param(self.vehicle_name + "/num_actuators")
        self.kF_coefficients = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_kf"), dtype=float)
        self.kM_coefficients = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_km"), dtype=float)
        self.kV_coefficients = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_kV"), dtype=float)
        self.kI_coefficients = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_kI"), dtype=float)
        self.actuators_r = float(rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_radius"))
        self.actuators_tconst = float(rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_tconst"))
        self.actuators_maxsat = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_maxsat")
        self.actuators_minsat = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_minsat")
        self.actuators_inv = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_inversion")
        self.actuators_pwm = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_pwm"))

        # Battery Properties
        self.bat_c_vmax = float(rospy.get_param(self.vehicle_name + "/battery" + "/cell_Vmax"))
        self.bat_c_vcut = float(rospy.get_param(self.vehicle_name + "/battery" + "/cell_Vcut"))
        self.bat_esr = float(rospy.get_param(self.vehicle_name + "/battery" + "/ESR"))
        self.bat_discharge_k = array(rospy.get_param(self.vehicle_name + "/battery" + "/discharge_K"), dtype=float)
        self.bat_cell_n = float(rospy.get_param(self.vehicle_name + "/battery" + "/cell_n"))
        self.bat_mAh = rospy.get_param(self.vehicle_name + "/battery" + "/capacity_mAh")

        # Initialization
        self.p_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_pose"), dtype=float)
        self.v_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_velocity"), dtype=float)
        self.frame_id = rospy.get_param(self.vehicle_name + "/dynamics" + "/frame_id")
        #   Init pose and velocity and period
        self.p_q_0 = zeros(7)  # Expressed with quaternions [x y z qi qj qk qr]
        self.p_q_0[0:3] = self.p_0[0:3]
        self.p_q_0[3:7] = tf.transformations.quaternion_from_euler(self.p_0[3], self.p_0[4], self.p_0[5], 'rxyz')
        self.p_e = self.p_0  # The normal euler convention [x y z roll pitch yaw]
        self.p_q = self.p_q_0
        self.v = self.v_0
        self.p_dot = zeros(6)
        self.v_dot = zeros(6)
        # Topics
        self.external_force_topic = rospy.get_param(self.vehicle_name + "/dynamics" + "/external_force_topic")
        """"""

        # self.altitude = -1.0
        self.y_1 = zeros(5)

        # internal variables for inverse dynamics
        self.invers_dyanmics_intialized = False
        self.t = 0
        self.d_ = []
        self.c_ = []
        self.c_v_ = []
        self.g_ = []

        self.tau_ = []
        self.M = []

        # Full State Handler
        self.state24_msg = full_state()
        #   Create publisher
        self.pub_pose = rospy.Publisher(self.output_topic, Pose, queue_size=10)
        self.pub_imu = rospy.Publisher(self.output_imu_topic, Imu, queue_size=10)
        self.pub_dyn_param = rospy.Publisher("/dolphin/dynamics/dynamics_parammmm", dynamics_param, queue_size=10)
        self.pub_state24 = rospy.Publisher("/dolphin/dynamics/full_state", full_state, queue_size=10)
        rospy.init_node("dynamics_" + self.vehicle_name, log_level=rospy.DEBUG)

        # Setup the battery model
        self.bat_Vnom = self.bat_cell_n * self.bat_c_vmax  # initial charged voltaged
        self.bat_mAh_consumed = 0
        self.bat_mA_draw = 0
        self.bat_discharge_Vd_poly = poly1d(self.bat_discharge_k)  # Polynomial Fit Function

        # Compute the Mass Matrix

        self.M = self.mass_matrix()
        self.invM = linalg.inv(self.M)
        rospy.logdebug("Mass Matrix M: \n %s", self.M)
        rospy.logdebug("Initial Position p: \n %s", self.p_e)
        rospy.logdebug("Initial Velocity v: \n %s", self.v)

        # Thruster Speeds
        self.w_hz_sp = zeros(self.num_actuators)  # Set Point
        self.w_hz_tr = zeros(self.num_actuators)  # Transient
        # Thruster Force/Torque - column vector
        self.Tn = zeros(self.num_actuators * 2)
        self.current_draw_In_poly = poly1d(self.kI_coefficients)
        # TODO: add the other polyfits as functions here
        # Compute the thruster B Matrix
        L = self.actuators_r


        # x forward z downward convention (y right)
        # Thrust on x axis, roll around x axis, pitch around y axis and yaw around z axis
        # -- 3  CW -- 1 CCW --
        # ---------\/---------
        # ---------/\---------
        # -- 2 CCW -- 4  CW --
        # The direction should be set by the mixer and the thrust / torque should be calculated in the generalized
        # force call, this matrix should be the geometric allocation matrix tau (6x1)= Bu (6x8)(8x1)
        # where u = [T1 T2 T3 T4 Tor1 Tor2 Tor3 Tor4]^T
        # Negative sign to account for reversed motors 3 & 4, motor speed sign should determine flow direction.
        # Positive speed sign means the thrust is pushing the vehicle in positive surge direction. This implies
        # that 2 motors rotate in the opposite direction and with mirrored impellers. The torque direction must be
        # carried in the B matrix, so we need to know the rot direction there as well to compute the roll torque

        a_inv = self.actuators_inv
        # TODO: Move this matrix to the yaml file, since it's based on pure geometric configuration
        self.B = array([[1, 1, 1, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 1*a_inv[0], 1*a_inv[1], 1*a_inv[2], 1*a_inv[3]],
                        [-L, L, -L, L, 0, 0, 0, 0],
                        [-L, L, L, -L, 0, 0, 0, 0], ])

        rospy.logdebug("Thruster Matrix B: \n %s", self.B)
        # Publish pose to UWSim at a set period rate - 200Hz?
        rospy.Timer(rospy.Duration(self.uwsim_period), self.pubPose)
        # Publish IMU at 400 Hz
        rospy.Timer(rospy.Duration(self.imu_period), self.pubIMU)
        # TODO: Check if time sample is done correctly
        #   Create Subscribers for thrusters and collisions
        # TODO: set the topic names as parameters
        rospy.Subscriber(self.input_topic, Float64MultiArray, self.update_thrusters)
        rospy.Subscriber(self.external_force_topic, WrenchStamped, self.update_collision)

        s = rospy.Service('/dynamics/reset', Empty, self.reset)

        self.dyn_bag = rosbag.Bag('/tmp/dyn_test.bag', 'w')
        self.t0 = rospy.Time.now()

    def iterate(self):
        t1 = rospy.Time.now()
        # Main loop operations

        # Compute body velocity
        self.inverse_dynamics()
        # Then propagate inertial pose
        self.kinematics()
        # And update the battery dynamics
        self.battery_dynamics()
        t2 = rospy.Time.now()
        self.t = (t2 - self.t0).to_sec()
        p = self.diffq_period - (t2 - t1).to_sec()

        # Publish Full State
        self.pub_state24.publish(self.state24_msg)
        if p < 0.0: p = 0.0
        rospy.sleep(p)


if __name__ == '__main__':
    try:
        dynamics = Dynamics()
        dynamics.t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            dynamics.iterate()
        dynamics.dyn_bag.close()

    except rospy.ROSInterruptException:
        pass
