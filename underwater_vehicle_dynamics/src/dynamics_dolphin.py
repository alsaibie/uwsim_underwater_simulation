#!/usr/bin/env python

from numpy import *
import tf
# Basic ROS imports
import roslib
import rospy
import PyKDL
import sys
import rosbag

# import msgs
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from underwater_vehicle_dynamics.msg import dynamics_param

# import services
from std_srvs.srv import Empty

roslib.load_manifest('underwater_vehicle_dynamics')

class Dynamics:
    # Utility Functions
    def s(self, x):
        """ Given a 3D vector computes the 3x3 antisymetric matrix (The cross product) """
        # rospy.loginfo("s(): \n %s", x)

        ret = array([0.0, -x[2], x[1], x[2], 0.0, -x[0], -x[1], x[0], 0.0])
        return ret.reshape(3, 3)

    def integral(self, x_dot, x, t):
        """ Computes the integral o x dt """
        return (x_dot * t) + x

    def mass_matrix(self):
        """ Computes the Mass Matrix = MRB + MA """
        # Inertia Tensor. Principal moments of inertia, and products of inertia [kg*m*m]
        Ixx = float(self.tensor[0])
        Ixy = float(self.tensor[1])
        Ixz = float(self.tensor[2])
        Iyx = float(self.tensor[3])
        Iyy = float(self.tensor[4])
        Iyz = float(self.tensor[5])
        Izx = float(self.tensor[6])
        Izy = float(self.tensor[7])
        Izz = float(self.tensor[8])
        r = self.radius
        a = r[0]
        b = r[1]
        c = r[2]


        Ix  = self.mass*(pow(b,2)+pow(c,2))/5
        Iy  = self.mass*(pow(a,2)+pow(c,2))/5
        Iz  = self.mass*(pow(a,2)+pow(b,2))/5

        m_zeros = zeros((3, 3), float)
        m_tl = m_zeros
        fill_diagonal(m_tl, self.mass)
        # Reassign to matrix - just for clarity
        m_br = matrix([[Ixx, Ixy, Ixz], [Iyx, Iyy, Iyz], [Izx, Izy, Izz]])
        # m_br = diag([Ix, Iy, Iz])
        m_zeros = zeros((3, 3), float)
        MRB = bmat([[m_tl, m_zeros], [m_zeros, m_br]])
        set_printoptions(linewidth=160)
        # compute the added mass matrix for a spheroid
        # semi axes of the ellipsoid as prolate spheroid with a along x [m]
        # x^2/a^2 + y^2/b^2 + z^2/c^2 = 1

        ecc = 1 - pow(b/a, 2)
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

        MA = diag([-Xud, -Yvd, -Zwd, -Kpd, -Mqd, -Nrd])
        print(MA)
        # return MRB
        return MA + MRB

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
        Xuu = self.quadratic_damping[0]     # [Kg/m]
        Yvv = Xuu*a/b                       # [Kg/m]
        Zww = Xuu*a/b                       # [Kg/m]
        Kpp = self.quadratic_damping[3]  # [Kg*m*m]
        Mqq = self.quadratic_damping[4]  # [Kg*m*m]
        Nrr = self.quadratic_damping[5]  # [Kg*m*m]

        d = diag([-Xu - Xuu * abs(self.v[0]),
                  -Yv - Yvv * abs(self.v[1]),
                  -Zw - Zww * abs(self.v[2]),
                  -Kp - Kpp * abs(self.v[3]),
                  -Mq - Mqq * abs(self.v[4]),
                  -Nr - Nrr * abs(self.v[5])])
        return d

    def coriolis_matrix(self):
        """ Compute the coriolis matrix C = CA + CRB """
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
        quaternion = tf.transformations.quaternion_from_euler(self.p[3], self.p[4], self.p[5])
        q1 = quaternion[0]
        q2 = quaternion[1]
        q3 = quaternion[2]
        q4 = quaternion[3]

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

    def thrusters_dynamics(self, u):
        y = zeros(size(u))
        for i in range(size(u)):
            y[i] = (self.period * u[i] + self.actuators_tconst * self.y_1[i]) / (self.period + self.actuators_tconst)
        self.y_1 = y
        return y

    def generalized_force(self, du):
        """ Computes the generalized force as B*u, being B the allocation matrix and u the control input """

        b = self.B
        # tau = generalized force
        du = multiply(abs(du),du)
        tau = dot(b, du)
        tau = squeeze(asarray(tau))  # Transforms a matrix into an array
        return tau

    # Update callbacks
    def update_thrusters(self, thrusters):
        """Receives the control input, the control input should be in PWM format, ranging from 1100 to 1900 with 1500
         as zero, 1100 as max negative, 1900 as max positive. This is the output of the pwm mixer, which should take
         care of any saturation and gain setting, Here it is scaled based on the thruster model to output w in
         radians/s
        """
        w_pwm = array(thrusters.data)
        #  convert pwm to rad/s based on nominal voltage and motor kv
        # TODO: the scaling of pwm to output in the mixer should be turned off,
        # because that's taking into account the nonlinear behvavior of w(pwm) function.
        # Or keep it and invert the affect here.

        w_rad = (w_pwm - 1500) / 400 * self.kV * self.actuators_refV * 2 * pi / 60 * self.actuators_loadf
        w_rad = squeeze(asarray(w_rad))
        # Flip motor signs
        w_rad = multiply(self.actuators_inv, w_rad)
        self.u = w_rad

    def inverse_dynamics(self):
        """ Given the setpoint for each thruster, the previous velocity and the
            previous position computes the v_dot """
        self.d_ = self.damping_matrix()
        self.c_ = self.coriolis_matrix()
        self.g_ = self.gravity()
        self.du_ = self.thrusters_dynamics(self.u)
        self.tau_ = self.generalized_force(self.du_)
        # c_v = dot((self.c_ + self.d_), self.v)
        self.c_v_ = dot(self.c_ + self.d_, self.v)
        # v_dot = dot(self.invM, (self.tau_ - c_v - self.g_ + self.collision_force))  # inv(M)*(tau-c_v-g+collisionForce)
        v_dot = dot(self.invM, (self.tau_ - self.c_v_ - self.g_))  # inv(M)*(tau-c_v-g+collisionForce)
        self.v_dot_ = squeeze(asarray(v_dot))  # Transforms a matrix into an array
        # self.collision_force = [0, 0, 0, 0, 0, 0]   # TODO: Fix. Why is this zeroed here?
        if self.invers_dyanmics_intialized is False:
            self.invers_dyanmics_intialized = True
        return self.v_dot_

    def kinematics(self):
        """ Transform the velocity to current position to represent p_dot """
        roll = self.p[3]
        pitch = self.p[4]
        yaw = self.p[5]

        rec = [cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
               sin(yaw) * sin(roll) + cos(yaw) * cos(roll) * sin(pitch),
               sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(roll) * sin(pitch) * sin(yaw),
               -cos(yaw) * sin(roll) + sin(pitch) * sin(yaw) * cos(roll),
               -sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll)]

        rec = array(rec).reshape(3, 3)

        to = [1.0, sin(roll) * tan(pitch), cos(roll) * tan(pitch),
              0.0, cos(roll), -sin(roll),
              0.0, sin(roll) / cos(pitch), cos(roll) / cos(pitch)]

        to = array(to).reshape(3, 3)

        p_dot = zeros(6)
        # Translate
        p_dot[0:3] = dot(rec, self.v[0:3])
        # Rotate
        p_dot[3:6] = dot(to, self.v[3:6])

        return p_dot



    def update_collision(self, force):
        self.colon_force = [force.wrench.force.x, force.wrench.force.y, force.wrench.force.z, force.wrench.torque.x,
                                force.wrench.torque.y, force.wrench.torque.z]

    def pubPose(self, event):
        pose = Pose()
        pose.position.x = self.p[0]
        pose.position.y = self.p[1]
        pose.position.z = self.p[2]

        orientation = tf.transformations.quaternion_from_euler(self.p[3], self.p[4], self.p[5], 'sxyz')
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        self.pub_pose.publish(pose)


        # Broadcast transform
        br = tf.TransformBroadcaster()
        br.sendTransform((self.p[0], self.p[1], self.p[2]), orientation,
                         rospy.Time.now(), "world", str(self.frame_id))

        if self.invers_dyanmics_intialized is True:
            dynamics_msg = dynamics_param()
            # dynamics_msg.mass_matrix = squeeze(self.M.flatten('C')).tolist()
            dynamics_msg.damping_matrix = squeeze(self.d_.flatten('C')).tolist()
            dynamics_msg.coriolis_matrix = squeeze(self.c_.flatten('C')).tolist()
            dynamics_msg.c_v = squeeze(self.c_v_.flatten('C')).tolist()
            dynamics_msg.gravity = squeeze(self.g_.flatten('C')).tolist()
            dynamics_msg.thrusters_dynamics = squeeze(self.du_.flatten('C')).tolist()
            dynamics_msg.tau = squeeze(self.tau_.flatten('C')).tolist()
            dynamics_msg.v_dot = squeeze(self.v_dot.flatten('C')).tolist()
            dynamics_msg.v = squeeze(self.v.flatten('C')).tolist()
            dynamics_msg.p_dot = squeeze(self.p_dot.flatten('C')).tolist()
            dynamics_msg.p = squeeze(self.p.flatten('C')).tolist()
            dynamics_msg.secs = self.t
            self.dyn_bag.write('dynamics param', dynamics_msg)

    def pubIMU(self, event):
        """ Generate Virtual IMU data, with noise and biases """
        if self.invers_dyanmics_intialized is True:
            imu_msg = Imu()
            # Orientation w.r.t to world frame - p TODO: VERIFY
            euler_noise = random.normal(0, self.imu_q_std, 3)
            p_w_noise = self.p[3:6] + euler_noise

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

            # Linear Acceleration w.r.t world frame  - pdot
            a_noise = random.normal(0, self.imu_a_std, 3)
            imu_msg.linear_acceleration.x = self.p_dot[0] + a_noise[0]
            imu_msg.linear_acceleration.y = self.p_dot[1] + a_noise[1]
            imu_msg.linear_acceleration.z = self.p_dot[2] + a_noise[2]
            imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] =\
                imu_msg.linear_acceleration_covariance[8] = pow(self.imu_a_std, 2)

            self.pub_imu.publish(imu_msg)

    def compute_tf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
        frame = PyKDL.Frame(r, v)
        return frame

    def reset(self, req):
        self.v = self.v_0
        self.p = self.p_0
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
        self.period = rospy.get_param(self.vehicle_name + "/dynamics" + "/period")
        self.uwsim_period = rospy.get_param(self.vehicle_name + "/dynamics/uwsim_period")

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
        self.tensor = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/tensor"))

        # Damping Properties
        self.damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/damping"))
        self.quadratic_damping = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/quadratic_damping"))

        self.dzv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dzv")
        self.dv = rospy.get_param(self.vehicle_name + "/dynamics" + "/dv")
        self.dh = rospy.get_param(self.vehicle_name + "/dynamics" + "/dh")
        self.density = rospy.get_param(self.vehicle_name + "/dynamics" + "/density")

        # Actuator Properties
        self.num_actuators = rospy.get_param(self.vehicle_name + "/num_actuators")
        self.kF = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_kf")
        self.kM = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_km")
        self.kV = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_kV")
        self.actuators_refV = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_reference_voltage")
        self.actuators_loadf = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_load_factor")
        self.actuators_r = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_radius")
        self.actuators_tconst = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_tconst")
        self.actuators_maxsat = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_maxsat")
        self.actuators_minsat = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_minsat")
        self.actuators_inv = rospy.get_param(self.vehicle_name + "/dynamics" + "/actuators_inversion")

        # Initialization
        self.p_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_pose"))
        self.v_0 = array(rospy.get_param(self.vehicle_name + "/dynamics" + "/initial_velocity"))
        self.frame_id = rospy.get_param(self.vehicle_name + "/dynamics" + "/frame_id")
        #   Init pose and velocity and period
        self.p = self.p_0
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
        self.du_ = []
        self.tau_ = []
        self.M = []

        #   Create publisher
        self.pub_pose = rospy.Publisher(self.output_topic, Pose, queue_size = 10)
        self.pub_imu = rospy.Publisher(self.output_imu_topic, Imu, queue_size = 10)
        self.pub_dyn_param = rospy.Publisher("/dolphin/dynamics/dynamics_param", dynamics_param, queue_size = 10)
        rospy.init_node("dynamics_" + self.vehicle_name, log_level = rospy.DEBUG)

        # Compute the Mass Matrix

        self.M = self.mass_matrix()
        self.invM = matrix(self.M).I
        rospy.logdebug("Mass Matrix M: \n %s", self.M)
        rospy.logdebug("Initial Position p: \n %s", self.p)
        rospy.logdebug("Initial Velocity v: \n %s", self.v)

        # Initial thrusters setpoint
        self.u = array(zeros(self.num_actuators))
        # Compute the thruster B Matrix
        kf = self.kF
        km = self.kM
        L = self.actuators_r

        # x forward z downward convention (y right)
        # Thrust on x axis, roll around x axis, pitch around y axis and yaw around z axis
        # -- 3  CW -- 1 CCW --
        # ---------\/---------
        # ---------/\---------
        # -- 2 CCW -- 4  CW --
        # The direction should be set by the mixer so this matrix should just scale the speed to force. And thrust will
        # Negative sign to account for reversed motors 3 & 4 in kf, motor speed sign should determine net roll with km
        a_inv = self.actuators_inv
        self.B = array([multiply(a_inv, [kf, kf, kf, kf]),
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [km, km, km, km],
                        multiply(a_inv, [-kf*L, kf*L, -kf*L, kf*L]),
                        multiply(a_inv, [-kf*L, kf*L, kf*L, -kf*L])])

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
        self.v_dot = self.inverse_dynamics()
        self.v = self.integral(self.v_dot, self.v, self.period)
        self.p_dot = self.kinematics()
        self.p = self.integral(self.p_dot, self.p, self.period)

        t2 = rospy.Time.now()
        self.t = (t2 - self.t0).to_sec()
        p = self.period - (t2 - t1).to_sec()
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
