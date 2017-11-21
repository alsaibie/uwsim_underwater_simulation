#!/usr/bin/env python

import termios, fcntl, sys, os
import rospy, tf
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float64MultiArray

# Set input trajectory file and output odometry log files
trajectory_file = '/home/alsaibie/UWSim/src/underwater_simulation/underwater_vehicle_dynamics/src/tests/data' \
                  '/thrusters_input.txt'
pose_log_file = '/home/alsaibie/UWSim/src/underwater_simulation/underwater_vehicle_dynamics/src/tests/data' \
                '/dynamics_output.csv'
pose_save = False
shutdown = False
# Load trajectory file or generate trajectory
print("Load Trajectory")
trajectory_data = np.genfromtxt(trajectory_file, delimiter=',')
trajectory_len = len(trajectory_data[:, 1])
trajectory_thrust = np.array(trajectory_data)
print(trajectory_len)
# Read current robot pose and offset trajectory

print("Dynamics Test")
print("Usage: S: Start\Resume | P: Pause | R: Restart | X: Reset - Not Case Sensitive")

# ROS Topics
thruster_topic = "/dolphin/thrusters_input"
pose_topic = "/dolphin/pose_sp"
pose_log_data = np.empty([1, 6])


def poseCallback(pose_data):
    # read and save pose somehow
    global trajectory_flag, pose_save, pose_log_data, shutdown
    (r_, p_, y_) = tf.transformations.euler_from_quaternion([pose_data.orientation.x, pose_data.orientation.y,
                                                             pose_data.orientation.z, pose_data.orientation.w])
    pdata = [[pose_data.position.x, pose_data.position.y, pose_data.position.z, r_, p_, y_]]
    if pose_save:
        np.savetxt(pose_log_file, pose_log_data, delimiter=',')
        pose_save = False
        # Quit Node
        shutdown = True

    if trajectory_flag == 'S' or trajectory_flag == 's':
        pose_log_data = np.append(pose_log_data, pdata, axis=0)

# Pub and Sub
thrusters_pub = rospy.Publisher(thruster_topic, Float64MultiArray, queue_size=10)
pose_sub = rospy.Subscriber(pose_topic, Pose, poseCallback)

# Start node
rospy.init_node('Dynamics Test')

# Initialize trajectory flags
trajectory_counter = 0
r = rospy.Rate(400)  # 400hz
msg_thrusters = Float64MultiArray()
trajectory_flag = 'S'

while trajectory_counter < trajectory_len:
    # And convert -1:1 to PWM 1100:1900
    msg_thrusters.data = trajectory_thrust[trajectory_counter, :] * 400 + 1500
    thrusters_pub.publish(msg_thrusters)
    trajectory_counter += 1
    r.sleep()

print("Trajectory Complete")
pose_save = True
trajectory_flag = 'X'
# rospy.sleep(1000)
rospy.signal_shutdown("done")
