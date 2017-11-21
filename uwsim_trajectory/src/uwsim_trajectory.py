#!/usr/bin/env python

import termios, fcntl, sys, os
import rospy, tf
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry


#Set input trajectory file and output odometry log files
trajectory_file = '/home/alsaibie/UWSim/trajectory_csv/test.csv'
odometry_log_file = '/home/alsaibie/UWSim/trajectory_csv/testlog.csv'
odometry_save = False

print("UWSim Trajectory Utility")
print("Usage: S: Start\Resume | P: Pause | R: Restart | X: Reset - Not Case Sensitive")

# ROS Topics
twist_topic = "/dolphin/velocityCommand"
pose_topic = "/dolphin/poseCommand"
odom_topic = "/uwsim/dolphin_odom"

# Setup input console
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

# Odomety Callback
odometry_data = Odometry()
def OdometryCB(data):
    global odometry_data
    odometry_data = data

# Pub abd Sub
pub_pose = rospy.Publisher(pose_topic, Pose, queue_size=1)
sub_odom = rospy.Subscriber(odom_topic, Odometry, OdometryCB)

# Start node
rospy.init_node('trajectoryUtility')

# Initialize trajectory flags
trajectory_flag = 'X'
trajectory_counter = 0

# Load trajectory file or generate trajectory
print("Load Trajectory")
trajectory_data = np.genfromtxt(trajectory_file, delimiter=',')
trajectory_index, trajectory_pose_pos, trajectory_pose_orient = np.hsplit(trajectory_data, [1,4])
pose_initial = deepcopy(odometry_data.pose.pose)
print("Initial X Y Z Position: ", pose_initial.position.x, pose_initial.position.y, pose_initial.position.z)
# Read current robot pose and offset trajectory


# This loop runs, terminal commands parsed determine what state it sits at.
def trajectory_loop():
    msg_pose = deepcopy(pose_initial)
    # print(odometry_data.pose.pose.position.x)

    global trajectory_flag, trajectory_counter, odometry_save

    if trajectory_flag == 'S' or trajectory_flag == 's':
        if trajectory_counter < len(trajectory_index):
            orientation = tf.transformations.quaternion_from_euler(trajectory_pose_orient[trajectory_counter,0],
                                                                   trajectory_pose_orient[trajectory_counter,1],
                                                                   trajectory_pose_orient[trajectory_counter,2],
                                                                   'sxyz')
            msg_pose.position.x = pose_initial.position.x + trajectory_pose_pos[trajectory_counter,0]
            msg_pose.position.y = pose_initial.position.y + trajectory_pose_pos[trajectory_counter,1]
            msg_pose.position.z = pose_initial.position.z + trajectory_pose_pos[trajectory_counter,2]
            msg_pose.orientation.x = orientation[0]
            msg_pose.orientation.y = orientation[1]
            msg_pose.orientation.z = orientation[2]
            msg_pose.orientation.w = orientation[3]
            trajectory_counter += 1
            pub_pose.publish(msg_pose)
        else:
            trajectory_flag = 'X'
            print("Trajectory Complete")
            odometry_save = True
    elif trajectory_flag == 'P' or trajectory_flag == 'p':
        a = 1 # Do nothing
    elif trajectory_flag == 'R' or trajectory_flag == 'r':
        trajectory_counter = 0 # Reset
        trajectory_flag = 'S' # Start
    elif trajectory_flag == 'X' or trajectory_flag == 'x':
        trajectory_counter = 0 # Reset Only


def trajectory_record():

    global trajectory_flag, odometry_data, odometry_save, odometry_log_data

    if odometry_save:
        np.savetxt(odometry_log_file, odometry_log_data, delimiter=',')
        odometry_save = False

    if trajectory_flag == 'S' or trajectory_flag == 's':
        pos = odometry_data.pose.pose.position
        orient = odometry_data.pose.pose.orientation
        odometry_log_data = np.append(odometry_log_data, [[pos.x, pos.y, pos.z]], axis = 0)
        print("Position X: {}mm, Y: {}mm, Z: {}mm ".format(pos.x*1000, pos.y*1000, pos.z*1000))

    elif trajectory_flag == 'P' or trajectory_flag == 'p':
        a = 1
    elif trajectory_flag == 'R' or trajectory_flag == 'r':
        odometry_log_data = np.empty([1,3])
    elif trajectory_flag == 'X' or trajectory_flag == 'x':
        odometry_log_data = np.empty([1,3])

try:

    while not rospy.is_shutdown():

        trajectory_loop()
        trajectory_record()
        rospy.sleep(0.1)

        try:
            c = sys.stdin.read(1)
            trajectory_flag = c

            if trajectory_flag == 'S' or trajectory_flag == 's':
                print("Start/Resume Trajectory")
            elif trajectory_flag == 'P' or trajectory_flag == 'p':
                print("Pause Trajectory")
            elif trajectory_flag == 'R' or trajectory_flag == 'r':
                print("Restart Trajectory")
            elif trajectory_flag == 'X' or trajectory_flag == 'x':
                print("Reset")
            else:
                print("Wrong key pressed")
                while c !='':
                    c = sys.stdin.read(1)
        except IOError: pass

finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)






