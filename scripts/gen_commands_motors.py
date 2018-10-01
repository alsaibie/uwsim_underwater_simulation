#!/usr/bin/env python
from __future__ import division
import numpy as np
import csv
import math
data_path = '/home/alsaibie/UWSim/src/underwater_simulation/dynamics_stack/unit_sim_nonreal/Install/data/'
samples = 400
ctrl_input = []
f = open(data_path + 'generic_input' +
         '.txt', 'wt')
# TODO: make separate files for each case
writer = csv.writer(f, escapechar=' ', quoting=csv.QUOTE_NONE)
cases = ["Sinusoid", "Square", "Ramp", 'Flat', "SingleSquare", "SquareMixed"]
roll_enable = 0
pitch_enable = 0
yaw_enable = 1
surge_enable = 0
case = "SingleSquare"


def gen_ramp(sample, samples, frequency_per_samples, enable):
    result = enable * (sample % (samples / frequency_per_samples)) / (samples / (frequency_per_samples * 4)) - 1
    if result > 1.0000:
        result = (-result + 2)
    return result

writer.writerow(["ch1,ch2,ch3,ch4"])

# # This assumes properly mixed thruster_inputs
# for sample in range(samples):
#     if case == cases[0]:
#         roll = roll_enable * math.sin((sample + 30) * (4 * math.pi / samples))
#         pitch = pitch_enable * math.sin((sample + 60) * (2 * math.pi / samples))
#         yaw = yaw_enable * math.sin((sample + 90) * (6 * math.pi / samples))
#         thrust = thrust_enable * math.sin((sample + 120) * (8 * math.pi / samples))
#         writer.writerow((roll, pitch, yaw, thrust))
#     if case == cases[1]:  # Square
#         roll = roll_enable * np.sign(math.sin((sample + 30) * (4 * math.pi / samples)))
#         pitch = pitch_enable * np.sign(math.sin((sample + 60) * (2 * math.pi / samples)))
#         yaw = yaw_enable * np.sign(math.sin((sample + 90) * (6 * math.pi / samples)))
#         thrust = thrust_enable * np.sign(math.sin((sample + 120) * (8 * math.pi / samples)))
#         writer.writerow((roll, pitch, yaw, thrust))
#     if case == cases[2]:  # Ramp
#         roll = gen_ramp(sample, samples, 12, roll_enable)
#         pitch = gen_ramp(sample, samples, 6, roll_enable)
#         yaw = gen_ramp(sample, samples, 4, roll_enable)
#         thrust = gen_ramp(sample, samples, 8, roll_enable)
#         writer.writerow((roll, pitch, yaw, thrust))

# This only works for single Tau for now - make sure only one uniform input is selected
m1 = 0
m2 = 0
m3 = 0
m4 = 0
def motor_cmds(motiontype, dir_):
    tseries_ = dir_
    m1_ = 0
    m2_ = 0
    m3_ = 0
    m4_ = 0
    if motiontype == 'Surge':
        m1_ = tseries_
        m2_ = tseries_
        m3_ = tseries_
        m4_ = tseries_
    elif motiontype == 'Roll':
        m1_ = tseries_
        m2_ = tseries_
        m3_ = -tseries_
        m4_ = -tseries_
    elif motiontype == 'Pitch':
        m1_ = -tseries_
        m2_ = tseries_
        m3_ = -tseries_
        m4_ = tseries_
    elif motiontype == 'Yaw':
        m1_ = -tseries_
        m2_ = tseries_
        m3_ = tseries_
        m4_ = -tseries_
    
    return m1_, m2_, m3_, m4_

for sample in range(samples):
    if case == cases[5]:
        
        if sample > 0 and sample <= 20:
            m1, m2, m3, m4 = motor_cmds('Surge', 1)
        elif sample > 20 and sample <= 40:
            m1, m2, m3, m4 = motor_cmds('Surge', -1)
        elif sample > 40 and sample <= 100:
            m1, m2, m3, m4 = motor_cmds('Surge', 0)
            
        elif sample > 100 and sample <= 120:
            m1, m2, m3, m4 = motor_cmds('Roll', 1)
        elif sample > 120 and sample <= 140:
            m1, m2, m3, m4 = motor_cmds('Roll', -1)
        elif sample > 140 and sample <= 200:
            m1, m2, m3, m4 = motor_cmds('Roll', 0)
            
        elif sample > 200 and sample <= 220:
            m1, m2, m3, m4 = motor_cmds('Pitch', 1)
        elif sample > 220 and sample <= 240:
            m1, m2, m3, m4 = motor_cmds('Pitch', -1)
        elif sample > 240 and sample <= 300:
            m1, m2, m3, m4 = motor_cmds('Pitch', 0)
            
        elif sample > 300 and sample <= 320:
            m1, m2, m3, m4 = motor_cmds('Yaw', 1)
        elif sample > 320 and sample <= 340:
            m1, m2, m3, m4 = motor_cmds('Yaw', -1)
        elif sample > 340 and sample <= 400:
            m1, m2, m3, m4 = motor_cmds('Yaw', 0)
        writer.writerow((m1, m2, m3, m4))
    
    else:
        if case == cases[0]: # Sinusoid
            tseries = math.sin(sample * (8 * math.pi / samples))
        if case == cases[1]: # Square
            tseries = np.sign(math.sin((sample + 30) * (4 * math.pi / samples)))

        if case == cases[2]: # Ramp
            tseries = gen_ramp(sample, samples, 12, roll_enable)
        if case == cases[3]: # Flat
            tseries = 1
        if case == cases[4]: # BangBang
            if sample > 0 and sample <=100:
                tseries = 1
            elif sample > 100 and sample <=300 :
                tseries = -1
            else:
                tseries = 0

        if surge_enable == 1:
            m1 = tseries
            m2 = tseries
            m3 = tseries
            m4 = tseries
        elif roll_enable == 1:
            m1 = tseries
            m2 = tseries
            m3 = -tseries
            m4 = -tseries
        elif pitch_enable == 1:
            m1 = -tseries
            m2 = tseries
            m3 = -tseries
            m4 = tseries
        elif yaw_enable == 1:
            m1 = -tseries
            m2 = tseries
            m3 = tseries
            m4 = -tseries

    
        writer.writerow((m1, m2, m3, m4))