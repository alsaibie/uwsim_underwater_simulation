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
cases = ["Sinusoid", "Square", "Ramp", 'Flat']
roll_enable = 0.0
pitch_enable = 0
yaw_enable = 0.5
thrust_enable = 0
case = "Flat"


def gen_ramp(sample, samples, frequency_per_samples, enable):
    result = enable * (sample % (samples / frequency_per_samples)) / (samples / (frequency_per_samples * 4)) - 1
    if result > 1.0000:
        result = (-result + 2)
    return result

writer.writerow(["ch1,ch2,ch3,ch4"])

# # This assumes properly mixed thruster_inputs
for sample in range(samples):
    if case == cases[0]:
         roll = roll_enable * math.sin((sample) * (4 * math.pi / samples))
         pitch = pitch_enable * math.sin((sample) * (2 * math.pi / samples))
         yaw = yaw_enable * math.sin((sample) * (6 * math.pi / samples))
         thrust = thrust_enable * math.sin((sample) * (8 * math.pi / samples))
         writer.writerow((roll, pitch, yaw, thrust))
    if case == cases[1]:  # Square
         roll = roll_enable * np.sign(math.sin((sample + 30) * (4 * math.pi / samples)))
         pitch = pitch_enable * np.sign(math.sin((sample + 60) * (2 * math.pi / samples)))
         yaw = yaw_enable * np.sign(math.sin((sample + 90) * (6 * math.pi / samples)))
         thrust = thrust_enable * np.sign(math.sin((sample + 120) * (8 * math.pi / samples)))
         writer.writerow((roll, pitch, yaw, thrust))
    if case == cases[2]:  # Ramp
         roll = gen_ramp(sample, samples, 12, roll_enable)
         pitch = gen_ramp(sample, samples, 6, pitch_enable)
         yaw = gen_ramp(sample, samples, 4, yaw_enable)
         thrust = gen_ramp(sample, samples, 8, thrust_enable)
         writer.writerow((roll, pitch, yaw, thrust))
    if case == cases[3]:  # Flat
         if sample <= 30:
             roll = roll_enable 
         if sample > 30:
            roll = roll_enable
         pitch = pitch_enable
         yaw = yaw_enable
         thrust = thrust_enable
         writer.writerow((roll, pitch, yaw, thrust))