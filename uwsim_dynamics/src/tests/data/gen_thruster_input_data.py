from __future__ import division
import numpy as np
import csv
import math

samples = 400
ctrl_input = []
f = open('/home/alsaibie/UWSim/src/underwater_simulation/underwater_vehicle_dynamics/src/tests/data/thrusters_input' +
         '.txt', 'wt')
# TODO: make separate files for each case
writer = csv.writer(f)
cases = ["Sinusoid", "Square", "Ramp", 'Flat']
roll_enable = 0
pitch_enable = 1
yaw_enable = 0
thrust_enable = 1
case = "Flat"


def gen_ramp(sample, samples, frequency_per_samples, enable):
    result = enable * (sample % (samples / frequency_per_samples)) / (samples / (frequency_per_samples * 4)) - 1
    if result > 1.0000:
        result = (-result + 2)
    return result

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

for sample in range(samples):
    if case == cases[0]:
        tseries = math.sin(sample * (8 * math.pi / samples))
    if case == cases[1]:  # Square
        tseries = np.sign(math.sin((sample + 30) * (4 * math.pi / samples)))

    if case == cases[2]:  # Ramp
        tseries = gen_ramp(sample, samples, 12, roll_enable)
    if case == cases[3]:  # Flat
        tseries = 1

    if thrust_enable == 1:
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