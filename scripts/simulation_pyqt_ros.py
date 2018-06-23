import os
import sys
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QTabWidget
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
import pandas as pd
import time
import rospy
from uwsim_msgs.msg import full_state

color_styles = ['#A5BE00', '#679436', '#EBF2FA', '#427AA1', '#0051BA', [240, 150, 100]]

data_groups_dictionary = [

    {'title': "Dynamics Output",
     'tabh': [],
     'topic': '/dolphin/dynamics/full_state',
     'plots': [{'name': 'Linear Position',
                'loc': [0, 0],
                'ylim': 'auto',
                'curves': [{'name': 'p.x', 'handle': ''},
                           {'name': 'p.y', 'curvedata': []},
                           {'name': 'p.z', 'curvedata': []}]},
               {'name': 'Linear Velocity',
                'loc': [1, 0],
                'ylim': 'auto',
                'curves': [{'name': 'v.x', 'curvedata': []},
                           {'name': 'v.y', 'curvedata': []},
                           {'name': 'v.z', 'curvedata': []}]},
               {'name': 'Linear Acceleration - Inertial',
                'loc': [2, 0],
                'ylim': 'auto',
                'curves': [{'name': 'pdot.x', 'curvedata': []},
                           {'name': 'pdot.y', 'curvedata': []},
                           {'name': 'pdot.z', 'curvedata': []}]},
               {'name': 'Linear Acceleration - Body',
                'loc': [3, 0],
                'ylim': 'auto',
                'curves': [{'name': 'vdot.x', 'curvedata': []},
                           {'name': 'vdot.y', 'curvedata': []},
                           {'name': 'vdot.z', 'curvedata': []}]},
               {'name': 'Attitude',
                'loc': [0, 1],
                'ylim': 'auto',
                'curves': [{'name': 'att.x', 'curvedata': []},
                           {'name': 'att.y', 'curvedata': []},
                           {'name': 'att.z', 'curvedata': []}]},
               {'name': 'Attitude Quaternion',
                'loc': [1, 1],
                'ylim': [-1.2, 1.2],
                'curves': [{'name': 'attq.w', 'curvedata': []},
                           {'name': 'attq.x', 'curvedata': []},
                           {'name': 'attq.y', 'curvedata': []},
                           {'name': 'attq.z', 'curvedata': []}]},
               {'name': 'Angular Velocity',
                'loc': [2, 1],
                'ylim': 'auto',
                'curves': [{'name': 'v.r', 'curvedata': []},
                           {'name': 'v.p', 'curvedata': []},
                           {'name': 'v.yaw', 'curvedata': []}]},
               {'name': 'Angular Acceleration',
                'loc': [3, 1],
                'ylim': 'auto',
                'curves': [{'name': 'vdot.r', 'curvedata': []},
                           {'name': 'vdot.p', 'curvedata': []},
                           {'name': 'vdot.yaw', 'curvedata': []}]}]
     },

    # {'title': "Dynamics Parameters",
    #  'tabh': [],
    #  'subgroups': [
    #      {'data': [],
    #       'plots': [{'name': 'Coriolis Parameters',
    #                  'loc': [0, 0],
    #                  'ylim': 'auto',
    #                  'curves': [{'name': 'c1', 'curvedata': []},
    #                             {'name': 'c2', 'curvedata': []},
    #                             {'name': 'c3', 'curvedata': []},
    #                             {'name': 'c4', 'curvedata': []},
    #                             {'name': 'c5', 'curvedata': []},
    #                             {'name': 'c6', 'curvedata': []}]}],
    #       'filename': 'dynamics_param_output_Cv.txt'},
    #
    #      {'data': [],
    #       'plots': [{'name': 'Damping Parameters',
    #                  'loc': [1, 0],
    #                  'ylim': 'auto',
    #                  'curves': [{'name': 'd1', 'curvedata': []},
    #                             {'name': 'd2', 'curvedata': []},
    #                             {'name': 'd3', 'curvedata': []},
    #                             {'name': 'd4', 'curvedata': []},
    #                             {'name': 'd5', 'curvedata': []},
    #                             {'name': 'd6', 'curvedata': []}]}],
    #       'filename': 'dynamics_param_output_Dv.txt'},
    #
    #      {'data': [],
    #       'plots': [{'name': 'Gravity Parameters',
    #                  'loc': [2, 0],
    #                  'ylim': 'auto',
    #                  'curves': [{'name': 'g1', 'curvedata': []},
    #                             {'name': 'g2', 'curvedata': []},
    #                             {'name': 'g3', 'curvedata': []},
    #                             {'name': 'g4', 'curvedata': []},
    #                             {'name': 'g5', 'curvedata': []},
    #                             {'name': 'g6', 'curvedata': []}]}],
    #       'filename': 'dynamics_param_output_g.txt'},
    #
    #      # {'data': [], 'plots': [{'name': 'Mass Parameters', 'curves': [], 'loc': [1,1],
    #      #                         'clist':['M1', 'M2', 'M3', 'M4', 'M5', 'M6']}],
    #      #  'filename': 'dynamics_param_output_Mv.txt'},
    #
    #      {'data': [],
    #       'plots': [{'name': 'Tau',
    #                  'loc': [3, 0],
    #                  'ylim': 'auto',
    #                  'curves': [{'name': 'tau1', 'curvedata': []},
    #                             {'name': 'tau2', 'curvedata': []},
    #                             {'name': 'tau3', 'curvedata': []},
    #                             {'name': 'tau4', 'curvedata': []},
    #                             {'name': 'tau5', 'curvedata': []},
    #                             {'name': 'tau6', 'curvedata': []}]}],
    #       'filename': 'dynamics_param_output_tau.txt'}]},

    # {'title': "Sensor Output",
    #  'tabh': [],
    #  'data': [],
    #  'plots': [{'name': 'Accelerometer',
    #             'loc': [0, 0],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'acc.x', 'curvedata': []},
    #                        {'name': 'acc.y', 'curvedata': []},
    #                        {'name': 'acc.y', 'curvedata': []}]},
    #            {'name': 'Gyro',
    #             'loc': [1, 0],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'gyro.x', 'curvedata': []},
    #                        {'name': 'gyro.y', 'curvedata': []},
    #                        {'name': 'gyro.z', 'curvedata': []}]},
    #            {'name': 'Magnetometer',
    #             'loc': [2, 0],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'mag.x', 'curvedata': []},
    #                        {'name': 'mag.y', 'curvedata': []},
    #                        {'name': 'mag.z', 'curvedata': []}]}],
    #  'filename': 'sensor_output.txt'},
    #
    # {'title': "Position Controller",
    #  'tabh': [],
    #  'data': [],
    #  'plots': [{'name': 'Thrust',
    #             'loc': [0, 0],
    #             'ylim': [-1.2, 1.2],
    #             'curves': [{'name': 'thrust', 'curvedata': []}]},
    #            {'name': 'Attitude',
    #             'loc': [1, 0],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'att.x', 'curvedata': []},
    #                        {'name': 'att.y', 'curvedata': []},
    #                        {'name': 'att.z', 'curvedata': []}]},
    #            {'name': 'Attitude Setpoint',
    #             'loc': [2, 0],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'attq.w', 'curvedata': []},
    #                        {'name': 'attq.x', 'curvedata': []},
    #                        {'name': 'attq.y', 'curvedata': []},
    #                        {'name': 'attq.z', 'curvedata': []}]}],
    #  'filename': 'pcontroller_output.txt'},
    #
    # {'title': "Attitude Controller",
    #  'tabh': [],
    #  'data': [],
    #  'plots': [{'name': 'Thrust',
    #             'loc': [0, 0],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'thrust', 'curvedata': []}]},
    #            {'name': 'Rates Sp',
    #             'loc': [0, 1],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'rate_sp.x', 'curvedata': []},
    #                        {'name': 'rate_sp.y', 'curvedata': []},
    #                        {'name': 'rate_sp.z', 'curvedata': []}]},
    #            {'name': 'Tau Sp',
    #             'loc': [1, 0],
    #             'ylim': [-1.2, 1.2],
    #             'curves': [{'name': 'tau_sp.x', 'curvedata': []},
    #                        {'name': 'tau_sp.y', 'curvedata': []},
    #                        {'name': 'tau_sp.z', 'curvedata': []}]},
    #            {'name': 'Motor 1 SP',
    #             'loc': [3, 1],
    #             'ylim': [-1.2, 1.2],
    #             'curves': [{'name': 'm.1', 'curvedata': []}]},
    #            {'name': 'Motor 2 SP',
    #             'loc': [4, 0],
    #             'ylim': [-1.2, 1.2],
    #             'curves': [{'name': 'm.2', 'curvedata': []}]},
    #            {'name': 'Motor 3 SP',
    #             'loc': [3, 0],
    #             'ylim': [-1.2, 1.2],
    #             'curves': [{'name': 'm.3', 'curvedata': []}]},
    #            {'name': 'Motor 4 SP',
    #             'loc': [4, 1],
    #             'ylim': [-1.2, 1.2],
    #             'curves': [{'name': 'm.4', 'curvedata': []}]}
    #            ],
    #  'filename': 'acontroller_output.txt'},
    #
    # {'title': "Performance Analysis",
    #  'tabh': [],
    #  'data': [],
    #  'plots': [{'name': 'Placeholder',
    #             'loc': [0, 0],
    #             'ylim': 'auto',
    #             'curves': [{'name': 'curve1', 'curvedata': []},
    #                        {'name': 'curve2', 'curvedata': []}]}],
    #  'filename': 'none'},

]


class App(QMainWindow):

    def __init__(self):
        super(App, self).__init__()
        self.title = 'Simulation'
        self.left = 0
        self.top = 0
        self.width = 1920
        self.height = 1080
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.table_widget = MyTableWidget(self)
        self.setCentralWidget(self.table_widget)
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', 'w')
        self.show()
        self.timer = pg.QtCore.QTimer()
        self.timer.start(500)


class MyTableWidget(QWidget):

    def __init__(self, parent):
        super(MyTableWidget, self).__init__(parent)

        self.setStyleSheet("QLabel {font: 30pt Comic Sans MS}")

        self.layout = QtGui.QGridLayout()
        # Initialize tab screen
        self.tabs = QTabWidget()

        self.binary_cmd = '../dynamics_stack/unit_sim_nonreal/Install/bin/unit_sim_nonreal'
        self.data_path = '../dynamics_stack/unit_sim_nonreal/Install/data/'

        self.data_groups = data_groups_dictionary

        self.add_tabs()

        self.pushButton_gen_command = QPushButton("Generate Commands")
        self.pushButton_run_sim = QPushButton("Run Simulation and Visualize")
        self.pushButton_update_vis = QPushButton("Update Visualizations")
        self.pushButton_run_all = QPushButton("Run All")

        self.layout.addWidget(self.pushButton_gen_command, 0, 0)
        self.layout.addWidget(self.pushButton_run_sim, 0, 1)
        self.layout.addWidget(self.pushButton_update_vis, 0, 2)
        self.layout.addWidget(self.pushButton_run_all, 0, 3)
        self.layout.addWidget(self.tabs, 1, 0, 1, 0)

        self.setLayout(self.layout)

        # add connections
        self.pushButton_gen_command.clicked.connect(self.on_command_gen_cmd)
        self.pushButton_run_sim.clicked.connect(self.on_run_sim_cmd)
        self.pushButton_update_vis.clicked.connect(self.on_update_vis_cmd)
        self.pushButton_run_all.clicked.connect(self.on_run_all_cmd)

        self.get_data()
        self.add_plots()
        self.update_plots()

    def add_tabs(self):
        for group in self.data_groups:
            group['tabh'] = QWidget()
            self.tabs.addTab(group['tabh'], group['title'])

    def get_data(self):
        for group in self.data_groups:
            if group['title'] == 'Dynamics Parameters':
                """ Multiple files """
                for subgroup in group['subgroups']:
                    subgroup['data'] = pd.read_csv(self.data_path + subgroup['filename'],
                                                   skipinitialspace=True).rename(
                        columns={'time': 'Time (ms)'}).set_index('Time (ms)')
            else:
                """ Single File """
                if group['filename'] != 'none':
                    group['data'] = pd.read_csv(self.data_path + group['filename'],
                                                skipinitialspace=True).rename(columns={'time': 'Time (ms)'}).set_index(
                        'Time (ms)')

    def add_plots(self):
        for group in self.data_groups:

            gridbox = QtGui.QGridLayout()
            group['tabh'].setLayout(gridbox)

            if group['title'] == 'Dynamics Parameters':
                for subgroup in group['subgroups']:
                    # Add plots
                    for plot in subgroup['plots']:
                        plotwidget = pg.PlotWidget()
                        plotwidget.setAntialiasing(True)
                        plotwidget.setBackground([50, 50, 50])
                        plotwidget.setTitle(plot['name'])
                        plotwidget.setLabel(axis='bottom', text='Time(ms)')

                        if plot['ylim'] != 'auto':
                            plotwidget.setYRange(plot['ylim'][0], plot['ylim'][1])

                        plotwidget.showGrid(x=True, y=True, alpha=0.35)
                        plotwidget.addLegend(offset=(-10, 1))
                        gridbox.addWidget(plotwidget, plot['loc'][0], plot['loc'][1])

                        # Add Curves
                        for k, curve in enumerate(plot['curves']):
                            curve['curvedata'] = pg.PlotCurveItem(name=curve['name'],
                                                                  pen=pg.mkPen(color_styles[k], width=3),
                                                                  antialias=True)
                            plotwidget.addItem(curve['curvedata'])

            else:
                # Add plots
                for plot in group['plots']:

                    plotwidget = pg.PlotWidget()
                    plotwidget.setAntialiasing(True)
                    plotwidget.setBackground([50, 50, 50])
                    plotwidget.setTitle(plot['name'])
                    plotwidget.setLabel(axis='bottom', text='Time(ms)')

                    if plot['ylim'] != 'auto':
                        plotwidget.setYRange(plot['ylim'][0], plot['ylim'][1])

                    plotwidget.showGrid(x=True, y=True, alpha=0.35)
                    plotwidget.addLegend(offset=(-10, 1))
                    gridbox.addWidget(plotwidget, plot['loc'][0], plot['loc'][1])

                    # Add Curves
                    for k, curve in enumerate(plot['curves']):
                        curve['curvedata'] = pg.PlotCurveItem(name=curve['name'],
                                                              pen=pg.mkPen(color_styles[k], width=3), antialias=True)
                        plotwidget.addItem(curve['curvedata'])

    def update_plots(self):
        for group in self.data_groups:
            if group['title'] == 'Dynamics Parameters':
                for subgroup in group['subgroups']:
                    for plot in subgroup['plots']:
                        for curve in plot['curves']:
                            curve['curvedata'].setData(x=subgroup['data'].index.values,
                                                       y=subgroup['data'][curve['name']].values)
                            # print(subgroup['data'].index.values)

            else:
                if group['filename'] != 'none':
                    for plot in group['plots']:
                        for curve in plot['curves']:
                            curve['curvedata'].setData(x=group['data'].index.values,
                                                       y=group['data'][curve['name']].values)

    def full_state_msg_callback(self):
        rospy.loginfo("Hello man!")

    def on_command_gen_cmd(self):
        print("generate Commands")
        os.system('./gen_commands_trpy.py')

    def on_run_sim_cmd(self):
        print("Run Simulation")
        ret_status = os.system(self.binary_cmd)
        if (ret_status > 0):
            self.get_data()
            self.update_plots()

    def on_update_vis_cmd(self):
        print("Update Plots")
        self.get_data()
        self.update_plots()

    def on_run_all_cmd(self):
        start_time = time.time()
        print("Run All")
        os.system('./gen_commands_trpy.py')
        ret_status = os.system(self.binary_cmd)
        if (ret_status > 0):
            self.get_data()
            self.update_plots()
        print("--- %s seconds ---" % (time.time() - start_time))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
