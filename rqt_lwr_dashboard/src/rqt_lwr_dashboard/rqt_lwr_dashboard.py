#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# derived from emergency_buttons_dashboard.py  rqt_lwr_dashboard
#
# derived from rqt_emergency_buttons: emergency_buttons_dashboard.py
#   original Authors Sammy Pfeiffer
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#

# author Guillaume WALCK (2016)

import rospy
import functools
from rqt_robot_dashboard.dashboard import Dashboard

from python_qt_binding.QtCore import QSize, QTimer, Qt
from QtGui import QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QLabel, \
    QCheckBox, QMessageBox

from lwr_dashboard.lwr_dashboard import LwrDashboard

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from .state_button import ControlStateButton


class RqtLwrDashboard(Dashboard):
    """
    Dashboard for LWR via fri and krl

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'LWR Buttons Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._last_dashboard_message_time = rospy.Time.now()
        self._state_buttons = {}
        self._widget_initialized = False
        self._last_error = {}
        self._last_power_state = {}
        self._last_control_state = {}
        self._last_control_strategy = {}

        # TODO read this list on the parameters
        group_names = ["left_arm", "right_arm"]

        self._last_error["left_arm"] = None
        self._last_power_state["left_arm"] = None
        self._last_control_state["left_arm"] = None
        self._last_control_strategy["left_arm"] = None
        self._last_error["right_arm"] = None
        self._last_power_state["right_arm"] = None
        self._last_control_state["right_arm"] = None
        self._last_control_strategy["right_arm"] = None

        self._lwrdb = {}
        self._lwrdb["right_arm"] = LwrDashboard(namespace="ra")
        self._lwrdb["left_arm"] = LwrDashboard(namespace="la")

        # create as many buttons as groups received
        for i, group_name in enumerate(group_names):
            self._state_buttons[group_name] = ControlStateButton(group_name, self)

        self._main_widget = QWidget()
        vlayout_main = QVBoxLayout()
        # enable
        hlayout_enable = QHBoxLayout()
        # power+fri
        hlayout_power_fri = QHBoxLayout()
        # power
        vlayout_power = QVBoxLayout()
        # fri
        hlayout_fricom_header = QHBoxLayout()
        hlayout_fricom_quality = QHBoxLayout()
        vlayout_fri = QVBoxLayout()
        # move
        vlayout_move = QVBoxLayout()

        # control
        vlayout_control_mode = QVBoxLayout()
        hlayout_control_mode = {}
        hlayout_control_mode["left_arm"] = QHBoxLayout()
        hlayout_control_mode["right_arm"] = QHBoxLayout()

        self.chk_all = QCheckBox("enable_all")
        self.chk_all.setChecked(True)

        # create buttons
        self.btn_command_mode = QPushButton("command mode")
        self.btn_monitor_mode = QPushButton("monitor mode")
        self.btn_command_mode.setStyleSheet("background-color: rgb(197, 197, 197)")
        self.btn_monitor_mode.setStyleSheet("background-color: rgb(197, 197, 197)")
        self.btn_drive_on = QPushButton("drive on")
        self.btn_drive_on.setStyleSheet("background-color: rgb(209, 149, 37)")
        self.btn_drive_off = QPushButton("drive on")
        self.btn_drive_off.setStyleSheet("background-color: rgb(90, 148, 95)")
        self.btn_reset_fri = QPushButton("reset fri")
        self.btn_end_krl = QPushButton("end krl")
        self.btn_home = QPushButton("home")
        self.btn_park = QPushButton("park")
        self.btn_ctrl = {}
        self.btn_ctrl["left_arm"] = {}
        self.btn_ctrl["left_arm"]["Position"] = QPushButton("Jnt")
        self.btn_ctrl["left_arm"]["Cartesian impedance"] = QPushButton("CartImp")
        self.btn_ctrl["left_arm"]["Joint impedance"] = QPushButton("JntImp")
        self.btn_ctrl["right_arm"] = {}
        self.btn_ctrl["right_arm"]["Position"] = QPushButton("Jnt")
        self.btn_ctrl["right_arm"]["Cartesian impedance"] = QPushButton("CartImp")
        self.btn_ctrl["right_arm"]["Joint impedance"] = QPushButton("JntImp")

        self.lbl_fricom = {}
        self.lbl_fricom["left_arm"] = QLabel("BAD")
        self.lbl_fricom["right_arm"] = QLabel("BAD")

        # disable some buttons by default
        self.btn_command_mode.setEnabled(False)
        self.btn_monitor_mode.setEnabled(False)
        self.btn_home.setEnabled(False)
        self.btn_park.setEnabled(False)

        # enable some other buttons
        self.btn_drive_on.setEnabled(True)
        self.btn_drive_off.setEnabled(True)

        self.btn_end_krl.setEnabled(True)
        self.btn_reset_fri.setEnabled(True)

        # place buttons
        hlayout_enable.addWidget(self.chk_all)

        vlayout_power.setAlignment(Qt.AlignCenter)
        vlayout_power.addWidget(QLabel("Power state"))
        vlayout_power.addWidget(self.btn_command_mode)
        vlayout_power.addWidget(self.btn_monitor_mode)
        vlayout_power.addWidget(self.btn_drive_on)
        vlayout_power.addWidget(self.btn_drive_off)

        hlayout_fricom_header.setAlignment(Qt.AlignCenter)
        hlayout_fricom_header.addWidget(QLabel("left"))
        hlayout_fricom_header.addWidget(QLabel("right"))
        hlayout_fricom_quality.setAlignment(Qt.AlignCenter)
        hlayout_fricom_quality.addWidget(self.lbl_fricom["left_arm"])
        hlayout_fricom_quality.addWidget(self.lbl_fricom["right_arm"])

        vlayout_fri.addWidget(QLabel("FRI"))
        vlayout_fri.addLayout(hlayout_fricom_header)
        vlayout_fri.addLayout(hlayout_fricom_quality)
        vlayout_fri.addWidget(self.btn_reset_fri)
        vlayout_fri.addWidget(self.btn_end_krl)

        vlayout_move.addWidget(QLabel("Move"))
        vlayout_move.addWidget(self.btn_home)
        vlayout_move.addWidget(self.btn_park)

        for group in self.btn_ctrl:
            hlayout_control_mode[group].addWidget(QLabel(group+":"))
            for key in self.btn_ctrl[group]:
                hlayout_control_mode[group].addWidget(self.btn_ctrl[group][key])

        vlayout_control_mode.addWidget(QLabel("Control mode"))
        vlayout_control_mode.addLayout(hlayout_control_mode["left_arm"])
        vlayout_control_mode.addLayout(hlayout_control_mode["right_arm"])

        hlayout_power_fri.addLayout(vlayout_power)
        hlayout_power_fri.addLayout(vlayout_fri)

        vlayout_main.addLayout(hlayout_enable)
        vlayout_main.addLayout(hlayout_power_fri)
        vlayout_main.addLayout(vlayout_control_mode)
        vlayout_main.addLayout(vlayout_move)

        # signals for buttons
        self.btn_command_mode.clicked.connect(functools.partial(self.on_btn_command_mode_clicked, group_name=None))
        self.btn_monitor_mode.clicked.connect(functools.partial(self.on_btn_monitor_mode_clicked, group_name=None))
        self.btn_drive_on.clicked.connect(functools.partial(self.on_btn_drive_on_clicked, group_name=None))
        self.btn_drive_off.clicked.connect(functools.partial(self.on_btn_drive_off_clicked, group_name=None))
        self.btn_reset_fri.clicked.connect(functools.partial(self.on_btn_reset_fri_clicked, group_name=None))
        self.btn_end_krl.clicked.connect(functools.partial(self.on_btn_end_krl_clicked, group_name=None))
        self.btn_home.clicked.connect(functools.partial(self.on_btn_home_clicked, group_name=None))
        self.btn_park.clicked.connect(functools.partial(self.on_btn_park_clicked, group_name=None))

        for group in self.btn_ctrl:
            for key in self.btn_ctrl[group]:
                self.btn_ctrl[group][key].clicked.connect(functools.partial(self.on_btn_ctrl_change_clicked, group_name=group, mode=key))

        self.chk_all.stateChanged.connect(self.on_enable_all_clicked)

        self._main_widget.setLayout(vlayout_main)
        self.context.add_widget(self._main_widget)
        # self._main_widget.addLayout(hlayout)
        self._widget_initialized = True

        self._diag_subs = {}
        self._diag_ns = {}
        self.init_subscribers("left_arm", "la")
        self.init_subscribers("right_arm", "ra")

        self.watchdog_timeout = {}
        self.watchdog_timeout["left_arm"] = True
        self.watchdog_timeout["right_arm"] = True

        self.timer = {}
        self.timer["left_arm"] = QTimer()
        self.timer["left_arm"].timeout.connect(functools.partial(self.on_timeout, group_name="left_arm"))
        self.timer["right_arm"] = QTimer()
        self.timer["right_arm"].timeout.connect(functools.partial(self.on_timeout, group_name="right_arm"))

        self.timer["left_arm"].start(2000)  # 5 sec timeout
        self.timer["right_arm"].start(2000)  # 5 sec timeout

    def init_subscribers(self, group_name=None, namespace=None):
        if group_name is not None:
            ns = namespace
            if namespace is None:
                if group_name in self._diag_ns:
                    ns = self._diag_ns[group_name]

            if ns is not None:
                if group_name not in self._diag_subs:
                    self._diag_subs[group_name] = rospy.Subscriber("/" + ns + "/diagnostics/", DiagnosticArray,
                                                                   functools.partial(self.diag_callback, group_name=group_name))
                self._diag_ns[group_name] = ns

    def on_timeout(self, group_name=None):
        if group_name is not None:
            if self.watchdog_timeout[group_name]:
                self.change_button_state(group_name)
                # cleanup connection that have a publisher and not data coming in
                if self._diag_subs[group_name].get_num_connections() > 0:
                    self._diag_subs[group_name].unregister()
                    del self._diag_subs[group_name]
                    self.init_subscribers(group_name)
            else:
                self.watchdog_timeout[group_name] = True

    def on_enable_all_clicked(self):
        """
        start
        """
        for group_name in self._state_buttons:
            self._state_buttons[group_name].enable_menu.setChecked(self.chk_all.isChecked())
            self._state_buttons[group_name].enable_menu.triggered.emit(self.chk_all.isChecked())

    def on_btn_command_mode_clicked(self, group_name=None):
        """
        switch to command mode
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                self._lwrdb[group_name].command_mode_request()
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].command_mode_request()

    def on_btn_drive_on_clicked(self, group_name=None):
        """
        switch drive on
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                self._lwrdb[group_name].enable_motors()
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].enable_motors()

    def on_btn_drive_off_clicked(self, group_name=None):
        """
        switch drive off
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                self._lwrdb[group_name].disable_motors()
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].disable_motors()

    def on_btn_ctrl_change_clicked(self, group_name=None, mode=None):
        """
        request a control mode change
        :param group_name: group concerned, default is None meaning act on all enabled groups
        :param mode: mode to change the control to, default is None meaning no change

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                if mode is not None:
                    if mode == "Position":
                        self._lwrdb[group_name].switch_joint_control()
                    if mode == "Cartesian impedance":
                        self._lwrdb[group_name].switch_cartesian_impedance_control()
                    if mode == "Joint impedance":
                        self._lwrdb[group_name].switch_joint_impedance_control()
                else:
                    print "btn control change requires a valid mode. (", mode, ") given"
        else:
            print "btn control change requires a valid group name. (", group_name, ") given"

    def on_btn_reset_fri_clicked(self, group_name=None):
        """
        reset the fri connection at the krl side
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                self._lwrdb[group_name].reset_fri()
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].reset_fri()

    def on_btn_end_krl_clicked(self, group_name=None):
        """
        terminate the krl prog
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                self._lwrdb[group_name].end_krl()
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].end_krl()

    def on_btn_monitor_mode_clicked(self, group_name=None):
        """
        switch to monitor mode
        :param group_name: group concerned, default is None meaning act on all enabled groups
        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                self._lwrdb[group_name].monitor_mode_request()
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].monitor_mode_request()

    def on_btn_home_clicked(self, group_name=None):
        """
        move to home
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                # request a confirmation
                """
                Show a warning message
                """
                flags = QMessageBox.Ok
                flags |= QMessageBox.Abort
                msg = "CHECK the " + group_name + " is SAFE to go HOME pose before proceeding !\n\
                       RESET your applications to restart from Home pose"

                response = QMessageBox.warning(self._main_widget, "Warning!", msg, flags, QMessageBox.Abort)
                if response == QMessageBox.Ok:
                    print "Proceeding to go home"
                    self._lwrdb[group_name].move_start_pos()

                else:
                    print "Canceled go home request"
        else:
            '''for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].move_start_pos()'''
            print "Going home not supported for dual arm yet"

    def on_btn_park_clicked(self, group_name=None):
        """
        move to park
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                # request a confirmation
                """
                Show a warning message
                """
                flags = QMessageBox.Ok
                flags |= QMessageBox.Abort
                msg = "CHECK the " + group_name + " is SAFE to go PARK pose before proceeding !\n\
                       RESET your applications to restart from PARK pose"

                response = QMessageBox.warning(self._main_widget, "Warning!", msg, flags, QMessageBox.Abort)
                if response == QMessageBox.Ok:
                    print "Proceeding to park pose"
                    self._lwrdb[group_name].move_park_pos()
                else:
                    print "Canceled park pose request"
        else:
            '''for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].move_park_pos()'''
            print "Going park pose not supported for dual arm yet"

    def get_widgets(self):
        widgets_list = []
        for group_name in self._state_buttons:
            widgets_list.append([self._state_buttons[group_name]])

        return widgets_list

    def diag_callback(self, msg, group_name):
        """
        callback to process lwr diagnostic messages
        :param msg:
        :type msg: Diagnostic
        """

        power_state = None
        control_strategy = None
        control_state = None
        error = False
        quality = None
        if group_name in self._state_buttons:
            # reset timer
            self.watchdog_timeout[group_name] = False
            if len(msg.status) > 0:
                if msg.status[0].name == "lwr FRI state":
                    for prop in msg.status[0].values:
                        if "State" in prop.key:
                            control_state = prop.value
                        if "Quality" in prop.key:
                            quality = prop.value
            if len(msg.status) > 1:
                if msg.status[1].name == "lwr robot state":
                    for prop in msg.status[1].values:
                        if "Power" in prop.key:
                            power_state = prop.value
                        if "Control Strategy" in prop.key:
                            control_strategy = prop.value
                    if msg.status[1].level == DiagnosticStatus.ERROR:
                        error = True

            self.change_button_state(group_name, power_state, control_state, control_strategy, quality, error)

    def change_button_state(self, group_name, power_state=None, control_state=None, control_strategy=None, quality=None, error=False):
        # update buttons on state change

        if error:
                self._state_buttons[group_name].set_state("error")
        else:
            if power_state is not None and control_state is not None:
                if power_state == "0000000":
                    self._state_buttons[group_name].set_state("monitor_off")
                elif power_state == "1111111" and control_state == "monitor":
                    self._state_buttons[group_name].set_state("monitor_on")
                elif power_state == "1111111" and control_state == "command":
                    self._state_buttons[group_name].set_state("command")
                else:
                    print "Invalid states power:", power_state, ", control_state:", control_state
            else:
                self._state_buttons[group_name].set_state("disabled")
                self._last_power_state[group_name] = None
                self._last_control_state[group_name] = None

            if self._last_control_strategy[group_name] != control_strategy and control_strategy is not None:
                for key in self.btn_ctrl[group_name]:
                    if key == control_strategy:
                        self.btn_ctrl[group_name][key].setStyleSheet("background-color: rgb(90, 148, 95)")  # green
                    else:
                        self.btn_ctrl[group_name][key].setStyleSheet("background-color: rgb(197, 197, 197)")  # grey
                self._last_control_strategy[group_name] = control_strategy

            if self.lbl_fricom[group_name].text() != quality and quality is not None:
                self.lbl_fricom[group_name].setText(quality)

        if self._last_power_state[group_name] != power_state and power_state is not None and error is False:
            if power_state == "1111111":
                self._lwrdb[group_name].reset()
                self.btn_command_mode.setEnabled(True)
                self.btn_monitor_mode.setEnabled(True)
                self.btn_command_mode.setStyleSheet("background-color: rgb(153, 42, 43)")
                self.btn_monitor_mode.setStyleSheet("background-color: rgb(209, 149, 37)")
            else:
                self.btn_command_mode.setEnabled(False)
                self.btn_monitor_mode.setEnabled(False)
                self.btn_command_mode.setStyleSheet("background-color: rgb(197, 197, 197)")
                self.btn_monitor_mode.setStyleSheet("background-color: rgb(197, 197, 197)")

            self._last_power_state[group_name] = power_state
            self._last_control_state[group_name] = control_state

        self._last_error[group_name] = error

    # def shutdown_dashboard(self):
