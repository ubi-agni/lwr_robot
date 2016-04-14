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

from python_qt_binding.QtCore import QSize
from QtGui import QPushButton, QVBoxLayout, QHBoxLayout, QWidget, \
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

        # TODO read this list on the parameters
        group_names = ["left_arm", "right_arm"]
        
        self._last_error["left_arm"] = None
        self._last_power_state["left_arm"] = None
        self._last_control_state["left_arm"] = None
        self._last_error["right_arm"] = None
        self._last_power_state["right_arm"] = None
        self._last_control_state["right_arm"] = None
        
        self._lwrdb = {}
        self._lwrdb["left_arm"] = LwrDashboard(namespace="la")
        self._lwrdb["right_arm"] = LwrDashboard(namespace="ra")
        

        # create as many buttons as groups received
        for group_name in group_names:
            self._state_buttons[group_name] = ControlStateButton(group_name, self)

        self._main_widget = QWidget()
        vlayout = QVBoxLayout()
        hlayout = QHBoxLayout()

        self.chk_all = QCheckBox("enable_all")
        self.chk_all.setChecked(True)

        # create buttons
        self.btn_command_mode = QPushButton("command_mode")
        self.btn_monitor_mode = QPushButton("monitor_mode")
        self.btn_home = QPushButton("home")
        self.btn_park = QPushButton("park")
        
        # disable buttons by default
        self.btn_command_mode.setEnabled(False)
        self.btn_monitor_mode.setEnabled(False)
        self.btn_home.setEnabled(False)
        self.btn_park.setEnabled(False)

        # place buttons
        hlayout.addWidget(self.chk_all)
        vlayout.addLayout(hlayout)
        vlayout.addWidget(self.btn_command_mode)
        vlayout.addWidget(self.btn_monitor_mode)
        vlayout.addWidget(self.btn_home)
        vlayout.addWidget(self.btn_park)

        # signals for buttons
        self.btn_command_mode.clicked.connect(functools.partial(self.on_btn_command_mode_clicked, group_name=None))
        self.btn_monitor_mode.clicked.connect(functools.partial(self.on_btn_monitor_mode_clicked, group_name=None))
        self.btn_home.clicked.connect(functools.partial(self.on_btn_home_clicked, group_name=None))
        self.btn_park.clicked.connect(functools.partial(self.on_btn_park_clicked, group_name=None))
        
        self.chk_all.stateChanged.connect(self.on_enable_all_clicked)

        self._main_widget.setLayout(vlayout)
        self.context.add_widget(self._main_widget)
        # self._main_widget.addLayout(hlayout)
        self._widget_initialized = True

        self._diag_subs = {}
        self._diag_subs["left_arm"] = rospy.Subscriber("/la/diagnostics/", DiagnosticArray,
                                                       functools.partial(self.diag_callback, group_name="left_arm"))
        self._diag_subs["right_arm"] = rospy.Subscriber("/ra/diagnostics/", DiagnosticArray,
                                                        functools.partial(self.diag_callback, group_name="right_arm"))

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
        if group_name in self._state_buttons:
            if len(msg.status) > 1:
                if msg.status[1].name == "lwr robot state":
                    for prop in msg.status[1].values:
                        if "Power" in prop.key:
                            power_state = prop.value
                        if "Control Strategy" in prop.key:
                            control_strategy = prop.value
                    if msg.status[1].level == DiagnosticStatus.ERROR:
                        error = True
                if msg.status[0].name == "lwr FRI state":
                    for prop in msg.status[0].values:
                        if "State" in prop.key:
                            control_state = prop.value

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

            # update buttons on power state change
            if self._last_power_state[group_name] != power_state and error is False:
                if power_state == "1111111":
                    self._lwrdb[group_name].reset()
                    self.btn_command_mode.setEnabled(True)
                    self.btn_monitor_mode.setEnabled(True)
                else:
                    
                    self.btn_command_mode.setEnabled(False)
                    self.btn_monitor_mode.setEnabled(False)

            self._last_error[group_name] = error
            self._last_power_state[group_name] = power_state
            self._last_control_state[group_name] = control_state

    #def shutdown_dashboard(self):
