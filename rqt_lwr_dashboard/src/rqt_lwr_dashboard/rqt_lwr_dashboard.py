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

from python_qt_binding.QtGui import QFont
from python_qt_binding.QtCore import QSize, QRect, Qt, QTimer, \
    QObject, QMetaObject, pyqtSignal
from QtWidgets import QPushButton, QWidget, QLabel, QSlider, \
    QCheckBox, QMessageBox, QLayout, QVBoxLayout, QHBoxLayout, QDialog, QDialogButtonBox

from lwr_dashboard.lwr_dashboard import LwrDashboard

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String

from .state_button import ControlStateButton


class StiffnessDamping_Dialog(object):
    def setupUi(self, Dialog, mode="Joint impedance", stiffness_current=None, damping_current=None):
        Dialog.setObjectName("Dialog")

        if mode is not None:
            impendances = ["stiffness", "damping"]
            if mode == "Joint impedance":
                self.slider_names = ["A1", "A2", "E1", "A3", "A4", "A5", "A6"]
                impedance_max = {}
                impedance_max["stiffness"] = [2000]*7
                impedance_max["damping"] = [10.0]*7
                impedance_default = {}
                if stiffness_current is not None and len(stiffness_current) == 7:
                    impedance_default["stiffness"] = stiffness_current
                else:
                    impedance_default["stiffness"] = [500] * 7
                if damping_current is not None and len(damping_current) == 7:
                    impedance_default["damping"] = damping_current
                else:
                    impedance_default["damping"] = [7.0] * 7  # use max*10 and divide later to get float
            else:
                if mode == "Cartesian impedance":
                    self.slider_names = ["X", "Y", "Z", "A", "B", "C"]
                    impedance_max = {}
                    
                    impedance_max["stiffness"] = [500] * 6
                    impedance_max["damping"] = [10.0] * 6  # use max*10 and divide later to get float
                    impedance_default = {}
                    if stiffness_current is not None and len(stiffness_current) == 6:
                        impedance_default["stiffness"] = stiffness_current
                    else:
                        impedance_default["stiffness"] = [200] * 6
                    if damping_current is not None and len(damping_current) == 6:
                        impedance_default["damping"] = damping_current
                    else:
                        impedance_default["damping"] = [7.0] * 6  # use max*10 and divide later to get float
                else:
                    print (mode, " not supported")
                    self.slider_names = []

            self.val = {}
            self.sl = {}

            self.vlayout = QVBoxLayout(Dialog)
            self.hlayout = QHBoxLayout()
            self.vlayout_lbl = QVBoxLayout()
            self.vlayout_val = {}
            self.vlayout_sl = {}

            self.hlayout.addLayout(self.vlayout_lbl)

            self.vlayout_lbl.addWidget(QLabel("  "))

            for k, impedance in enumerate(impendances):
                self.val[impedance] = {}
                self.sl[impedance] = {}
                self.vlayout_sl[impedance] = QVBoxLayout()
                self.vlayout_val[impedance] = QVBoxLayout()

                self.vlayout_sl[impedance].addWidget(QLabel(impedance))
                self.vlayout_val[impedance].addWidget(QLabel("  "))

                for i, slider_name in enumerate(self.slider_names):
                    if k == 0:
                        self.vlayout_lbl.addWidget(QLabel(slider_name + ":"))

                    self.val[impedance][slider_name] = QLabel()
                    self.val[impedance][slider_name].setMinimumWidth(4*10)
                    if impedance == "damping":
                        self.val[impedance][slider_name].setText(str(impedance_default[impedance][i]/10.0))
                    else:
                        self.val[impedance][slider_name].setText(str(impedance_default[impedance][i]))
                    self.sl[impedance][slider_name] = QSlider()
                    self.sl[impedance][slider_name].setOrientation(Qt.Horizontal)
                    self.sl[impedance][slider_name].setObjectName(slider_name)
                    
                    if impedance == "damping":
                        self.sl[impedance][slider_name].setTickInterval(1)
                    else:
                        self.sl[impedance][slider_name].setTickInterval(50)
                    self.sl[impedance][slider_name].setMaximum(impedance_max[impedance][i])
                    self.sl[impedance][slider_name].setMinimum(0)
                    self.sl[impedance][slider_name].setValue(impedance_default[impedance][i])

                    self.sl[impedance][slider_name].valueChanged.connect(functools.partial(self.valueChange, slider_name=slider_name, impedance=impedance))

                    self.vlayout_sl[impedance].addWidget(self.sl[impedance][slider_name])
                    self.vlayout_val[impedance].addWidget(self.val[impedance][slider_name])

                self.hlayout.addStretch()
                self.hlayout.addLayout(self.vlayout_sl[impedance])
                self.hlayout.addLayout(self.vlayout_val[impedance])

            self.buttonBox = QDialogButtonBox()
            self.buttonBox.setOrientation(Qt.Horizontal)
            self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel | QDialogButtonBox.Ok)
            self.buttonBox.setObjectName("buttonBox")
            self.buttonBox.accepted.connect(Dialog.accept)
            self.buttonBox.rejected.connect(Dialog.reject)

            self.vlayout.addLayout(self.hlayout)
            self.vlayout.addWidget(self.buttonBox)

            QMetaObject.connectSlotsByName(Dialog)

    def valueChange(self, slider_name, impedance):
        if impedance in self.val:
            if slider_name in self.val[impedance]:
                if impedance == "damping":
                    self.val[impedance][slider_name].setText(str((self.sl[impedance][slider_name].value()/10.0)))
                else:
                    self.val[impedance][slider_name].setNum(self.sl[impedance][slider_name].value())

    def getValues(self):
        result = dict()
        for impedance in self.sl:
            result[impedance] = []
            for i, slider_name in enumerate(self.slider_names):
                if impedance == "damping":
                    result[impedance].append(self.sl[impedance][slider_name].value()/10.0)
                else:
                    result[impedance].append(self.sl[impedance][slider_name].value())
        return result

    # def retranslateUi(self, Dialog):
    #    Dialog.setWindowTitle(QtGui.QApplication.translate("Dialog", "Dialog", None, QtGui.QApplication.UnicodeUTF8))
    #    self.label.setText(QtGui.QApplication.translate("Dialog", "Set example value:", None, QtGui.QApplication.UnicodeUTF8))


class StartDlg(QDialog, StiffnessDamping_Dialog):
    def __init__(self, mode, stiffness, damping, parent=None):
        QDialog.__init__(self, parent)
        self.setupUi(self, mode=mode, stiffness_current=stiffness, damping_current=damping)


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

        # TODO read this list on the parameters
        group_names = ["left_arm", "right_arm"]

        self._lwrdb = {}
        self._lwrdb["right_arm"] = LwrDashboard(namespace="ra")
        self._lwrdb["left_arm"] = LwrDashboard(namespace="la")

        self._main_widget = QWidget()
        self._main_widget.setWindowTitle("LWR Dashboard")
        vlayout_main = QVBoxLayout()
        hlayout_main = QHBoxLayout()
        # enable
        hlayout_enable = QHBoxLayout()
        # labels
        vlayout_labels = QVBoxLayout()
        # both arms
        vlayout_both = QVBoxLayout()
        # power
        vlayout_power = QVBoxLayout()
        # fri
        vlayout_fri = QVBoxLayout()
        # move
        vlayout_move = QVBoxLayout()

        # single arms
        vlayout_arm = {}
        vlayout_arm["left_arm"] = QVBoxLayout()
        vlayout_arm["right_arm"] = QVBoxLayout()

        self.chk_all = QCheckBox("enable_all")
        self.chk_all.setChecked(True)

        # create buttons
        self.btn_command_mode = QPushButton("command mode")
        self.btn_monitor_mode = QPushButton("monitor mode")
        self.btn_command_mode.setStyleSheet("background-color: rgb(197, 197, 197)")
        self.btn_monitor_mode.setStyleSheet("background-color: rgb(197, 197, 197)")
        self.btn_drive_on = QPushButton("ready")
        self.btn_drive_on.setStyleSheet("background-color: rgb(209, 149, 37)")
        self.btn_drive_off = QPushButton("standby")
        self.btn_drive_off.setStyleSheet("background-color: rgb(90, 148, 95)")
        self.btn_reset_fri = QPushButton("reset fri")
        self.btn_end_krl = QPushButton("end krl")
        self.btn_home = QPushButton("home")
        self.btn_park = QPushButton("park")
        self.btn_ctrl = {}
        self.btn_stiffdamp = {}
        self.lbl_friquality = {}
        # create as many buttons as groups received
        for i, group_name in enumerate(group_names):
            self._state_buttons[group_name] = ControlStateButton(group_name, self)
            self.btn_ctrl[group_name] = {}
            self.btn_ctrl[group_name]["Position"] = QPushButton("Jnt")
            self.btn_ctrl[group_name]["Cartesian impedance"] = QPushButton("CartImp")
            self.btn_ctrl[group_name]["Joint impedance"] = QPushButton("JntImp")
            self.btn_stiffdamp[group_name] = {}
            self.btn_stiffdamp[group_name]["Joint impedance"] = QPushButton("JntImp")
            self.btn_stiffdamp[group_name]["Cartesian impedance"] = QPushButton("CartImp")
            self.lbl_friquality[group_name] = QLabel("BAD")

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
        hlayout_enable.setContentsMargins(1, 1, 1, 1)
        vlayout_labels.addWidget(QLabel("Arm:"))
        vlayout_labels.addStretch()
        vlayout_labels.addWidget(QLabel("Control"))
        vlayout_labels.addWidget(QLabel("mode"))
        vlayout_labels.addWidget(QLabel("settings"))
        vlayout_labels.addStretch()
        vlayout_labels.addWidget(QLabel("stiffness"))
        vlayout_labels.addWidget(QLabel("and damping"))
        vlayout_labels.addStretch()
        vlayout_labels.addWidget(QLabel("Fri comm"))
        vlayout_labels.addStretch()

        vlayout_power.setAlignment(Qt.AlignCenter)
        vlayout_power.addWidget(QLabel("Power state"))
        vlayout_power.addWidget(self.btn_command_mode)
        vlayout_power.addWidget(self.btn_monitor_mode)
        vlayout_power.addWidget(self.btn_drive_on)
        vlayout_power.addWidget(self.btn_drive_off)
        vlayout_fri.addWidget(self.btn_reset_fri)
        vlayout_fri.addWidget(self.btn_end_krl)
        vlayout_move.addWidget(QLabel("Move"))
        vlayout_move.addWidget(self.btn_home)
        vlayout_move.addWidget(self.btn_park)

        # hlayout_fricom_header.setAlignment(Qt.AlignCenter)
        vlayout_both.addLayout(vlayout_power)
        vlayout_both.addStretch()
        vlayout_both.addLayout(vlayout_fri)
        vlayout_both.addLayout(vlayout_move)

        for group in self.btn_ctrl:
            vlayout_arm[group].addWidget(QLabel(group))
            vlayout_arm[group].addStretch()
            for key in self.btn_ctrl[group]:
                vlayout_arm[group].addWidget(self.btn_ctrl[group][key])
            vlayout_arm[group].addStretch()
            for key in self.btn_stiffdamp[group]:
                vlayout_arm[group].addWidget(self.btn_stiffdamp[group][key])
            vlayout_arm[group].addStretch()
            vlayout_arm[group].addWidget(self.lbl_friquality[group])
            vlayout_arm[group].addStretch()

        hlayout_main.addLayout(vlayout_labels)
        hlayout_main.addLayout(vlayout_arm["left_arm"])
        hlayout_main.addLayout(vlayout_arm["right_arm"])
        hlayout_main.addLayout(vlayout_both)

        vlayout_main.addLayout(hlayout_enable)
        vlayout_main.addLayout(hlayout_main)

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

        for group in self.btn_stiffdamp:
            for key in self.btn_stiffdamp[group]:
                self.btn_stiffdamp[group][key].clicked.connect(functools.partial(self.on_btn_stiffdamp_change_clicked, group_name=group, mode=key))
        self.chk_all.stateChanged.connect(self.on_enable_all_clicked)

        self._main_widget.setLayout(vlayout_main)
        self.context.add_widget(self._main_widget)
        # self._main_widget.addLayout(hlayout)
        self._widget_initialized = True

        self._subs_ns = {}
        self._diag_subs = {}
        self._state_subs = {}
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
                if group_name in self._subs_ns:
                    ns = self._subs_ns[group_name]

            if ns is not None:
                self._subs_ns[group_name] = ns
                if group_name not in self._diag_subs:
                    self._diag_subs[group_name] = rospy.Subscriber("/" + ns + "/diagnostics/", DiagnosticArray,
                                                                   functools.partial(self.diag_callback, group_name=group_name))

                if group_name not in self._state_subs:
                    self._state_subs[group_name] = rospy.Subscriber("/" + ns + "/power_state/", String,
                                                                   functools.partial(self.state_callback, group_name=group_name))

    def on_timeout(self, group_name=None):
        if group_name is not None:
            if self.watchdog_timeout[group_name]:
                self.change_panel_state(group_name)
                # cleanup connection that have a publisher and not data coming in
                if self._diag_subs[group_name] is not None:
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
                
                if not self._lwrdb[group_name].enable_motors():
                    print ("failed to enable motors")
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    if not  self._lwrdb[group_name].enable_motors():
                        print ("failed to enable motors")

    def on_btn_drive_off_clicked(self, group_name=None):
        """
        switch drive off
        :param group_name: group concerned, default is None meaning act on all enabled groups

        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                if not self._lwrdb[group_name].disable_motors():
                    print ("failed to disable motors")
        else:
            for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    if not self._lwrdb[group_name].disable_motors():
                        print ("failed to disable motors")

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
                    print ("btn control change requires a valid mode. (", mode, ") given")
        else:
            print ("btn control change requires a valid group name. (", group_name, ") given")

    def on_btn_stiffdamp_change_clicked(self, group_name=None, mode=None):
        """
        request a user stiffness or damping change
        :param group_name: group concerned, default is None meaning act on all enabled groups
        :param mode: mode to change the control to, default is None meaning no change
        """
        if group_name is not None:
            if self._state_buttons[group_name].enable_menu.isChecked():
                if mode is not None:
                    # show a dialog with sliders
                    last_stiffness = None
                    last_damping = None
                    if mode == "Joint impedance":
                        [last_stiffness, last_damping] = self._lwrdb[group_name].get_last_axis_stiffness_damping()
                    if mode == "Cartesian impedance":
                        [last_stiffness, last_damping] = self._lwrdb[group_name].get_last_cp_stiffness_damping()
                    self.ui = StartDlg(mode, last_stiffness, last_damping)
                    if self.ui.exec_():
                        values = self.ui.getValues()
                        #print (values)
                        ret = False
                        if mode == "Joint impedance":
                            ret = self._lwrdb[group_name].set_axis_stiffness_damping(values["stiffness"], values["damping"])
                        if mode == "Cartesian impedance":
                            ret = self._lwrdb[group_name].set_cp_stiffness_damping(values["stiffness"], values["damping"])

                        if ret is False:
                            print ("btn stiffness change did not succeed in changing stiffness")
                else:
                    print ("btn stiffness change requires a valid mode. (", mode, ") given")
            else:
                print (group_name, " is disabled, cannot set stiffness/damping")
        else:
            print ("btn stiffness change requires a valid group name. (", group_name, ") given")

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
                    print ("Proceeding to go home")
                    self._lwrdb[group_name].move_start_pos()

                else:
                    print ("Canceled go home request")
        else:
            '''for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].move_start_pos()'''
            print ("Going home not supported for dual arm yet")

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
                    print ("Proceeding to park pose")
                    self._lwrdb[group_name].move_park_pos()
                else:
                    print ("Canceled park pose request")
        else:
            '''for group_name in self._state_buttons:
                if self._state_buttons[group_name].enable_menu.isChecked():
                    self._lwrdb[group_name].move_park_pos()'''
            print ("Going park pose not supported for dual arm yet")

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

            self.change_panel_state(group_name, control_strategy, quality)

    def state_callback(self, msg, group_name):
        """
        callback to process lwr power state messages
        :param msg:
        :type msg: String
        """

        if group_name in self._state_buttons:
            # reset timer
            self.watchdog_timeout[group_name] = False
            if len(msg.data) > 0:
                if msg.data in self._state_buttons[group_name]._state_dict:
                    self._state_buttons[group_name].set_state(msg.data)
                    if msg.data in ["READY", "MONITOR", "COMMAND"]:
                    #  self._state_buttons[group_name].enable_menu.setChecked(False)
                        self.btn_monitor_mode.setEnabled(True)
                        self.btn_monitor_mode.setStyleSheet("background-color: rgb(209, 149, 37)")
                    else:
                        self.btn_monitor_mode.setEnabled(False)
                        self.btn_monitor_mode.setStyleSheet("background-color: rgb(197, 197, 197)")

                    if msg.data in ["MONITOR", "COMMAND"]:
                        self.btn_command_mode.setEnabled(True)
                        self.btn_command_mode.setStyleSheet("background-color: rgb(153, 42, 43)")
                    else:
                        self.btn_command_mode.setEnabled(False)
                        self.btn_command_mode.setStyleSheet("background-color: rgb(197, 197, 197)")

    def change_panel_state(self, group_name, control_strategy=None, quality=None):
        # update buttons on state change
        if self.lbl_friquality[group_name].text() != quality and quality is not None:
            self.lbl_friquality[group_name].setText(quality)


    def unregister(self):
        it = QTreeWidgetItemIterator(self)
        while it.value():
            try:
                it.value().on_close()
            except:
                pass
            it += 1

    def shutdown_dashboard(self):
        self.timer["left_arm"].stop()
        self.timer["right_arm"].stop()
        for group in self._diag_subs:
            self._diag_subs[group].unregister()
            self._state_subs[group].unregister()
