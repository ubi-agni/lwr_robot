#!/usr/bin/env python
#
# author Guillaume Walck (2015)
#
# derived from pr2_breaker.py
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import rospy
import functools
from python_qt_binding.QtCore import QSize, pyqtSignal

from rqt_robot_dashboard.widgets import MenuDashWidget

from python_qt_binding.QtGui import QMessageBox


class ControlStateButton(MenuDashWidget):
    """
    Dashboard widget to display and interact with the lwr state.
    """
    # create a signal for external triggering
    _set_enabled_signal = pyqtSignal(int)

    def __init__(self, group_name, parent):
        """
        :param group_name: Name of the group
        :type group_name: str
        :param group_index: Index of the group
        :type group_index: int
        """
        
        self._serial = 0
        self._parent = parent
        self._name = group_name

        if group_name == 'left_arm':
            state_icon = 'ic-larm.svg'
        elif group_name == 'right_arm':
            state_icon = 'ic-rarm.svg'
        else:
            state_icon = 'ic-breaker.svg'

        command_icon = ['bg-green.svg', state_icon]
        monitor_on_icon = ['bg-yellow.svg', state_icon]
        monitor_off_icon = ['bg-grey.svg', state_icon]
        error_icon = ['bg-red.svg', state_icon, 'ol-err-badge.svg']
        disabled_icon = ['bg-light_grey.svg', state_icon]

        icons = [disabled_icon, error_icon, monitor_off_icon, monitor_on_icon, command_icon]
        self._state_dict= {"disabled": 0, "error": 1, "monitor_off": 2, "monitor_on": 3, "command": 4}

        super(ControlStateButton, self).__init__('State:' + group_name, icons=icons, icon_paths=[['rqt_lwr_dashboard', 'images']])

        # init the button in disabled
        self.update_state(0)

        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

        self.add_action('Command', functools.partial(self._parent.on_btn_command_mode_clicked, group_name=self._name))
        self.add_action('Monitor', functools.partial(self._parent.on_btn_monitor_mode_clicked, group_name=self._name))
        self.add_action('Home', functools.partial(self._parent.on_btn_home_clicked, group_name=self._name))
        self.add_action('Park', functools.partial(self._parent.on_btn_park_clicked, group_name=self._name))
        self.enable_menu = self.add_action('Enable/Disable', self.on_enable_disable)
        self.enable_menu.setCheckable(True)
        self.enable_menu.setChecked(False)
        self.set_group_enabled(False)

        self._set_enabled_signal.connect(self.on_enable_disable)

        
        self._pending_msg =  None
        self._state = None
        self._last_status_msg = None
        self.setToolTip(group_name)

    def on_enable_disable(self):
        if self.enable_menu.isChecked():
            if self._state is not None:
                self._state = None
                self.set_group_enabled(True)
                self.set_state(self._pending_msg)
            else:
                self.enable_menu.setChecked(False)
        else:
            self.set_group_enabled(False)

    def set_run(self):
        if (not self.control(self._name, 2)):
            return

        self.control(self._name, 3)

    def set_freeze(self):
        self.control(self._name, 2)

    def set_instandby(self):
        self.control(self._name, 1)

    def set_run_all(self):
        if (not self.control3(2)):
            return
        self.control3(3)

    def set_freeze_all(self):
        self.control3(2)

    def set_standby_all(self):
        self.control3(1)

    def set_state(self, state):
        """
        Sets state of button based on msg

        :param msg: message containing the M3 control state
        :type msg: m3meka_msgs.msg.M3ControlStates
        """

        if (self.enable_menu.isChecked() or self._state is None):
            status_msg = "Running"
            # if first message received, enable the group
            if self._state is None:
                self.set_group_enabled(True)
                self.enable_menu.setChecked(True)
            if self._state != state:
                self._state = state
                if state in self._state_dict:
                    self.update_state(self._state_dict[state])

                if (status_msg != self._last_status_msg):
                    self.setToolTip("Group: %s \nState: %s" % (self._name, status_msg))
                    self._last_status_msg = status_msg
        else:
            self._pending_msg = state

    def set_group_enabled(self, val):
        if not val:
            self.update_state(0)
        for action in self._menu.actions():
            if not action.isCheckable():
                action.setEnabled(val)

