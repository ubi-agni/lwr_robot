#!/usr/bin/env python
# author Guillaume WALCK (2016)
"""
Dashboard interface for LWR with friorocosokc.krl
"""

import rospy
from lwr_fri.msg import FriKrlData

OKC_ACK_IDX = 14
OKC_SEQ_IDX = 13
OKC_CMD_IDX = 15

OKC_FRI_START = 1
OKC_FRI_STOP = 2
OKC_RESET_STATUS = 3
OKC_SET_CP_STIFFNESS_DAMPING = 4
OKC_SET_AXIS_STIFFNESS_DAMPING = 5
OKC_SWITCH_CP_CONTROL = 6
OKC_SWITCH_AXIS_CONTROL = 7
OKC_SWITCH_GRAVCOMP = 8
OKC_SWITCH_POSITION = 9
OKC_MOVE_START_POSITION = 10
OKC_RESET_COUNTER = 11
OROCOS_OKC_DRIVE_OFF = 20
OROCOS_OKC_DRIVE_ON = 21
OROCOS_OKC_MOVE_AXIS_POS = 22
OROCOS_OKC_SWITCH_CONTROL_MODE = 30
OROCOS_OKC_MOVE_PARK_POS = 90


class LwrDashboard(object):
    """
        Dashboard to interact with the LWR via KRL
    """

    def __init__(self, namespace=None):
        """
        init
        """
        self._krl_pub = None
        self._krl_sub = None
        self._last_krl_cmd = FriKrlData()
        self._last_krl_ret = None
        self._seq_cnt = 0
        self._namespace = namespace


        self.init_publisher()
        self.init_subscriber()
            

    def init_publisher(self):
        """
        Initialize publisher to communicate with the kcp via FRI
        """
        if self._namespace is not None:
            self._krl_pub = rospy.Publisher(self._namespace + "/toKRL", FriKrlData, queue_size=1)
        else:
            self._krl_pub = rospy.Publisher("toKRL", FriKrlData, queue_size=1)
        return True

    def init_subscriber(self):
        """
        Initialize subscriber to communicate with the kcp via FRI
        """
        if self._namespace is not None:
            self._krl_sub = rospy.Subscriber(self._namespace + "/fromKRL", FriKrlData, self.callback)
        else:
            self._krl_sub = rospy.Subscriber("fromKRL", FriKrlData, self.callback)
        return True

    def reset(self):
        """ reset internal command status
        used if missed ack and commands are blocked
        """
        self._last_krl_cmd = FriKrlData()

    def callback(self, data):
        """
        incoming krl message processing (acknowledge)
        """
        # check if incoming data is an acknowledge
        if data.boolData & (1 << 0):

            # compare the seq numbers (should be incremented by one)
            if self._seq_cnt + 1 == data.intData[OKC_SEQ_IDX]:
                self._seq_cnt = data.intData[OKC_SEQ_IDX]

                # if we actually commanded something
                if self._last_krl_cmd.intData[OKC_CMD_IDX] != 0:

                    # compare the ack with the last command sent out
                    if self._last_krl_cmd.intData[OKC_CMD_IDX] == data.intData[OKC_ACK_IDX]:
                        # reset the command in the last message
                        self._last_krl_cmd.intData[OKC_CMD_IDX] = 0
                    else:
                        rospy.logwarn("invalid ack %d for command %d"
                                      % (data.intData[OKC_ACK_IDX], self._last_krl_cmd.intData[OKC_CMD_IDX]))
                # just store
                self._last_krl_ret = data
            else:
                rospy.logwarn("invalid seq number %d should be %d"
                              % (data.intData[OKC_SEQ_IDX], self._seq_cnt + 1))
                # currently force the counter
                self._seq_cnt = data.intData[OKC_SEQ_IDX]

    def krl_request(self, cmd):
        """
        generic krl command request
        """
        if self._last_krl_cmd.boolData & (1 << 0):
            rospy.logwarn("cannot request while there is a pending command with code %d"
                          % self._last_krl_cmd.intData[OKC_CMD_IDX])
            return False
        else:
            msg = FriKrlData()
            msg.intData[OKC_CMD_IDX] = cmd
            self._krl_pub.publish(msg)
            self._last_krl_cmd = msg
            return True

    def command_mode_request(self):
        """
        Request command mode to fri
        """
        return self.krl_request(OKC_FRI_START)

    def monitor_mode_request(self):
        """
        Request monitor mode to fri
        """
        return self.krl_request(OKC_FRI_STOP)

    def reset_counter(self):
        """
        reset KRL seq counter as well as internal counter
        """
        if self.krl_request(OKC_RESET_COUNTER):
            self._seq_cnt = 0
            return True
        return False

    def move_park_pos(self):
        """
        Request move_part_pos
        """
        return self.krl_request(OROCOS_OKC_MOVE_PARK_POS)

    def move_start_pos(self):
        """
        Request move_start_pos
        """
        return self.krl_request(OKC_MOVE_START_POSITION)

    def enable_motors(self):
        """
        Enable lwr motor drives (DRIVE ON)
        Should permit releasing brakes
        """
        print "Enable motor is not yet implemented, as it requires o2i_drv.o on the KRC2"

    def disable_motors(self):
        """
        Disable lwr motor drives (DRIVE OFF)
        Should brake
        """
        print "Disable motor is not yet implemented, as it requires o2i_drv.o on the KRC2"
