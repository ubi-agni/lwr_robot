#!/usr/bin/env python
# author Guillaume WALCK (2016)
"""
Dashboard interface for LWR with friorocosokc.krl and for lwr_panel_driver
"""

import rospy
from lwr_fri.msg import FriKrlData
from std_srvs.srv import Empty

LWR_DOF = 7

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
OROCOS_OKC_RESET_FRI = 98
OROCOS_OKC_END_KRL = 99


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
        self._lwr_drive_on = None
        self._lwr_drive_off = None
        self._last_krl_cmd = FriKrlData()
        self._last_krl_ret = None
        self._seq_cnt = None
        self._namespace = namespace
        self._seq_mismatch_count = 0

        self.init_publisher()
        self.init_subscriber()
        self.init_service()

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
            self._krl_sub = rospy.Subscriber(self._namespace + "/fromKRL", FriKrlData, self.callback, queue_size=10)
        else:
            self._krl_sub = rospy.Subscriber("fromKRL", FriKrlData, self.callback, queue_size=10)
        return True

    def init_service(self):
        """
        Initialize service to communicate with the lwr panel driver
        """
        if self._namespace is not None:
            try:
                self._lwr_drive_on = rospy.ServiceProxy(self._namespace + "/drive_on", Empty)
                self._lwr_drive_on.wait_for_service(timeout=1.0)
            except rospy.ROSException, e:
                # print "Service  failed: %s"%e
                self._lwr_drive_on = None
            try:
                self._lwr_drive_off = rospy.ServiceProxy(self._namespace + "/drive_off", Empty)
                self._lwr_drive_off.wait_for_service(timeout=1.0)
            except rospy.ROSException, e:
                # print "Service  failed: %s"%e
                self._lwr_drive_off = None

        else:
            rospy.logerr("A namespace must be specified for panel operations")
        return True

    def reset(self):
        """ reset internal command status
        used if missed ack and commands are blocked
        """
        self._last_krl_cmd = FriKrlData()
        self._seq_cnt = None
        rospy.logwarn("resetted the dashboard " + self._namespace)

    def callback(self, data):
        """
        incoming krl message processing (acknowledge)
        """

        # initialize the counter on first message
        if self._seq_cnt is None:
            self._seq_cnt = data.intData[OKC_SEQ_IDX]
            rospy.loginfo("resetted sequence number to %d " % self._seq_cnt)

        # check if incoming data is an acknowledge
        if data.boolData & (1 << 0):

            # compare the seq numbers (should be incremented by one)
            if self._seq_cnt == data.intData[OKC_SEQ_IDX]:
                self._seq_mismatch_count = 0

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
                self._seq_mismatch_count += 1
                if self._seq_mismatch_count % 5000 == 0:
                    rospy.logwarn("invalid seq number %d should be %d"
                                  % (data.intData[OKC_SEQ_IDX], self._seq_cnt))
                # currently force the counter
                # self._seq_cnt = data.intData[OKC_SEQ_IDX]

    def krl_request(self, cmd, intdata=None, realdata=None):
        """
        generic krl command request
        """
        if self._last_krl_cmd.boolData & (1 << 0):
            rospy.logwarn("cannot request while there is a pending command with code %d"
                          % self._last_krl_cmd.intData[OKC_CMD_IDX])
            return False
        else:
            if self._seq_cnt is not None:
                msg = FriKrlData()
                msg.intData[OKC_CMD_IDX] = cmd
                if intdata is not None:
                    for key in intdata:
                        msg.intData[key] = intdata[key]
                if realdata is not None:
                    for key in realdata:
                        msg.realData[key] = realdata[key]
                self._seq_cnt += 1
                self._krl_pub.publish(msg)
                self._last_krl_cmd = msg
                return True
            else:
                rospy.logwarn("Did not receive any data fromKRL yet, cannot send a command")
                return False

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
            self._seq_cnt = 1
            return True
        return False

    def reset_fri(self):
        """
        Request reset_fri
        """
        return self.krl_request(OROCOS_OKC_RESET_FRI)

    def end_krl(self):
        """
        Request end_krl
        """
        return self.krl_request(OROCOS_OKC_END_KRL)

    def switch_cartesian_impedance_control(self):
        """
        Request cartesian control mode (strategy 20)
        """
        return self.krl_request(OKC_SWITCH_CP_CONTROL)

    def switch_joint_impedance_control(self):
        """
        Request axis control (joint impedance) mode (strategy 30)
        """
        return self.krl_request(OKC_SWITCH_AXIS_CONTROL)

    def switch_joint_control(self):
        """
        Request axis control (joint) mode (strategy 10)
        """
        return self.krl_request(OKC_SWITCH_POSITION)

    def switch_control_mode(self, mode=None):
        """
        Request control mode change
        """
        return self.krl_request(OROCOS_OKC_SWITCH_CONTROL_MODE)

    def set_axis_stiffness_damping(self, stiffness=None, damping=None):
        """
        Set stiffness and damping in axis mode
        """
        mystiffness = stiffness
        if stiffness is not None:
            if len(stiffness) == LWR_DOF:
                mystiffness = dict([(8, stiffness[0]),
                                    (9, stiffness[1]),
                                    (10, stiffness[2]),
                                    (11, stiffness[3]),
                                    (12, stiffness[4]),
                                    (13, stiffness[5]),
                                    (14, stiffness[6])])
            else:
                rospy.logwarn("wrong stiffness size %d, should be 7" 
                              % len(stiffness))

        mydamping = damping
        if damping is not None:
            if len(damping) == LWR_DOF:
                mydamping = dict([(9, damping[0]),
                                  (10, damping[1]),
                                  (11, damping[2]),
                                  (12, damping[3]),
                                  (13, damping[4]),
                                  (14, damping[5]),
                                  (15, damping[6])])
            else:
                rospy.logwarn("wrong damping size %d, should be 7"
                              % len(damping))
        if mystiffness is not None and mydamping is not None:
            return self.krl_request(OKC_SET_AXIS_STIFFNESS_DAMPING, mystiffness, mydamping)

    def set_cp_stiffness_damping(self, stiffness=None, damping=None):
        """
        Set stiffness and damping in axis mode
        """
        mystiffness = stiffness
        if stiffness is not None:
            if len(stiffness) == 6:
                mystiffness = dict([(9, stiffness[0]),
                                    (10, stiffness[1]),
                                    (11, stiffness[2]),
                                    (12, stiffness[3]),
                                    (13, stiffness[4]),
                                    (14, stiffness[5])])
            else:
                rospy.logwarn("wrong stiffness size %d, should be 6"
                              % len(stiffness))

        mydamping = damping
        if damping is not None:
            if len(damping) == 6:
                mydamping = dict([(10, damping[0]),
                                  (11, damping[1]),
                                  (12, damping[2]),
                                  (13, damping[3]),
                                  (14, damping[4]),
                                  (15, damping[5])])
            else:
                rospy.logwarn("wrong damping size %d, should be 6"
                              % len(damping))
        if mystiffness is not None and mydamping is not None:
            return self.krl_request(OKC_SET_CP_STIFFNESS_DAMPING, mystiffness, mydamping)

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

    def move_to_axis_pos(self, jnt_pos=None):
        """
        Request move to jnt_pos
        """
        myjnt_pos = jnt_pos
        if jnt_pos is not None:
            if len(jnt_pos) == LWR_DOF:
                myjnt_pos = dict([(9, jnt_pos[0]),
                                  (10, jnt_pos[1]),
                                  (11, jnt_pos[2]),
                                  (12, jnt_pos[3]),
                                  (13, jnt_pos[4]),
                                  (14, jnt_pos[5]),
                                  (15, jnt_pos[6])])

        return self.krl_request(OROCOS_OKC_MOVE_AXIS_POS, None, myjnt_pos)

    def enable_motors(self):
        """
        Enable lwr motor drives (DRIVE ON)
        Should permit releasing brakes
        """
        ret = False
        if self._lwr_drive_on is not None:
            try:
                ret = self._lwr_drive_on()
            except rospy.ServiceException, e:
                rospy.logwarn("Failed to execute the Drive ON/OFF service %s"%e)
            except rospy.ROSException, e:
                rospy.logwarn("Other Drive ON/OFF service exception: %s"%e)
        else:
            rospy.logwarn("Drive ON/OFF is not initialized properly")
        return ret

    def disable_motors(self):
        """
        Disable lwr motor drives (DRIVE OFF)
        Should brake
        """
        ret = False
        if self._lwr_drive_off is not None:
            try:
                ret = self._lwr_drive_off()
            except rospy.ServiceException, e:
                rospy.logwarn("Failed to execute the Drive ON/OFF service %s"%e)
            except rospy.ROSException, e:
                rospy.logwarn("Other Drive ON/OFF service exception: %s"%e)
        else:
            rospy.logwarn("Drive ON/OFF is not initialized properly")
        return ret
