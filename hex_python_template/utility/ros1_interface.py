#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import rospy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist

from .interface_base import InterfaceBase
from .hexcan import HEXCANMessage

class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown", rate: int = 10):
        super(DataInterface, self).__init__()

        ### ros node
        rospy.init_node(name, anonymous=True)
        self.__rate = rospy.Rate(rate)

        ### pamameter
        self._str_param = {
            "name": rospy.get_param('~str_name'),
        }
        self._int_param = {
            "range": rospy.get_param('~int_range'),
        }

        ### publisher
        self.__can_pub = rospy.Publisher('sent_messages', Frame, queue_size=10, tcp_nodelay=True)

        ### subscriber
        self.__can_sub = rospy.Subscriber('received_messages', Frame,
                                             self.__can_callback)
        self.__cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.__cmd_vel_callback)

        self.__can_sub
        self.__cmd_vel_sub

    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)
    
    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)
    
    def send_can_frame(self, can_frame: HEXCANMessage):
        # Transform HEXCANMessage to Frame
        f = Frame()
        f.id = can_frame.can_id
        f.data = can_frame.data
        f.is_extended = can_frame.extended
        f.dlc = len(can_frame.data)
        self.__can_pub.publish(f)

    def ok(self):
        return not rospy.is_shutdown()

    def sleep(self):
        self.__rate.sleep()

    def shutdown(self):
        pass

    def pub_out_str(self, out: str):
        pass

    def pub_out_int(self, out: int):
        pass

    def __frame_to_hex_can_msg(self, frame: Frame):
        return HEXCANMessage(frame.id, frame.data, frame.is_extended, frame.header.stamp.to_sec())

    def __can_callback(self, msg: Frame):
        self._hex_can_frame_queue.put(self.__frame_to_hex_can_msg(msg))

    def __cmd_vel_callback(self, msg: Twist):
        target_spd = [msg.linear.x, msg.linear.y, msg.angular.z]
        self._target_spd_queue.put(target_spd)

