#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import rclpy
import rclpy.node
import threading

from std_msgs.msg import String
from std_msgs.msg import Int32
from can_msgs.msg import Frame

from .interface_base import InterfaceBase
from .hexcan import HEXCANMessage
from geometry_msgs.msg import Twist

class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown", rate: int = 10):
        super(DataInterface, self).__init__()

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__rate = self.__node.create_rate(rate)

        ### pamameter
        # declare parameters
        self.__node.declare_parameter('str_name', "")
        self.__node.declare_parameter('int_range', [-1_000_000, 1_000_000])
        # str
        self._str_param = {
            "name":
            self.__node.get_parameter(
                'str_name').get_parameter_value().string_value,
        }
        # int
        self._int_param = {
            "range":
            self.__node.get_parameter(
                'int_range').get_parameter_value().integer_array_value,
        }

        ### publisher
        self.__can_pub = self.__node.create_publisher(Frame, 'sent_messages', 10)

        ### subscriber
        self.__can_sub = self.__node.create_subscription(Frame, 'received_messages',
                                             self.__can_callback, 10)
        self.__cmd_vel_sub = self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 10)
        self.__can_sub
        self.__cmd_vel_sub

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

    
    def logd(self, msg, *args, **kwargs):
        self. __logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)
    
    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    def send_can_frame(self, can_frame: HEXCANMessage):
        f = Frame()
        f.id = can_frame.can_id
        f.data = can_frame.data
        f.is_extended = can_frame.extended
        f.dlc = len(can_frame.data)
        self.__can_pub.publish(f)

    def ok(self):
        return rclpy.ok()

    def sleep(self):
        self.__rate.sleep()

    def shutdown(self):
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join()

    def __spin(self):
        rclpy.spin(self.__node)

    def pub_out_str(self, out: str):
        msg = String()
        msg.data = out
        self.__out_str_pub.publish(msg)

    def pub_out_int(self, out: int):
        msg = Int32()
        msg.data = out
        self.__out_int_pub.publish(msg)

    def __frame_to_hex_can_msg(self, frame: Frame) -> HEXCANMessage:
        return HEXCANMessage(frame.id, frame.data[:frame.dlc], frame.is_extended, frame.header.stamp.to_sec())

    def __can_callback(self, msg: Frame):
        self._hex_can_frame_queue.put(self.__frame_to_hex_can_msg(msg))

    def __cmd_vel_callback(self, msg: Twist):
        target_spd = [msg.linear.x, msg.linear.y, msg.angular.z]
        self._target_spd_queue.put(target_spd)
