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

from .interface_base import InterfaceBase


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
        self.__out_str_pub = self.__node.create_publisher(
            String, 'out_str', 10)
        self.__out_int_pub = self.__node.create_publisher(Int32, 'out_int', 10)

        ### subscriber
        self.__in_str_sub = self.__node.create_subscription(
            String, 'in_str', self.__in_str_callback, 10)
        self.__in_int_sub = self.__node.create_subscription(
            Int32, 'in_int', self.__in_int_callback, 10)
        self.__in_str_sub
        self.__in_int_sub

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

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

    def __in_str_callback(self, msg: String):
        self._in_str_queue.put(msg.data)

    def __in_int_callback(self, msg: Int32):
        self._in_int_queue.put(msg.data)
