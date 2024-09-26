#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import queue
import typing
from abc import ABC, abstractmethod
from .hexcan import HEXCANMessage
class InterfaceBase(ABC):
    def __init__(self):
        ### ROS parameters
        self._str_param = {}
        self._int_param = {}

        ### ROS RX msg queues
        self._hex_can_frame_queue = queue.Queue(100)

    def __del__(self):
        self.shutdown()

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### parameters
    ####################
    def get_str_param(self) -> dict:
        return self._str_param

    def get_int_param(self) -> dict:
        return self._int_param
    
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")
    
    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")
    
    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")
    
    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")
    
    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    ####################
    ### publishers
    ####################
    @abstractmethod
    def send_can_frame(self, can_frame: HEXCANMessage):
        raise NotImplementedError("InterfaceBase.send_can_frame")

    ####################
    ### subscribers
    ####################
    def get_can_frame(self) -> typing.Optional[HEXCANMessage]:
        try:
            return self._hex_can_frame_queue.get_nowait()
        except queue.Empty:
            return None
        
    def get_target_speed(self) -> typing.Optional[list]:
        # todo. get target speed from ros
        return None
