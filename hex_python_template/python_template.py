#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Kison He. All rights reserved.
# Author: Kison He kisonhepenpal@gmail.com
# Date  : 2024-09-25
################################################################

import os
import sys
import time
import typing

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)

from utility import DataInterface
from utility.hexcan import HEXCANMessage
from utility.hexcan import HEXCANIDClass, HEXCANIDFunctionStd, StdFunctionGeneralSetting
from vehicle_can_msgs import HEXCANIDFunctionVehicle, VehicleFunctionStateSet, VehicleFunctionStateReport, \
    VehicleFunctionSpeedSet, VehicleFunctionSpeedReport, VehicleFunctionMainOdomReport, VehicleFunctionSecondaryOdomReport
from vehicle_consts import VehicleModel, VehicleMode, VEHICLE_INFO


class Vehicle:
    def __init__(self, caculate_odom_from_speed = False):
        self.__data_interface = DataInterface("xnode_vehicle", 100)
        self.__str_param = self.__data_interface.get_str_param()
        self.__int_param = self.__data_interface.get_int_param()

        self._can_id = None # Init can id to None, to be determined by picking up the can id from the can bus.
        self._target_mode = VehicleMode.CAN
        self._current_mode = None
        self._current_enabled = None

        self._caculate_odom_from_speed = caculate_odom_from_speed

        self._target_speed = [0, 0, 0] # x,y,z. x,y in m/s, z in rad/s
        self._target_speed_got_time = None
        self._cmd_vel_timeout = 0.5 # If we don't get target speed in this time, we should stop the vehicle
        self._read_speed = [0, 0, 0] # x,y,z. x,y in m/s, z in rad/s todo what about arkerman steering angle?
        self._odom = [0, 0, 0] # x,y,z. x,y in m, z in rad. 

        # target vehicle status
        self.target_brake_state = 0 # 0=off 1=on

        # vehicle status
        self.work_state = None # 0=fine 1=error
        self.brake_state = None # 0=off 1=on
        self.work_mode = None # Enum VehicleMode
        self.battery_vol = None # (V) float voltage of battery
        self.err_info = ""

        # Safety measures
        self.last_heartbeat_time = None # time of last heartbeat message received

    def get_vehicle_info(self) -> typing.Optional[dict]:
        if self._can_id is None:
            return None
        idc = (self._can_id >> 24) & 0xFF
        if idc not in VEHICLE_INFO:
            return None
        return VEHICLE_INFO[idc]

    def handle_state_report(self, can_msg: HEXCANMessage):
        state = VehicleFunctionStateReport.from_bytes(can_msg.data)
        self.work_state = state.state
        self._current_mode = state.mode
        self.battery_vol = state.voltage / 10.0
        self.brake_state = state.brake

    def handle_speed_report(self, can_msg: HEXCANMessage):
        spd = VehicleFunctionSpeedReport.from_bytes(can_msg.data)

        self._read_speed[0] = spd.x / 1000.0
        self._read_speed[1] = spd.y / 1000.0
        if self.get_vehicle_info()["model"] == VehicleModel.ARKERMAN:
            self._read_speed[2] = spd.a / 1000.0
        else:
            self._read_speed[2] = spd.w / 1000.0

    def handle_main_odom_report(self, can_msg: HEXCANMessage):
        # todo
        pass

    def handle_secondary_odom_report(self, can_msg: HEXCANMessage):
        # todo
        pass

    def handle_error_report(self, can_msg: HEXCANMessage):
        # If any byte is not 0, then there is an error.
        # Let's be simple so we just tell user which bit of which byte is not 0
        # If none of the byte is not 0, then set err_info to ""
        has_error = False
        for idx, x in enumerate(can_msg.data):
            if x != 0:
                has_error = True
                self.err_info = f"Byte {idx}: {x}"
        if not has_error:
            self.err_info = ""

    def handle_can_msg(self, can_msg: HEXCANMessage):
        # Check if the message is extended
        if not can_msg.extended:
            return
        
        # Find the first B0 message to set as our target device
        if self._can_id is None:
            # self.__data_interface.logi("Got idc 0x{:02X} idt 0x{:02X} idn 0x{:02X} idf 0x{:02X}".format(
            #     can_msg.get_id_class(), can_msg.get_id_type(), can_msg.get_id_num(), can_msg.get_id_function()))
            # Wait for first B0 message to set as our target device:
            if can_msg.get_id_class() == HEXCANIDClass.CHASSIS.value and can_msg.get_id_function() == HEXCANIDFunctionStd.HEARTBEAT.value:
                # Make sure the device is in our list
                if can_msg.get_id_type() not in VEHICLE_INFO:
                    self.__data_interface.logw(f"Unknown device: {can_msg.can_id}")
                    return
                self.__data_interface.logi(f"Found device: {VEHICLE_INFO[can_msg.get_id_type()]['name']} 0x{can_msg.can_id:08X}")
                self._can_id = can_msg.can_id & 0xFFFFFF00
                self.last_heartbeat_time = can_msg.time
            return
        
        # Check if the message is from our target device,
        # If it is, the id class, id type, and id num should be the same as our target device
        if (can_msg.can_id & 0xFFFFFF00) != (self._can_id & 0xFFFFFF00):
            return

        id_function = can_msg.get_id_function()
        if id_function == HEXCANIDFunctionVehicle.STATE_REPORT.value:
            self.handle_state_report(can_msg)
        elif id_function == HEXCANIDFunctionVehicle.SPEED_REPORT.value:
            self.handle_speed_report(can_msg)
        elif id_function == HEXCANIDFunctionVehicle.MAIN_ODOM_REPORT.value:
            self.handle_main_odom_report(can_msg)
        elif id_function == HEXCANIDFunctionVehicle.SECONDARY_ODOM_REPORT.value:
            self.handle_secondary_odom_report(can_msg)
        elif id_function == HEXCANIDFunctionVehicle.ERROR_REPORT.value:
            self.handle_error_report(can_msg)
        elif id_function == HEXCANIDFunctionStd.HEARTBEAT.value:
            self.last_heartbeat_time = can_msg.time
            self._current_enabled = bool(can_msg.data[0])
        else:
            # Ignore unknown function
            # Note this demo code does not handle all functions, because it is a demo :)
            # You can refer to docs for all functions and implement them yourself. And PR is welcome!
            pass
        pass
    
    def generate_can_msg(self) -> list:
        """
        This func will generate a list of HEXCANMessage to be sent to the chassis.
        """
        # First, check if we've got a target device
        if self._can_id is None:
            return []
        
        # Then, check is the chassis still online?
        now = time.time()
        if now - self.last_heartbeat_time > 1.5:
            # If the chassis is offline, we should send all 0 to the chassis to stop it, and apply brake
            self.__data_interface.logw("Chassis offline, trying to send stop and brake anyway")
            self._target_speed = [0, 0, 0]
            self.target_brake_state = 1
        
        # Gen can message. Always send SPEED_SET message, and if mode or brake state does not match, 
        msg_list = []
        if self._current_enabled != True:
            msg_list.append(StdFunctionGeneralSetting(self._can_id >> 24, (self._can_id >> 16) & 0xFF, (self._can_id >> 8) & 0xFF, True).to_hexcan_message(self._can_id))
            self.__data_interface.logi("Trying to enable the chassis")
            return msg_list
        
        # SPEED_SET message
        msg = HEXCANMessage(self._can_id | HEXCANIDFunctionVehicle.SPEED_SET.value, b'', True, now)
        # If _target_speed_got_time is None or too old, we should stop the vehicle
        if self._target_speed_got_time is None or now - self._target_speed_got_time > self._cmd_vel_timeout:
            self._target_speed = [0, 0, 0]        
        if self.get_vehicle_info()["model"] == VehicleModel.ARKERMAN:
            msg_list.append(VehicleFunctionSpeedSet(x=self._target_speed[0], y=self._target_speed[1], w=0, a=self._target_speed[2]).to_hexcan_message(self._can_id))
        else:
            msg_list.append(VehicleFunctionSpeedSet(x=self._target_speed[0], y=self._target_speed[1], w=self._target_speed[2], a=0).to_hexcan_message(self._can_id))

        # STATE_SET message
        if self._current_mode != self._target_mode.value or self.brake_state != self.target_brake_state:
            msg_list.append(VehicleFunctionStateSet(mode=self._target_mode, beep=0, brake=self.target_brake_state, special=0).to_hexcan_message(self._can_id))
            self.__data_interface.logi(f"Mode or brake state mismatch, Target mode: {self._target_mode}, Real mode: {self._current_mode}, Target brake: {self.target_brake_state}, Real brake: {self.brake_state}")
        return msg_list

    def run(self):
        self.__data_interface.logi("Vehicle started")
        self.__data_interface.logi("Waiting for a chassis to connect")
        while self.__data_interface.ok():
            # Fisrt, receive all can messages (from ros) and process 
            can_msg = self.__data_interface.get_can_frame()
            while can_msg is not None:
                self.handle_can_msg(can_msg)
                can_msg = self.__data_interface.get_can_frame()

            # Then, receive other ros messages
            spd = self.__data_interface.get_target_speed()
            if spd is not None:
                self._target_speed_got_time = time.time()
                self._target_speed = spd
            # Then get brake state
            
            # Last, generate can messages to be sent to the chassis (via ros)
            can_msg = self.generate_can_msg()
            for msg in can_msg:
                self.__data_interface.send_can_frame(msg)
            
            # You can periodically log info about the vehicle to debug or monitor
            # self.__data_interface.logi(f"Read Speed: {self._read_speed} Odom: {self._odom} BatteryVol: {self.battery_vol} Error: {self.err_info}")
            self.__data_interface.sleep()

def main():
    vehicle = Vehicle()
    vehicle.run()

if __name__ == '__main__':
    main()
