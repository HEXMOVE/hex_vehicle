"""
This file contains the CAN message classes for the vehicle.
Note this is just a demo, not all functions are implemented. Not all can messages are handled.
"""

from enum import Enum
import struct
from dataclasses import dataclass
import time
from utility.hexcan import HEXCANMessage
from vehicle_consts import VehicleMode

class HEXCANIDFunctionVehicle(Enum):
    """
    This is a list of functions that are used by all vehicles(id class = HEXCANIDClass.CHASSIS).
    """
    STATE_SET = 0x11
    STATE_REPORT = 0xB1
    SPEED_SET = 0x12
    SPEED_REPORT = 0xB2
    MAIN_ODOM_REPORT = 0xB3
    SECONDARY_ODOM_REPORT = 0xB4
    ERROR_REPORT = 0xB5


# ----------------- CAN ID Function classes -----------------

@dataclass
class VehicleFunctionStateSet:
    """
    This is a data class for the state set function.
    Byte 0: mode, value is from VehicleMode
    Byte 1: beep state, 0 to disable, 1 to enable
    Byte 2: brake state, 0 to disable, 1 to enable
    Byte 3: special state, 0 to disable, 1 to enable
    """
    mode: VehicleMode = VehicleMode.CAN
    beep: bool = 0
    brake: bool = 0
    special: bool = 0

    CONST_ID_FUNCTION = HEXCANIDFunctionVehicle.STATE_SET.value

    def to_bytes(self) -> bytes:
        return struct.pack("<BBBB", self.mode.value, self.beep, self.brake, self.special)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())
    
    @staticmethod
    def from_bytes(data: bytes) -> 'VehicleFunctionStateSet':
        return VehicleFunctionStateSet(VehicleMode(data[0]), *data[1:])
    
@dataclass
class VehicleFunctionStateReport:
    """
    This is a data class for the state report function.
    Byte 0: vehicle state, 0 normal, 1 error
    Byte 1: mode, value is from VehicleMode
    Byte 2-3: uint16, battery voltage in 0.1v
    Byte 4: beep state, 0 disabled, 1 enabled
    Byte 5: remoter state, 0 online, 1 offline
    Byte 6: brake state, 0 disabled, 1 enabled
    Byte 7: special state, 0 disabled, 1 enabled
    """
    state: bool = 0
    mode: VehicleMode = VehicleMode.CAN
    voltage: int = 0
    beep: bool = 0
    remoter: bool = 0
    brake: bool = 0
    special: bool = 0

    CONST_ID_FUNCTION = HEXCANIDFunctionVehicle.STATE_REPORT.value

    def to_bytes(self) -> bytes:
        return struct.pack("<BBHBBBB", self.state, self.mode.value, self.voltage, self.beep, self.remoter, self.brake, self.special)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())
    
    @staticmethod
    def from_bytes(data: bytes) -> 'VehicleFunctionStateReport':
        return VehicleFunctionStateReport(bool(data[0]), VehicleMode(data[1]), *data[2:])
    
@dataclass
class VehicleFunctionSpeedSet:
    """
    This is a data class for the speed set function.
    Byte 0-1: int16, x linear speed in mm/s
    Byte 2-3: int16, y linear speed in mm/s
    Byte 4-5: int16, angular speed in mrad/s. Arkerman should leave this 0
    Byte 6-7: int16, Arkerman steering angle in mrad, int16. Non-arkerman should leave this 0
    """
    x: int = 0
    y: int = 0
    w: int = 0
    a: int = 0

    CONST_ID_FUNCTION = HEXCANIDFunctionVehicle.SPEED_SET.value

    def to_bytes(self) -> bytes:
        return struct.pack("<hhhh", self.x, self.y, self.w, self.a)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())
    
    @staticmethod
    def from_bytes(data: bytes) -> 'VehicleFunctionSpeedSet':
        return VehicleFunctionSpeedSet(*struct.unpack("<hhhh", data))
    
@dataclass
class VehicleFunctionSpeedReport:
    """
    This is a data class for the speed report function.
    Byte 0-1: int16, x linear speed in mm/s
    Byte 2-3: int16, y linear speed in mm/s
    Byte 4-5: int16, angular speed in mrad/s. Arkerman will also report this as just a reference
    Byte 6-7: int16, Arkerman steering angle in mrad, int16. 
    """
    x: int = 0
    y: int = 0
    w: int = 0
    a: int = 0

    CONST_ID_FUNCTION = HEXCANIDFunctionVehicle.SPEED_REPORT.value

    def to_bytes(self) -> bytes:
        return struct.pack("<hhhh", self.x, self.y, self.w, self.a)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())
    
    @staticmethod
    def from_bytes(data: bytes) -> 'VehicleFunctionSpeedReport':
        return VehicleFunctionSpeedReport(*struct.unpack("<hhhh", data))

@dataclass
class VehicleFunctionMainOdomReport:
    """
    This is a data class for the main odom report function.
    Byte 0-3: int32, left wheel odometry in mm
    Byte 4-7: int32, right wheel odometry in mm
    """
    left: int = 0
    right: int = 0

    CONST_ID_FUNCTION = HEXCANIDFunctionVehicle.MAIN_ODOM_REPORT.value

    def to_bytes(self) -> bytes:
        return struct.pack("<ii", self.left, self.right)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())
    
    @staticmethod
    def from_bytes(data: bytes) -> 'VehicleFunctionMainOdomReport':
        return VehicleFunctionMainOdomReport(*struct.unpack("<ii", data))
    
@dataclass
class VehicleFunctionSecondaryOdomReport:
    """
    This is a data class for the secondary odom report function.
    Byte 0-3: int32, left back wheel odometry in mm
    Byte 4-7: int32, right back wheel odometry in mm
    """
    left_back: int = 0
    right_back: int = 0

    CONST_ID_FUNCTION = HEXCANIDFunctionVehicle.SECONDARY_ODOM_REPORT.value

    def to_bytes(self) -> bytes:
        return struct.pack("<ii", self.left_back, self.right_back)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())
    
    @staticmethod
    def from_bytes(data: bytes) -> 'VehicleFunctionSecondaryOdomReport':
        return VehicleFunctionSecondaryOdomReport(*struct.unpack("<ii", data))
    
@dataclass
class VehicleFunctionErrorReport:
    """
    This is a data class for the error report function.
    Byte 0: motor error code
    Byte 1: driver error code
    Byte 2: communication error code
    Byte 3: other error code
    Byte 4: power error code

    Note that this message allows constrcuting from bytes less than 5 bytes, since some chassis might not have all error codes.
    In that case, the rest of the error codes will be considered as 0.
    """
    error: int = 0

    CONST_ID_FUNCTION = HEXCANIDFunctionVehicle.ERROR_REPORT.value

    def to_bytes(self) -> bytes:
        return struct.pack("<BBBBB", self.error, 0, 0, 0, 0)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())
    
    @staticmethod
    def from_bytes(data: bytes) -> 'VehicleFunctionErrorReport':
        """
        If data input is less than 5 bytes, pad with 0.
        """
        return VehicleFunctionErrorReport(*struct.unpack("<BBBBB", data.ljust(5, b'\x00')))