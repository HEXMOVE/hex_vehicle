from enum import Enum
import struct
from dataclasses import dataclass
import time

# ----------------- CAN ID constants -----------------
"""
This is a list of constants that are used by all devices.
"""
class HEXCANIDClass(Enum):
    CHASSIS = 0x01

class HEXCANIDFunctionStd(Enum):
    """
    This is a list of standard functions that are used by all devices.
    Note this list is not exhaustive, and each device may have its own functions.
    Also, this list is a full list of std functions, becuase this is just a demo.
    """
    GENERAL_SETTING = 0x03      # General settings. This sets the enable state of the device
    CLEAR_ERROR = 0x04          # Clears error, if success
    CLEAR_KINESTATE = 0x05      # Clears kinematic state, this will reset odometry, if success
    FIND_DEVICE = 0x07          # Find device, not all devices support this
    HEARTBEAT = 0xB0


# ----------------- CAN Message class -----------------
class HEXCANMessage:
    """
    The class to represent a CAN message through ROS1 and ROS2.
    """
    def __init__(self, can_id:int, data:bytes, extended:bool, time:float):
        self.can_id = can_id
        self.data = data
        self.extended = extended
        self.time = time

    def get_id_class(self) -> int:
        if not self.extended:
            raise ValueError("Can not get class from standard ID")
        return (self.can_id >> 24) & 0xFF
    
    def get_id_type(self) -> int:
        if not self.extended:
            raise ValueError("Can not get type from standard ID")
        return (self.can_id >> 16) & 0xFF
    
    def get_id_num(self) -> int:
        if not self.extended:
            raise ValueError("Can not get num from standard ID")
        return (self.can_id >> 8) & 0xFF
    
    def get_id_function(self) -> int:
        if not self.extended:
            raise ValueError("Can not get function from standard ID")
        return self.can_id & 0xFF
    
    def get_len(self) -> int:
        return len(self.data)

    def __str__(self):
        return f"can_id: {self.can_id}, data: {self.data}, extended: {self.extended}, time: {self.time}"



# ----------------- CAN ID Function classes -----------------
"""
This is a list of classes that represent the functions that are used by all devices.
All classes should have the following methods:
    to_bytes: convert the class to bytes
    to_hexcan_message: convert the class to HEXCANMessage
    from_bytes: convert bytes to the class
"""

@dataclass
class StdFunctionGeneralSetting:
    """
    This is a data class for the general setting function.
    Byte 0: same as id class
    Byte 1: same as id type
    Byte 2: same as id num
    Byte 3: 1 to enable, 0 to disable
    """
    idc: int = 0
    idt: int = 0
    idn: int = 0
    enable: bool = False

    CONST_ID_FUNCTION = HEXCANIDFunctionStd.GENERAL_SETTING.value

    def to_bytes(self) -> bytes:
        return struct.pack("<BBBB", self.idc, self.idt, self.idn, self.enable)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())

    @staticmethod
    def from_bytes(data: bytes) -> 'StdFunctionGeneralSetting':
        return StdFunctionGeneralSetting(*struct.unpack("<BBBB", data))

@dataclass
class StdFunctionClearError:
    """
    This is a data class for the clear error function.
    Byte 0: Always 0xCC
    Byte 1-7: Pad with 0
    """
    clear: int = 0xCC

    CONST_ID_FUNCTION = HEXCANIDFunctionStd.CLEAR_ERROR.value

    def to_bytes(self) -> bytes:
        return struct.pack("<B7x", self.clear)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())

    @staticmethod
    def from_bytes(data: bytes) -> 'StdFunctionClearError':
        return StdFunctionClearError(*struct.unpack("<B7x", data))

@dataclass
class StdFunctionClearKinematicState:
    """
    This is a data class for the clear kinematic state function.
    Byte 0: Always 0xCC
    Byte 1-7: Pad with 0
    """
    clear: int = 0xCC

    CONST_ID_FUNCTION = HEXCANIDFunctionStd.CLEAR_KINESTATE.value

    def to_bytes(self) -> bytes:
        return struct.pack("<B7x", self.clear)
    
    def to_hexcan_message(self, can_id: int) -> 'HEXCANMessage':
        return HEXCANMessage(can_id | self.CONST_ID_FUNCTION, self.to_bytes(), True, time.time())

    @staticmethod
    def from_bytes(data: bytes) -> 'StdFunctionClearKinematicState':
        return StdFunctionClearKinematicState(*struct.unpack("<B7x", data))
