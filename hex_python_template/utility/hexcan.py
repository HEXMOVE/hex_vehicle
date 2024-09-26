from enum import Enum

class HEXCANIDClass(Enum):
    CHASSIS = 0x01

class HEXCANIDStdFunction(Enum):
    """
    This is a list of standard functions that are used by all devices.
    """
    GENERAL_SETTING = 0x03      # General settings. This sets the enable state of the device
    CLEAR_ERROR = 0x04          # Clears error, if success
    CLEAR_KINESTATE = 0x05      # Clears kinematic state, this will reset odometry, if success
    FIND_DEVICE = 0x07          # Find device, not all devices support this
    HEARTBEAT = 0xB0

HEXCAN_ID_CLASS_CHASSIS = 0x01

class HEXCANMessage:
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

    def __str__(self):
        return f"can_id: {self.can_id}, data: {self.data}, extended: {self.extended}, time: {self.time}"
