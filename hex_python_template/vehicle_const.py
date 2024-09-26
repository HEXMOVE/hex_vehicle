from enum import Enum


class HEXCANIDVehicleFunction(Enum):
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

class VehicleModel(Enum):
    DIFFERENTIAL = 0
    MECANUM = 1
    ARKERMAN = 2
    DELTA_OMNI = 3
    QUAD_OMNI = 4

VEHICLE_INFO = {
    0x01: {
        "name": "ARK",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.249,
        "wheel_spacing_length": 0,
    },
    0x02: {
        "name": "ARK-MINI",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.265,
        "wheel_spacing_length": 0,
    },
    0x03: {
        "name": "ARK-PLUS",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 999,
        "wheel_spacing_length": 0,
    },
    0x05: {
        "name": "RAY",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.198,
        "wheel_spacing_length": 0,
    },
    0x06: {
        "name": "ORCS-DIFF",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.520,
        "wheel_spacing_length": 0.410,
    },
    0x07: {
        "name": "ORCS-MCNM",
        "model": VehicleModel.MECANUM,
        "wheel_spacing_width": 0.468,
        "wheel_spacing_length": 0.412,
    },
    0x08: {
        "name": "ORCS-MCNM-PRO",
        "model": VehicleModel.MECANUM,
        "wheel_spacing_width": 0.508,
        "wheel_spacing_length": 0.412,
    },
    0x09: {
        "name": "ORCS-MINI",
        "model": VehicleModel.MECANUM,
        "wheel_spacing_width": 0.326,
        "wheel_spacing_length": 0.260,
    },
    0x0B: {
        "name": "NEOS",
        "model": VehicleModel.ARKERMAN,
        "wheel_spacing_width": 0.547,
        "wheel_spacing_length": 0.535,
    },
    0x0E: {
        "name": "HAMMER-80",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.63,
        "wheel_spacing_length": 0,
    },
    0x11: {
        "name": "TRIGGER-A",
        "model": VehicleModel.DELTA_OMNI,
        "wheel_spacing_width": 0.554,
        "wheel_spacing_length": 0,
    },
    0x12: {
        "name": "TRIGGER-X",
        "model": VehicleModel.QUAD_OMNI,
        "wheel_spacing_width": 999,
        "wheel_spacing_length": 0,
    },
    0xC1: {
        "name": "MARK_C1",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.44,
        "wheel_spacing_length": 0.335,
    },
    0xC2: {
        "name": "MARK_C3",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.525,
        "wheel_spacing_length": 0,
    },
    0xC3: {
        "name": "TRIGGER-C1",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.426,
        "wheel_spacing_length": 0,
    },
    0xC4: {
        "name": "TRIGGER-C2",
        "model": VehicleModel.QUAD_OMNI,
        "wheel_spacing_width": 0.485,
        "wheel_spacing_length": 0.550,
    },
    0xCF: {
        "name": "custom",
        "model": VehicleModel.DIFFERENTIAL,
        "wheel_spacing_width": 0.305,
        "wheel_spacing_length": 0,
    },
}

class VehicleMode(Enum):
    STANDBY = 0 # Recrive no order with motors locked
    REMOTER = 1 # Use remoter for control
    CAN = 2     # Use CAN bus for control
    FREE = 3    # Recrive no order with motors unlocked
