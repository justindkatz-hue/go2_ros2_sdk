from enum import Enum


class RobotState(Enum):
    UNKNOWN  = "unknown"
    DAMPED   = "damped"    # lying flat, joints damped
    STANDING = "standing"  # balance stand, ready to move
    SITTING  = "sitting"
    MOVING   = "moving"    # executing walk or turn
