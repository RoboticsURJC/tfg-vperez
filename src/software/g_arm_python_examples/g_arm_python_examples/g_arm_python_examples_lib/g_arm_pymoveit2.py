from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "tool"


def joint_names() -> List[str]:
    return [
        "joint1",
        "joint2",
        "joint3",
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "end_effector_tip"


def tool_joint_name() -> List[str]:
    return [
        "jointPWM"
    ]

def electromagnet_on() -> List[float]:
    return [1.0]

def electromagnet_off() -> List[float]:
    return [0.0]