from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "tool"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.00001, 0.00001]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


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


def gripper_joint_names() -> List[str]:
    return [
        "jointPWM"
    ]
