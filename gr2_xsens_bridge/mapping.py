"""Pure mapping function: Xsens joint angles -> GR-2 upper-body command dict."""

from __future__ import annotations

import math
from typing import Dict, List

from xsens_mvn_ros_python.data_types import HumanDataHandler

from . import config


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _get_angles(human_data: HumanDataHandler, name: str):
    """Return (x, y, z) in degrees, or (0, 0, 0) if joint missing."""
    joint = human_data.get_joint(name)
    if joint is None:
        return (0.0, 0.0, 0.0)
    return joint.state.angles


def _apply_signs(values: List[float], group: str) -> List[float]:
    signs = config.SIGN_MAP.get(group, [1.0] * len(values))
    return [v * s for v, s in zip(values, signs)]


def _apply_offsets(values: List[float], group: str) -> List[float]:
    offsets = config.JOINT_OFFSETS.get(group, [0.0] * len(values))
    return [v - o for v, o in zip(values, offsets)]


def _map_group_from_sources(human_data: HumanDataHandler, group: str) -> List[float]:
    mapped: List[float] = []
    for joint_name, axis_index in config.JOINT_SOURCE_MAP[group]:
        angles = _get_angles(human_data, joint_name)
        mapped.append(_deg2rad(angles[axis_index]))
    return mapped


def xsens_to_gr2(human_data: HumanDataHandler) -> Dict[str, List[float]]:
    """Convert current Xsens state to GR-2 upper-body position command.

    Returns dict with keys: waist, head, left_manipulator, right_manipulator.
    All values in radians.
    """

    # --- Waist (1 DOF): sum of spinal yaw (Z axis) ---
    spine_joints = ["l5_s1", "l4_l3", "l1_t12", "t9_t8"]
    waist_yaw = sum(_get_angles(human_data, j)[2] for j in spine_joints)
    waist = [_deg2rad(waist_yaw)]

    # --- Head (2 DOF): configurable Xsens axis selection ---
    head = _map_group_from_sources(human_data, "head")

    # --- Arms (7 DOF each): configurable Xsens joint/axis selection ---
    right_manipulator = _map_group_from_sources(human_data, "right_manipulator")
    left_manipulator = _map_group_from_sources(human_data, "left_manipulator")

    # Apply per-joint sign corrections and manual neutral offsets.
    cmd = {
        "waist": _apply_offsets(_apply_signs(waist, "waist"), "waist"),
        "head": _apply_offsets(_apply_signs(head, "head"), "head"),
        "right_manipulator": _apply_offsets(
            _apply_signs(right_manipulator, "right_manipulator"),
            "right_manipulator",
        ),
        "left_manipulator": _apply_offsets(
            _apply_signs(left_manipulator, "left_manipulator"),
            "left_manipulator",
        ),
    }
    return cmd
