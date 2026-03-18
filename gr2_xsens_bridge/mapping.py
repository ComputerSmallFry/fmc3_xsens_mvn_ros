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


def xsens_to_gr2(human_data: HumanDataHandler) -> Dict[str, List[float]]:
    """Convert current Xsens state to GR-2 upper-body position command.

    Returns dict with keys: waist, head, left_manipulator, right_manipulator.
    All values in radians.
    """

    # --- Waist (1 DOF): sum of spinal yaw (Z axis) ---
    spine_joints = ["l5_s1", "l4_l3", "l1_t12", "t9_t8"]
    waist_yaw = sum(_get_angles(human_data, j)[2] for j in spine_joints)
    waist = [_deg2rad(waist_yaw)]

    # --- Head (2 DOF): pan = Z, tilt = X ---
    head_angles = _get_angles(human_data, "c1_head")
    head = [_deg2rad(head_angles[2]), _deg2rad(head_angles[0])]

    # --- Right arm (7 DOF) ---
    r_sh = _get_angles(human_data, "right_shoulder")
    r_el = _get_angles(human_data, "right_elbow")
    r_wr = _get_angles(human_data, "right_wrist")
    right_manipulator = [
        _deg2rad(r_sh[0]),   # shoulder_pitch (flexion/extension)
        _deg2rad(r_sh[2]),   # shoulder_roll (abduction) — sign in SIGN_MAP
        _deg2rad(r_sh[1]),   # shoulder_yaw (int/ext rotation)
        _deg2rad(r_el[2]),   # elbow_pitch (flexion, post -90° offset)
        _deg2rad(r_wr[0]),   # wrist_pitch
        _deg2rad(r_wr[1]),   # wrist_roll
        _deg2rad(r_wr[2]),   # wrist_yaw
    ]

    # --- Left arm (7 DOF) ---
    l_sh = _get_angles(human_data, "left_shoulder")
    l_el = _get_angles(human_data, "left_elbow")
    l_wr = _get_angles(human_data, "left_wrist")
    left_manipulator = [
        _deg2rad(l_sh[0]),   # shoulder_pitch
        _deg2rad(l_sh[2]),   # shoulder_roll — sign in SIGN_MAP
        _deg2rad(l_sh[1]),   # shoulder_yaw
        _deg2rad(l_el[2]),   # elbow_pitch (post +90° offset)
        _deg2rad(l_wr[0]),   # wrist_pitch
        _deg2rad(l_wr[1]),   # wrist_roll
        _deg2rad(l_wr[2]),   # wrist_yaw
    ]

    # Apply per-joint sign corrections.
    cmd = {
        "waist": _apply_signs(waist, "waist"),
        "head": _apply_signs(head, "head"),
        "right_manipulator": _apply_signs(right_manipulator, "right_manipulator"),
        "left_manipulator": _apply_signs(left_manipulator, "left_manipulator"),
    }
    return cmd
