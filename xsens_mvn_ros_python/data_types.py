"""Lightweight data classes used by the Python client.

The original C++ project stores everything in Eigen-based structs.
Here we keep the same concepts, but use small Python dataclasses so the
parser can stay dependency-free.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import sqrt
from typing import Dict, Tuple

Vector3 = Tuple[float, float, float]


@dataclass
class Quaternion:
    """Quaternion stored in (w, x, y, z) order."""

    w: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def normalized(self) -> "Quaternion":
        """Return a normalized quaternion.

        Xsens packets already provide normalized quaternions, but normalizing
        again keeps the downstream state stable and mirrors the C++ behavior.
        """

        norm = sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if norm == 0.0:
            return Quaternion()
        return Quaternion(
            w=self.w / norm,
            x=self.x / norm,
            y=self.y / norm,
            z=self.z / norm,
        )

    def multiply(self, other: "Quaternion") -> "Quaternion":
        """Quaternion multiplication, used for the arm correction offsets."""

        return Quaternion(
            w=self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x=self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y=self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z=self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        ).normalized()


@dataclass
class Twist:
    linear: Vector3 = (0.0, 0.0, 0.0)
    angular: Vector3 = (0.0, 0.0, 0.0)


@dataclass
class LinkState:
    position: Vector3 = (0.0, 0.0, 0.0)
    orientation: Quaternion = field(default_factory=Quaternion)
    velocity: Twist = field(default_factory=Twist)
    acceleration: Twist = field(default_factory=Twist)


@dataclass
class Link:
    name: str
    parent_joint: str = ""
    length: Vector3 = (0.0, 0.0, 0.0)
    state: LinkState = field(default_factory=LinkState)


@dataclass
class JointState:
    angles: Vector3 = (0.0, 0.0, 0.0)


@dataclass
class Joint:
    name: str
    parent_link: str = ""
    child_link: str = ""
    state: JointState = field(default_factory=JointState)


class HumanDataHandler:
    """Simple in-memory state store.

    This mirrors the role of the original C++ HumanDataHandler class: the
    parser updates one shared object, and other code can read snapshots from it.
    """

    def __init__(self) -> None:
        self._joint_map: Dict[str, Joint] = {}
        self._link_map: Dict[str, Link] = {}
        self._com: Vector3 = (0.0, 0.0, 0.0)

    def set_joint(self, joint_name: str, joint: Joint) -> None:
        self._joint_map[joint_name] = joint

    def set_joint_angles(self, joint_name: str, joint_angles: Vector3) -> None:
        joint = self._joint_map.get(joint_name, Joint(name=joint_name))
        joint.state.angles = joint_angles
        self._joint_map[joint_name] = joint

    def get_joint(self, joint_name: str) -> Joint | None:
        return self._joint_map.get(joint_name)

    def set_link(self, link_name: str, link: Link) -> None:
        self._link_map[link_name] = link

    def set_link_pose(self, link_name: str, link_pos: Vector3, link_orient: Quaternion) -> None:
        link = self._link_map.get(link_name, Link(name=link_name))
        link.state.position = link_pos
        link.state.orientation = link_orient.normalized()
        self._link_map[link_name] = link

    def set_link_state(self, link_name: str, link_state: LinkState) -> None:
        link = self._link_map.get(link_name, Link(name=link_name))
        link.state = link_state
        link.state.orientation = link.state.orientation.normalized()
        self._link_map[link_name] = link

    def get_link(self, link_name: str) -> Link | None:
        return self._link_map.get(link_name)

    def get_links(self) -> Dict[str, Link]:
        return dict(self._link_map)

    def get_joints(self) -> Dict[str, Joint]:
        return dict(self._joint_map)

    def get_com(self) -> Vector3:
        return self._com

    def set_com(self, com: Vector3) -> None:
        self._com = com
