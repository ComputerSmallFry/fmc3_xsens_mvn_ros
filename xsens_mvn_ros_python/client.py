"""High-level UDP client that mirrors the C++ XSensClient behavior."""

from __future__ import annotations

from dataclasses import asdict
import socket
from typing import Dict, Iterable, List, Tuple

from .data_types import HumanDataHandler, Joint, Link, LinkState, Quaternion
from .model import JOINT_NAMES, LINK_NAMES
from .protocol import (
    AngularSegmentKinematicsDatagram,
    BaseDatagram,
    CenterOfMassDatagram,
    JointAngle,
    JointAnglesDatagram,
    LinearSegmentKinematicsDatagram,
    ParserManager,
    QuaternionDatagram,
)

MAX_MVN_DATAGRAM_SIZE = 5000


class XSensUDPClient:
    """Receive Xsens UDP datagrams and keep an in-memory human model updated."""

    def __init__(self, udp_port: int = 8001, bind_host: str = "0.0.0.0") -> None:
        self.udp_port = udp_port
        self.bind_host = bind_host
        self.parser_manager = ParserManager()
        self.human_data = HumanDataHandler()
        self.link_name_list: List[str] = []
        self.joint_name_list: List[str] = []
        self.segment_to_link_name: Dict[int, str] = {}
        self.socket: socket.socket | None = None

    def bind(self) -> None:
        """Open and bind the UDP socket."""

        if self.udp_port < 2000:
            raise ValueError("Port number should be bigger than 2000")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.bind_host, self.udp_port))

    def read_data(self) -> BaseDatagram:
        """Read one UDP packet and parse it into a typed datagram."""

        if self.socket is None:
            raise RuntimeError("Socket is not bound yet. Call bind() first.")

        packet, _sender = self.socket.recvfrom(MAX_MVN_DATAGRAM_SIZE)
        return self.parser_manager.read_datagram(packet)

    def wait_for_quaternion_datagram(self) -> QuaternionDatagram:
        while True:
            datagram = self.read_data()
            if isinstance(datagram, QuaternionDatagram):
                return datagram

    def wait_for_joint_angles_datagram(self) -> JointAnglesDatagram:
        while True:
            datagram = self.read_data()
            if isinstance(datagram, JointAnglesDatagram):
                return datagram

    def bootstrap_from_datagrams(
        self,
        quaternion_datagram: QuaternionDatagram,
        joint_angles_datagram: JointAnglesDatagram,
    ) -> None:
        """Initialize the in-memory model from already parsed bootstrap packets.

        This helper lets the CLI inspect incoming packets during bootstrap while
        still reusing the same model-building logic afterward.
        """

        self.link_name_list.clear()
        self.joint_name_list.clear()
        self.segment_to_link_name.clear()

        for item in quaternion_datagram.items:
            if item.segment_id < 0 or item.segment_id >= len(LINK_NAMES):
                continue
            link_name = LINK_NAMES[item.segment_id]
            self.human_data.set_link(link_name, Link(name=link_name))
            self.link_name_list.append(link_name)
            self.segment_to_link_name[item.segment_id] = link_name

        for index, _item in enumerate(joint_angles_datagram.items):
            if index >= len(JOINT_NAMES):
                break
            joint_name = JOINT_NAMES[index]
            self.human_data.set_joint(joint_name, Joint(name=joint_name))
            self.joint_name_list.append(joint_name)

    def build_model(self) -> None:
        """Bootstrap link/joint names from the first quaternion and joint packets."""

        quaternion_datagram = self.wait_for_quaternion_datagram()
        joint_angles_datagram = self.wait_for_joint_angles_datagram()
        self.bootstrap_from_datagrams(quaternion_datagram, joint_angles_datagram)

    def process_next_packet(self) -> BaseDatagram:
        """Read one packet and apply it to the internal state."""

        datagram = self.read_data()
        self.handle_datagram(datagram)
        return datagram

    def handle_datagram(self, datagram: BaseDatagram) -> None:
        if isinstance(datagram, JointAnglesDatagram):
            self._update_joint_angles(datagram)
        elif isinstance(datagram, QuaternionDatagram):
            self._update_link_poses(datagram)
        elif isinstance(datagram, LinearSegmentKinematicsDatagram):
            self._update_link_linear_twists(datagram)
        elif isinstance(datagram, AngularSegmentKinematicsDatagram):
            self._update_link_angular_twists(datagram)
        elif isinstance(datagram, CenterOfMassDatagram):
            self.human_data.set_com(datagram.position)

    def run_forever(self) -> None:
        """Convenience loop for quick local debugging."""

        while True:
            self.process_next_packet()

    def snapshot(self) -> dict:
        """Return a JSON-friendly snapshot of the current human state."""

        links = {
            name: {
                "name": link.name,
                "position": link.state.position,
                "orientation": asdict(link.state.orientation),
                "velocity": {
                    "linear": link.state.velocity.linear,
                    "angular": link.state.velocity.angular,
                },
                "acceleration": {
                    "linear": link.state.acceleration.linear,
                    "angular": link.state.acceleration.angular,
                },
            }
            for name, link in self.human_data.get_links().items()
        }
        joints = {
            name: {
                "name": joint.name,
                "angles": joint.state.angles,
            }
            for name, joint in self.human_data.get_joints().items()
        }
        return {
            "links": links,
            "joints": joints,
            "com": self.human_data.get_com(),
        }

    def _update_joint_angles(self, datagram: JointAnglesDatagram) -> None:
        """Reproduce the C++ segment-to-joint mapping and axis remapping."""

        self._set_joint_angles(datagram, "l5_s1", 1, 2, 1, 1, 1)
        self._set_joint_angles(datagram, "l4_l3", 2, 3, 1, 1, 1)
        self._set_joint_angles(datagram, "l1_t12", 3, 4, 1, 1, 1)
        self._set_joint_angles(datagram, "t9_t8", 4, 5, 1, 1, 1)
        self._set_joint_angles(datagram, "t1_c7", 5, 6, 1, 1, 1)
        self._set_joint_angles(datagram, "c1_head", 6, 7, 1, 1, 1)
        self._set_joint_angles(datagram, "right_c7_shoulder", 5, 8, -1, 1, -1)
        self._set_joint_angles(datagram, "right_shoulder", 8, 9, -1, 1, -1)
        self._set_joint_angles(datagram, "right_elbow", 9, 10, -1, 1, -1)
        self._set_joint_angles(datagram, "right_wrist", 10, 11, -1, 1, -1)
        self._set_joint_angles(datagram, "left_c7_shoulder", 5, 12, 1, -1, -1)
        self._set_joint_angles(datagram, "left_shoulder", 12, 13, 1, -1, -1)
        self._set_joint_angles(datagram, "left_elbow", 13, 14, 1, -1, -1)
        self._set_joint_angles(datagram, "left_wrist", 14, 15, 1, -1, -1)
        self._set_joint_angles(datagram, "right_hip", 1, 16, -1, 1, -1)
        self._set_joint_angles(datagram, "right_knee", 16, 17, -1, 1, 1)
        self._set_joint_angles(datagram, "right_ankle", 17, 18, -1, 1, -1)
        self._set_joint_angles(datagram, "right_ballfoot", 18, 19, -1, 1, -1)
        self._set_joint_angles(datagram, "left_hip", 1, 20, 1, -1, -1)
        self._set_joint_angles(datagram, "left_knee", 20, 21, 1, -1, 1)
        self._set_joint_angles(datagram, "left_ankle", 21, 22, 1, -1, -1)
        self._set_joint_angles(datagram, "left_ballfoot", 22, 23, 1, -1, -1)

        # Same elbow offset correction as the original C++ node.
        right_elbow = self.human_data.get_joint("right_elbow")
        if right_elbow is not None:
            angles = right_elbow.state.angles
            right_elbow.state.angles = (angles[0], angles[1], angles[2] - 90.0)
            self.human_data.set_joint("right_elbow", right_elbow)

        left_elbow = self.human_data.get_joint("left_elbow")
        if left_elbow is not None:
            angles = left_elbow.state.angles
            left_elbow.state.angles = (angles[0], angles[1], angles[2] + 90.0)
            self.human_data.set_joint("left_elbow", left_elbow)

    def _set_joint_angles(
        self,
        datagram: JointAnglesDatagram,
        joint_name: str,
        parent_segment_id: int,
        child_segment_id: int,
        x_axis: int,
        y_axis: int,
        z_axis: int,
    ) -> None:
        joint_angle = datagram.get_item(parent_segment_id, child_segment_id)
        if joint_angle is None:
            return
        self.human_data.set_joint_angles(
            joint_name,
            self._joint_angle_to_vector3d(joint_angle, x_axis, y_axis, z_axis),
        )

    def _joint_angle_to_vector3d(
        self,
        joint_angle: JointAngle,
        x_axis: int,
        y_axis: int,
        z_axis: int,
    ) -> Tuple[float, float, float]:
        """Mirror the axis shuffle in the C++ helper.

        The original code maps:

        - output x <- input rotation[0]
        - output y <- input rotation[2]
        - output z <- input rotation[1]
        """

        return (
            x_axis * joint_angle.rotation[0],
            z_axis * joint_angle.rotation[2],
            y_axis * joint_angle.rotation[1],
        )

    def _update_link_poses(self, datagram: QuaternionDatagram) -> None:
        for segment_id, link_name in self.segment_to_link_name.items():
            item = datagram.get_item(segment_id)
            if item is None:
                continue
            self.human_data.set_link_pose(link_name, item.sensor_pos, item.quaternion)

        # These fixed rotations are copied from the C++ implementation.
        self._rotate_link("right_upper_arm", Quaternion(w=0.7071068, x=-0.7071068, y=0.0, z=0.0))
        self._rotate_link("right_forearm", Quaternion(w=0.7071068, x=-0.7071068, y=0.0, z=0.0))
        self._rotate_link("right_hand", Quaternion(w=0.7071068, x=-0.7071068, y=0.0, z=0.0))

        self._rotate_link("left_upper_arm", Quaternion(w=0.7071068, x=0.7071068, y=0.0, z=0.0))
        self._rotate_link("left_forearm", Quaternion(w=0.7071068, x=0.7071068, y=0.0, z=0.0))
        self._rotate_link("left_hand", Quaternion(w=0.7071068, x=0.7071068, y=0.0, z=0.0))

    def _rotate_link(self, link_name: str, correction: Quaternion) -> None:
        link = self.human_data.get_link(link_name)
        if link is None:
            return
        link.state.orientation = link.state.orientation.multiply(correction)
        self.human_data.set_link(link_name, link)

    def _update_link_linear_twists(self, datagram: LinearSegmentKinematicsDatagram) -> None:
        for segment_id, link_name in self.segment_to_link_name.items():
            item = datagram.get_item(segment_id)
            link = self.human_data.get_link(link_name)
            if item is None or link is None:
                continue
            state = link.state
            state.velocity.linear = item.velocity
            state.acceleration.linear = item.acceleration
            self.human_data.set_link_state(link_name, state)

    def _update_link_angular_twists(self, datagram: AngularSegmentKinematicsDatagram) -> None:
        for segment_id, link_name in self.segment_to_link_name.items():
            item = datagram.get_item(segment_id)
            link = self.human_data.get_link(link_name)
            if item is None or link is None:
                continue
            state = link.state
            state.velocity.angular = item.angular_velocity
            state.acceleration.angular = item.angular_acceleration
            self.human_data.set_link_state(link_name, state)
