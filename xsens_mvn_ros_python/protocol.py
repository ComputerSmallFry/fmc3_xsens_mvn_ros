"""Xsens MVN UDP protocol parsing.

The C++ code uses a small SDK submodule and a custom Streamer class to unpack
bytes one field at a time. In Python we can express the same layout much more
directly with ``struct.unpack_from``.

Packet layout used by the current project:

- 24 bytes fixed header
- variable payload whose format depends on the message type

All numeric fields are parsed as big-endian because that is what the original
C++ Streamer effectively reconstructs from the incoming byte sequence.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import ClassVar, List, Sequence

from .data_types import Quaternion, Vector3

HEADER_STRUCT = struct.Struct(">6sIBBIB7s")
HEADER_SIZE = HEADER_STRUCT.size


class StreamingProtocol(IntEnum):
    SPPoseEuler = 0x01
    SPPoseQuaternion = 0x02
    SPPosePositions = 0x03
    SPJackProcessSimulate = 0x04
    SPPoseUnity3D = 0x05
    SPMetaMoreMeta = 0x12
    SPMetaScaling = 0x13
    SPJointAngles = 0x20
    SPLinearSegmentKinematics = 0x21
    SPAngularSegmentKinematics = 0x22
    SPTrackerKinematics = 0x23
    SPCenterOfMass = 0x24
    SPTimeCode = 0x25


@dataclass(frozen=True)
class DatagramHeader:
    message_type_text: str
    protocol: StreamingProtocol
    sample_counter: int
    datagram_counter: int
    data_count: int
    frame_time: int
    avatar_id: int
    reserved: bytes

    @property
    def is_last_fragment(self) -> bool:
        """The top bit indicates the final UDP fragment of a sample."""

        return bool(self.datagram_counter & 0x80)

    @property
    def fragment_index(self) -> int:
        return self.datagram_counter & 0x7F


@dataclass(frozen=True)
class QuaternionKinematics:
    segment_id: int
    sensor_pos: Vector3
    quaternion: Quaternion


@dataclass(frozen=True)
class JointAngle:
    parent: int
    child: int
    parent_segment_id: int
    child_segment_id: int
    rotation: Vector3


@dataclass(frozen=True)
class LinearSegmentKinematics:
    segment_id: int
    pos: Vector3
    velocity: Vector3
    acceleration: Vector3


@dataclass(frozen=True)
class AngularSegmentKinematics:
    segment_id: int
    segment_orientation: Quaternion
    angular_velocity: Vector3
    angular_acceleration: Vector3


@dataclass
class BaseDatagram:
    header: DatagramHeader
    payload_length: int


@dataclass
class UnknownDatagram(BaseDatagram):
    raw_payload: bytes


@dataclass
class QuaternionDatagram(BaseDatagram):
    ITEM_STRUCT: ClassVar[struct.Struct] = struct.Struct(">i7f")

    items: List[QuaternionKinematics]

    def get_item(self, segment_id: int) -> QuaternionKinematics | None:
        for item in self.items:
            if item.segment_id == segment_id:
                return item
        return None


@dataclass
class JointAnglesDatagram(BaseDatagram):
    ITEM_STRUCT: ClassVar[struct.Struct] = struct.Struct(">ii3f")

    items: List[JointAngle]

    def get_item(self, parent_segment_id: int, child_segment_id: int) -> JointAngle | None:
        for item in self.items:
            if item.parent_segment_id == parent_segment_id and item.child_segment_id == child_segment_id:
                return item
        return None


@dataclass
class LinearSegmentKinematicsDatagram(BaseDatagram):
    ITEM_STRUCT: ClassVar[struct.Struct] = struct.Struct(">i9f")

    items: List[LinearSegmentKinematics]

    def get_item(self, segment_id: int) -> LinearSegmentKinematics | None:
        for item in self.items:
            if item.segment_id == segment_id:
                return item
        return None


@dataclass
class AngularSegmentKinematicsDatagram(BaseDatagram):
    ITEM_STRUCT: ClassVar[struct.Struct] = struct.Struct(">i10f")

    items: List[AngularSegmentKinematics]

    def get_item(self, segment_id: int) -> AngularSegmentKinematics | None:
        for item in self.items:
            if item.segment_id == segment_id:
                return item
        return None


@dataclass
class CenterOfMassDatagram(BaseDatagram):
    COM_STRUCT: ClassVar[struct.Struct] = struct.Struct(">3f")

    position: Vector3


def _parse_header(packet: bytes) -> DatagramHeader:
    if len(packet) < HEADER_SIZE:
        raise ValueError(f"Datagram is too short: got {len(packet)} bytes, need at least {HEADER_SIZE}.")

    raw_type, sample_counter, datagram_counter, data_count, frame_time, avatar_id, reserved = (
        HEADER_STRUCT.unpack_from(packet, 0)
    )
    message_type_text = raw_type.decode("ascii")
    if not message_type_text.startswith("MXTP"):
        raise ValueError(f"Unexpected datagram prefix: {message_type_text!r}")

    try:
        protocol_code = int(message_type_text[4:6], 16)
        protocol = StreamingProtocol(protocol_code)
    except ValueError as exc:
        raise ValueError(f"Unknown Xsens protocol code in header {message_type_text!r}") from exc

    return DatagramHeader(
        message_type_text=message_type_text,
        protocol=protocol,
        sample_counter=sample_counter,
        datagram_counter=datagram_counter,
        data_count=data_count,
        frame_time=frame_time,
        avatar_id=avatar_id,
        reserved=reserved,
    )


def _ensure_payload_size(
    packet: bytes,
    item_struct: struct.Struct,
    item_count: int,
    protocol: StreamingProtocol,
) -> None:
    expected_length = HEADER_SIZE + item_struct.size * item_count
    if len(packet) < expected_length:
        raise ValueError(
            f"{protocol.name} packet is truncated: got {len(packet)} bytes, expected at least {expected_length}."
        )


def _unpack_vector(values: Sequence[float]) -> Vector3:
    return (float(values[0]), float(values[1]), float(values[2]))


def _parse_quaternion_datagram(header: DatagramHeader, packet: bytes) -> QuaternionDatagram:
    _ensure_payload_size(packet, QuaternionDatagram.ITEM_STRUCT, header.data_count, header.protocol)

    items: List[QuaternionKinematics] = []
    offset = HEADER_SIZE
    for _ in range(header.data_count):
        segment_id, px, py, pz, qw, qx, qy, qz = QuaternionDatagram.ITEM_STRUCT.unpack_from(packet, offset)
        items.append(
            QuaternionKinematics(
                segment_id=segment_id,
                sensor_pos=(px, py, pz),
                quaternion=Quaternion(w=qw, x=qx, y=qy, z=qz),
            )
        )
        offset += QuaternionDatagram.ITEM_STRUCT.size

    return QuaternionDatagram(header=header, payload_length=offset - HEADER_SIZE, items=items)


def _parse_joint_angles_datagram(header: DatagramHeader, packet: bytes) -> JointAnglesDatagram:
    _ensure_payload_size(packet, JointAnglesDatagram.ITEM_STRUCT, header.data_count, header.protocol)

    items: List[JointAngle] = []
    offset = HEADER_SIZE
    for _ in range(header.data_count):
        parent, child, rx, ry, rz = JointAnglesDatagram.ITEM_STRUCT.unpack_from(packet, offset)
        items.append(
            JointAngle(
                parent=parent,
                child=child,
                parent_segment_id=parent // 256,
                child_segment_id=child // 256,
                rotation=(rx, ry, rz),
            )
        )
        offset += JointAnglesDatagram.ITEM_STRUCT.size

    return JointAnglesDatagram(header=header, payload_length=offset - HEADER_SIZE, items=items)


def _parse_linear_segment_kinematics_datagram(
    header: DatagramHeader, packet: bytes
) -> LinearSegmentKinematicsDatagram:
    _ensure_payload_size(packet, LinearSegmentKinematicsDatagram.ITEM_STRUCT, header.data_count, header.protocol)

    items: List[LinearSegmentKinematics] = []
    offset = HEADER_SIZE
    for _ in range(header.data_count):
        unpacked = LinearSegmentKinematicsDatagram.ITEM_STRUCT.unpack_from(packet, offset)
        segment_id = unpacked[0]
        items.append(
            LinearSegmentKinematics(
                segment_id=segment_id,
                pos=_unpack_vector(unpacked[1:4]),
                velocity=_unpack_vector(unpacked[4:7]),
                acceleration=_unpack_vector(unpacked[7:10]),
            )
        )
        offset += LinearSegmentKinematicsDatagram.ITEM_STRUCT.size

    return LinearSegmentKinematicsDatagram(header=header, payload_length=offset - HEADER_SIZE, items=items)


def _parse_angular_segment_kinematics_datagram(
    header: DatagramHeader, packet: bytes
) -> AngularSegmentKinematicsDatagram:
    _ensure_payload_size(packet, AngularSegmentKinematicsDatagram.ITEM_STRUCT, header.data_count, header.protocol)

    items: List[AngularSegmentKinematics] = []
    offset = HEADER_SIZE
    for _ in range(header.data_count):
        unpacked = AngularSegmentKinematicsDatagram.ITEM_STRUCT.unpack_from(packet, offset)
        segment_id = unpacked[0]
        items.append(
            AngularSegmentKinematics(
                segment_id=segment_id,
                segment_orientation=Quaternion(
                    w=unpacked[1],
                    x=unpacked[2],
                    y=unpacked[3],
                    z=unpacked[4],
                ),
                angular_velocity=_unpack_vector(unpacked[5:8]),
                angular_acceleration=_unpack_vector(unpacked[8:11]),
            )
        )
        offset += AngularSegmentKinematicsDatagram.ITEM_STRUCT.size

    return AngularSegmentKinematicsDatagram(header=header, payload_length=offset - HEADER_SIZE, items=items)


def _parse_center_of_mass_datagram(header: DatagramHeader, packet: bytes) -> CenterOfMassDatagram:
    expected_length = HEADER_SIZE + CenterOfMassDatagram.COM_STRUCT.size
    if len(packet) < expected_length:
        raise ValueError(
            f"{header.protocol.name} packet is truncated: got {len(packet)} bytes, expected at least {expected_length}."
        )

    position = CenterOfMassDatagram.COM_STRUCT.unpack_from(packet, HEADER_SIZE)
    return CenterOfMassDatagram(
        header=header,
        payload_length=CenterOfMassDatagram.COM_STRUCT.size,
        position=(position[0], position[1], position[2]),
    )


def parse_datagram(packet: bytes) -> BaseDatagram:
    """Parse a single UDP datagram.

    This function intentionally mirrors the current C++ behavior:
    one UDP packet in, one parsed datagram out. It does not attempt to
    reassemble fragmented samples spread over multiple UDP packets.
    """

    header = _parse_header(packet)

    if header.protocol == StreamingProtocol.SPPoseQuaternion:
        return _parse_quaternion_datagram(header, packet)
    if header.protocol == StreamingProtocol.SPJointAngles:
        return _parse_joint_angles_datagram(header, packet)
    if header.protocol == StreamingProtocol.SPLinearSegmentKinematics:
        return _parse_linear_segment_kinematics_datagram(header, packet)
    if header.protocol == StreamingProtocol.SPAngularSegmentKinematics:
        return _parse_angular_segment_kinematics_datagram(header, packet)
    if header.protocol == StreamingProtocol.SPCenterOfMass:
        return _parse_center_of_mass_datagram(header, packet)

    return UnknownDatagram(
        header=header,
        payload_length=max(len(packet) - HEADER_SIZE, 0),
        raw_payload=packet[HEADER_SIZE:],
    )


class ParserManager:
    """Small compatibility wrapper around ``parse_datagram``.

    The original C++ code stores only the last parsed datagram and exposes
    typed getters. Recreating that interface makes the Python version easier
    to compare against the existing codebase.
    """

    def __init__(self) -> None:
        self._datagram: BaseDatagram | None = None

    def read_datagram(self, packet: bytes) -> BaseDatagram:
        self._datagram = parse_datagram(packet)
        return self._datagram

    def get_quaternion_datagram(self) -> QuaternionDatagram | None:
        if isinstance(self._datagram, QuaternionDatagram):
            return self._datagram
        return None

    def get_joint_angles_datagram(self) -> JointAnglesDatagram | None:
        if isinstance(self._datagram, JointAnglesDatagram):
            return self._datagram
        return None

    def get_linear_segment_kinematics_datagram(self) -> LinearSegmentKinematicsDatagram | None:
        if isinstance(self._datagram, LinearSegmentKinematicsDatagram):
            return self._datagram
        return None

    def get_angular_segment_kinematics_datagram(self) -> AngularSegmentKinematicsDatagram | None:
        if isinstance(self._datagram, AngularSegmentKinematicsDatagram):
            return self._datagram
        return None

    def get_center_of_mass_datagram(self) -> CenterOfMassDatagram | None:
        if isinstance(self._datagram, CenterOfMassDatagram):
            return self._datagram
        return None
