import struct
import unittest

from xsens_mvn_ros_python.client import XSensUDPClient
from xsens_mvn_ros_python.protocol import (
    AngularSegmentKinematicsDatagram,
    CenterOfMassDatagram,
    JointAnglesDatagram,
    LinearSegmentKinematicsDatagram,
    QuaternionDatagram,
    parse_datagram,
)


def build_header(message_type: bytes, data_count: int) -> bytes:
    return struct.pack(
        ">6sIBBIB7s",
        message_type,
        123,
        0x80,
        data_count,
        456,
        0,
        b"\x00" * 7,
    )


class ProtocolParsingTest(unittest.TestCase):
    def test_parse_quaternion_packet(self) -> None:
        packet = build_header(b"MXTP02", 1) + struct.pack(">i7f", 1, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)
        datagram = parse_datagram(packet)

        self.assertIsInstance(datagram, QuaternionDatagram)
        self.assertEqual(datagram.header.sample_counter, 123)
        self.assertEqual(datagram.items[0].segment_id, 1)
        self.assertEqual(datagram.items[0].sensor_pos, (1.0, 2.0, 3.0))
        self.assertEqual(datagram.items[0].quaternion.w, 1.0)

    def test_parse_joint_angles_packet(self) -> None:
        packet = build_header(b"MXTP20", 1) + struct.pack(">ii3f", 256, 512, 10.0, 20.0, 30.0)
        datagram = parse_datagram(packet)

        self.assertIsInstance(datagram, JointAnglesDatagram)
        item = datagram.get_item(1, 2)
        self.assertIsNotNone(item)
        self.assertEqual(item.parent_segment_id, 1)
        self.assertEqual(item.child_segment_id, 2)
        self.assertEqual(item.rotation, (10.0, 20.0, 30.0))

    def test_parse_linear_segment_kinematics_packet(self) -> None:
        packet = build_header(b"MXTP21", 1) + struct.pack(
            ">i9f",
            3,
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
        )
        datagram = parse_datagram(packet)

        self.assertIsInstance(datagram, LinearSegmentKinematicsDatagram)
        self.assertEqual(datagram.items[0].pos, (1.0, 2.0, 3.0))
        self.assertEqual(datagram.items[0].velocity, (4.0, 5.0, 6.0))
        self.assertEqual(datagram.items[0].acceleration, (7.0, 8.0, 9.0))

    def test_parse_angular_segment_kinematics_packet(self) -> None:
        packet = build_header(b"MXTP22", 1) + struct.pack(
            ">i10f",
            4,
            1.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.2,
            0.3,
            0.4,
            0.5,
            0.6,
        )
        datagram = parse_datagram(packet)

        self.assertIsInstance(datagram, AngularSegmentKinematicsDatagram)
        self.assertEqual(datagram.items[0].segment_orientation.w, 1.0)
        for actual, expected in zip(datagram.items[0].angular_velocity, (0.1, 0.2, 0.3)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(datagram.items[0].angular_acceleration, (0.4, 0.5, 0.6)):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_parse_center_of_mass_packet(self) -> None:
        packet = build_header(b"MXTP24", 1) + struct.pack(">3f", 1.0, 2.0, 3.0)
        datagram = parse_datagram(packet)

        self.assertIsInstance(datagram, CenterOfMassDatagram)
        self.assertEqual(datagram.position, (1.0, 2.0, 3.0))


class ClientBehaviorTest(unittest.TestCase):
    def test_joint_mapping_and_axis_shuffle(self) -> None:
        client = XSensUDPClient()

        # Pre-create joints used by the update path.
        from xsens_mvn_ros_python.data_types import Joint

        client.human_data.set_joint("l5_s1", Joint(name="l5_s1"))
        client.human_data.set_joint("right_elbow", Joint(name="right_elbow"))

        packet = build_header(b"MXTP20", 2)
        packet += struct.pack(">ii3f", 256, 512, 10.0, 20.0, 30.0)
        packet += struct.pack(">ii3f", 9 * 256, 10 * 256, 1.0, 2.0, 3.0)
        datagram = parse_datagram(packet)

        client.handle_datagram(datagram)

        self.assertEqual(client.human_data.get_joint("l5_s1").state.angles, (10.0, 30.0, 20.0))
        self.assertEqual(client.human_data.get_joint("right_elbow").state.angles, (-1.0, -3.0, -88.0))


if __name__ == "__main__":
    unittest.main()
