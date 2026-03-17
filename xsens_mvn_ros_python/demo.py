"""Small command line helper for manual UDP debugging."""

from __future__ import annotations

import argparse
import json
import time

from .client import XSensUDPClient
from .model import JOINT_NAMES
from .protocol import JointAnglesDatagram, QuaternionDatagram


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Listen to Xsens UDP packets and print parsed summaries.")
    parser.add_argument("--host", default="0.0.0.0", help="Local interface to bind. Default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=8001, help="UDP port to bind. Default: 8001")
    parser.add_argument(
        "--summary-every",
        type=int,
        default=50,
        help="Print one summary every N parsed packets after bootstrap. Default: 50",
    )
    parser.add_argument(
        "--dump-json",
        action="store_true",
        help="Print the full internal state as JSON instead of a short summary.",
    )
    parser.add_argument(
        "--print-joints",
        action="store_true",
        help="Skip full model bootstrap and print joint angle packets as soon as they arrive.",
    )
    parser.add_argument(
        "--bootstrap-debug",
        action="store_true",
        help="Print each packet type received while waiting for bootstrap packets.",
    )
    return parser


def print_joint_angles(datagram: JointAnglesDatagram) -> None:
    """Pretty-print one joint angle datagram."""

    output = {}
    for index, item in enumerate(datagram.items):
        joint_name = JOINT_NAMES[index] if index < len(JOINT_NAMES) else f"joint_{index}"
        output[joint_name] = {
            "parent_segment_id": item.parent_segment_id,
            "child_segment_id": item.child_segment_id,
            "rotation_xyz_deg": item.rotation,
        }
    print(json.dumps(output, indent=2, sort_keys=True))


def bootstrap_with_debug(client: XSensUDPClient) -> None:
    """Bootstrap while printing the packet types we actually receive."""

    quaternion_datagram = None
    joint_angles_datagram = None

    while quaternion_datagram is None or joint_angles_datagram is None:
        datagram = client.read_data()
        print(f"bootstrap received {datagram.header.message_type_text}")

        if quaternion_datagram is None and isinstance(datagram, QuaternionDatagram):
            quaternion_datagram = datagram
        if joint_angles_datagram is None and isinstance(datagram, JointAnglesDatagram):
            joint_angles_datagram = datagram

    client.bootstrap_from_datagrams(quaternion_datagram, joint_angles_datagram)


def main() -> None:
    args = build_arg_parser().parse_args()

    client = XSensUDPClient(udp_port=args.port, bind_host=args.host)
    client.bind()

    print(f"Listening for Xsens UDP packets on {args.host}:{args.port}")
    if args.print_joints:
        print("Printing joint angle packets directly. Full model bootstrap is skipped.")
        try:
            while True:
                datagram = client.read_data()
                print(f"received {datagram.header.message_type_text}")
                if isinstance(datagram, JointAnglesDatagram):
                    print_joint_angles(datagram)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        return

    print("Waiting for bootstrap packets (quaternion + joint angles)...")

    if args.bootstrap_debug:
        bootstrap_with_debug(client)
    else:
        client.build_model()

    print(
        f"Model ready: {len(client.link_name_list)} links, {len(client.joint_name_list)} joints. "
        "Streaming summaries..."
    )

    packet_count = 0
    start_time = time.time()

    try:
        while True:
            datagram = client.process_next_packet()
            packet_count += 1

            if packet_count % args.summary_every != 0:
                continue

            elapsed = time.time() - start_time
            rate = packet_count / elapsed if elapsed > 0 else 0.0

            if args.dump_json:
                print(json.dumps(client.snapshot(), indent=2, sort_keys=True))
                continue

            pelvis = client.human_data.get_link("pelvis")
            com = client.human_data.get_com()
            pelvis_pos = pelvis.state.position if pelvis is not None else None
            print(
                f"packets={packet_count} rate={rate:.1f}Hz "
                f"last_type={datagram.header.message_type_text} pelvis={pelvis_pos} com={com}"
            )
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
