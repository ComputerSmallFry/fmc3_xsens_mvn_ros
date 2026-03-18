"""Main control loop: Xsens UDP -> GR-2 upper-body commands."""

from __future__ import annotations

import argparse
import copy
import json
import os
import signal
import time
from typing import Dict, List

from xsens_mvn_ros_python.client import XSensUDPClient

from . import config
from .mapping import xsens_to_gr2
from .safety import (
    DataTimeoutMonitor,
    clamp_to_limits,
    lerp_cmd,
    low_pass_filter,
    rate_limit,
)


def drain_latest(client: XSensUDPClient) -> bool:
    """Read all pending UDP packets, keep only the latest state.

    Returns True if at least one packet was consumed.
    """
    sock = client.socket
    if sock is None:
        return False
    sock.setblocking(False)
    got_any = False
    try:
        while True:
            try:
                client.process_next_packet()
                got_any = True
            except (BlockingIOError, OSError):
                break
    finally:
        sock.setblocking(True)
    return got_any


def ramp_to_pose(
    aurora_client,
    start_cmd: Dict[str, List[float]],
    end_cmd: Dict[str, List[float]],
    duration: float,
    rate: int = config.CONTROL_RATE_HZ,
) -> None:
    """Linearly interpolate from start to end over *duration* seconds."""
    steps = max(int(duration * rate), 1)
    for i in range(steps):
        t = (i + 1) / steps
        cmd = lerp_cmd(start_cmd, end_cmd, t)
        aurora_client.set_group_cmd(position_cmd=cmd)
        time.sleep(1.0 / rate)


CALIBRATION_FILE = os.path.join(
    os.path.dirname(__file__), "tpose_calibration.json"
)


def save_calibration(offsets: Dict[str, List[float]], path: str = CALIBRATION_FILE) -> None:
    with open(path, "w") as f:
        json.dump(offsets, f, indent=2)
    print(f"[bridge] Calibration saved to {path}")


def load_calibration(path: str = CALIBRATION_FILE) -> Dict[str, List[float]] | None:
    if not os.path.exists(path):
        return None
    with open(path) as f:
        data = json.load(f)
    print(f"[bridge] Calibration loaded from {path}")
    return data


def calibrate_tpose(
    xsens: XSensUDPClient,
    duration: float = 3.0,
    save: bool = True,
) -> Dict[str, List[float]]:
    """Record T-pose baseline: average mapping output over *duration* seconds.

    The user should stand in T-pose (arms out, palms down) while this runs.
    The returned offsets are subtracted from live mapping output so that
    T-pose maps to GR-2 zero pose.
    """
    print(f"[bridge] Stand in T-pose for {duration:.0f} seconds ...")
    samples: List[Dict[str, List[float]]] = []
    t_end = time.monotonic() + duration
    while time.monotonic() < t_end:
        drain_latest(xsens)
        samples.append(xsens_to_gr2(xsens.human_data))
        time.sleep(0.02)

    if not samples:
        return config.ZERO_POSE

    # Average per group/joint.
    avg: Dict[str, List[float]] = {}
    for group in config.UPPER_BODY_GROUPS:
        n_joints = len(samples[0][group])
        avg[group] = [
            sum(s[group][j] for s in samples) / len(samples)
            for j in range(n_joints)
        ]
    print("[bridge] T-pose calibration recorded.")
    if save:
        save_calibration(avg)
    return avg


def apply_calibration(
    cmd: Dict[str, List[float]],
    offsets: Dict[str, List[float]],
) -> Dict[str, List[float]]:
    """Subtract T-pose offsets so that T-pose maps to zero."""
    return {
        group: [v - o for v, o in zip(vals, offsets.get(group, [0.0] * len(vals)))]
        for group, vals in cmd.items()
    }


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Xsens MVN -> GR-2 upper-body bridge")
    p.add_argument("--xsens-port", type=int, default=config.XSENS_UDP_PORT,
                    help=f"Xsens UDP port (default: {config.XSENS_UDP_PORT})")
    p.add_argument("--domain-id", type=int, default=config.GR2_DOMAIN_ID,
                    help=f"DDS domain ID (default: {config.GR2_DOMAIN_ID})")
    p.add_argument("--dry-run", action="store_true",
                    help="Print commands instead of sending to GR-2")
    p.add_argument("--no-calibrate", action="store_true",
                    help="Skip T-pose calibration entirely (use zero offsets)")
    p.add_argument("--recalibrate", action="store_true",
                    help="Force new T-pose calibration even if saved file exists")
    return p


def main() -> None:
    args = build_arg_parser().parse_args()

    # ---- Xsens init ----
    print(f"[bridge] Binding Xsens UDP on port {args.xsens_port} ...")
    xsens = XSensUDPClient(udp_port=args.xsens_port)
    xsens.bind()

    aurora_client = None
    try:
        print("[bridge] Waiting for first Xsens packets to build model ...")
        xsens.build_model()
        print(f"[bridge] Model ready: {len(xsens.joint_name_list)} joints, "
              f"{len(xsens.link_name_list)} links")

        # ---- Aurora / GR-2 init ----
        if not args.dry_run:
            from fourier_aurora_client import AuroraClient

            print(f"[bridge] Connecting to GR-2 (domain_id={args.domain_id}) ...")
            aurora_client = AuroraClient.get_instance(
                domain_id=args.domain_id, robot_name="gr2"
            )
            time.sleep(1.0)

            # FSM: Upper body user command (legs stay standing)
            print("[bridge] Setting FSM -> UpperBodyUserCommand(11) ...")
            aurora_client.set_fsm_state(11)
            time.sleep(1.0)

            print("[bridge] Setting upper FSM -> MoveCommand(4) ...")
            aurora_client.set_upper_fsm_state(4)
            time.sleep(0.5)

            # Configure PD gains
            print("[bridge] Configuring PD gains ...")
            aurora_client.set_motor_cfg_pd(kp_config=config.KP, kd_config=config.KD)
            time.sleep(0.5)

        # ---- State ----
        prev_cmd = copy.deepcopy(config.ZERO_POSE)
        filtered_cmd = copy.deepcopy(config.ZERO_POSE)
        timeout_monitor = DataTimeoutMonitor()
        dt = 1.0 / config.CONTROL_RATE_HZ
        running = True

        def _shutdown(signum, frame):
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, _shutdown)
        signal.signal(signal.SIGTERM, _shutdown)

        # ---- T-pose calibration ----
        if args.no_calibrate:
            tpose_offsets = copy.deepcopy(config.ZERO_POSE)
        elif args.recalibrate:
            tpose_offsets = calibrate_tpose(xsens)
        else:
            # Try loading saved calibration first.
            loaded = load_calibration()
            if loaded is not None:
                tpose_offsets = loaded
            else:
                tpose_offsets = calibrate_tpose(xsens)

        # ---- Startup ramp ----
        # Read a few frames to get a valid initial target.
        for _ in range(10):
            drain_latest(xsens)
            time.sleep(0.02)

        initial_target = apply_calibration(xsens_to_gr2(xsens.human_data), tpose_offsets)
        initial_target = clamp_to_limits(initial_target)
        print("[bridge] Ramping to initial pose ...")
        if aurora_client is not None:
            ramp_to_pose(aurora_client, config.ZERO_POSE, initial_target,
                         config.RAMP_DURATION_SEC)
        prev_cmd = copy.deepcopy(initial_target)
        filtered_cmd = copy.deepcopy(initial_target)

        # ---- Main loop ----
        print(f"[bridge] Running at {config.CONTROL_RATE_HZ} Hz. Ctrl+C to stop.")
        loop_count = 0
        while running:
            t_start = time.monotonic()

            # 1. Drain Xsens UDP
            got_data = drain_latest(xsens)
            if got_data:
                timeout_monitor.feed()

            # 2. Check timeout
            if timeout_monitor.check():
                # Hold last safe position
                if loop_count % 50 == 0:
                    print("[bridge] WARNING: Xsens data timeout, holding position")
                if aurora_client is not None:
                    aurora_client.set_group_cmd(position_cmd=prev_cmd)
                _sleep_remainder(t_start, dt)
                loop_count += 1
                continue

            # 3. Map
            raw_cmd = apply_calibration(xsens_to_gr2(xsens.human_data), tpose_offsets)

            # 4. Safety pipeline
            cmd = clamp_to_limits(raw_cmd)
            cmd = low_pass_filter(cmd, filtered_cmd)
            filtered_cmd = copy.deepcopy(cmd)
            cmd = rate_limit(cmd, prev_cmd, dt)
            prev_cmd = copy.deepcopy(cmd)

            # 5. Send
            if aurora_client is not None:
                aurora_client.set_group_cmd(position_cmd=cmd)
            elif loop_count % 25 == 0:
                # Dry-run: print every 0.5s
                _print_cmd(cmd)

            _sleep_remainder(t_start, dt)
            loop_count += 1

        # ---- Graceful shutdown ----
        print("\n[bridge] Shutting down, ramping to zero pose ...")
        if aurora_client is not None:
            ramp_to_pose(aurora_client, prev_cmd, config.ZERO_POSE,
                         config.SHUTDOWN_RAMP_SEC)
            aurora_client.close()

    except KeyboardInterrupt:
        print("\n[bridge] Interrupted.")
    finally:
        if xsens.socket is not None:
            xsens.socket.close()
            print("[bridge] UDP socket closed.")
    print("[bridge] Done.")


def _sleep_remainder(t_start: float, dt: float) -> None:
    elapsed = time.monotonic() - t_start
    remaining = dt - elapsed
    if remaining > 0:
        time.sleep(remaining)


def _print_cmd(cmd: Dict[str, List[float]]) -> None:
    parts = []
    for group in config.UPPER_BODY_GROUPS:
        vals = cmd.get(group, [])
        formatted = ", ".join(f"{v:+.3f}" for v in vals)
        parts.append(f"  {group}: [{formatted}]")
    print("[dry-run]\n" + "\n".join(parts))


if __name__ == "__main__":
    main()
