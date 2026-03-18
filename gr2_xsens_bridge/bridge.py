"""Main control loop: Xsens UDP -> GR-2 upper-body commands."""

from __future__ import annotations

import argparse
import copy
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
    """Read all pending UDP packets, keep only the latest state."""
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

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Xsens MVN -> GR-2 upper-body bridge")
    p.add_argument("--xsens-port", type=int, default=config.XSENS_UDP_PORT,
                    help=f"Xsens UDP port (default: {config.XSENS_UDP_PORT})")
    p.add_argument("--domain-id", type=int, default=config.GR2_DOMAIN_ID,
                    help=f"DDS domain ID (default: {config.GR2_DOMAIN_ID})")
    p.add_argument("--dry-run", action="store_true",
                    help="Print commands instead of sending to GR-2")
    p.add_argument("--print-xsens", action="store_true",
                    help="Print raw Xsens joint angles and mapped commands while running")
    return p


def main() -> None:
    args = build_arg_parser().parse_args()

    # ---- Xsens init ----
    print(f"[bridge] Binding Xsens UDP on port {args.xsens_port} ...")
    xsens = XSensUDPClient(udp_port=args.xsens_port)
    xsens.bind()
    # 4MB receive buffer — survive slow set_group_cmd calls
    import socket as _sock
    xsens.socket.setsockopt(_sock.SOL_SOCKET, _sock.SO_RCVBUF, 4 * 1024 * 1024)

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
            if aurora_client is None:
                raise RuntimeError(
                    "AuroraClient initialization failed. Check simulator/AuroraCore status and domain ID."
                )
            time.sleep(1.0)

            # FSM: PdStand — control upper body under PD standing mode
            print("[bridge] Setting FSM -> PdStand(2) ...")
            aurora_client.set_fsm_state(2)
            time.sleep(1.0)

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

        # ---- Startup ramp ----
        for _ in range(10):
            drain_latest(xsens)
            time.sleep(0.02)
        initial_target = xsens_to_gr2(xsens.human_data)
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
        cmd_time_max = 0.0
        while running:
            t_start = time.monotonic()

            # 1. Drain every pending Xsens packet and keep only the newest state.
            got_data = drain_latest(xsens)
            t_drain = time.monotonic() - t_start
            if got_data:
                timeout_monitor.feed()

            # 2. Timeout check
            if timeout_monitor.check():
                data_age = time.monotonic() - timeout_monitor.last_data_time
                if loop_count % 50 == 0:
                    print(f"[bridge] WARNING: timeout ({data_age:.2f}s) drain={t_drain*1000:.1f}ms cmd_max={cmd_time_max*1000:.1f}ms")
                if aurora_client is not None:
                    aurora_client.set_group_cmd(position_cmd=prev_cmd)
                _sleep_remainder(t_start, dt)
                loop_count += 1
                continue

            # 3. Map
            raw_cmd = xsens_to_gr2(xsens.human_data)

            # 4. Safety pipeline
            limited_cmd = clamp_to_limits(raw_cmd)
            cmd = limited_cmd
            cmd = low_pass_filter(cmd, filtered_cmd)
            filtered_cmd = copy.deepcopy(cmd)
            cmd = rate_limit(cmd, prev_cmd, dt)
            prev_cmd = copy.deepcopy(cmd)

            # 5. Send
            if aurora_client is not None:
                t_cmd = time.monotonic()
                aurora_client.set_group_cmd(position_cmd=cmd)
                cmd_dur = time.monotonic() - t_cmd
                if cmd_dur > cmd_time_max:
                    cmd_time_max = cmd_dur
            if args.print_xsens and loop_count % 25 == 0:
                _print_xsens_state(xsens.human_data, raw_cmd, limited_cmd, cmd)
            elif aurora_client is None and loop_count % 25 == 0:
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


def _print_xsens_state(
    human_data,
    raw_cmd: Dict[str, List[float]],
    limited_cmd: Dict[str, List[float]],
    cmd: Dict[str, List[float]],
) -> None:
    joint_names = [
        "c1_head",
        "left_shoulder",
        "left_elbow",
        "left_wrist",
        "right_shoulder",
        "right_elbow",
        "right_wrist",
    ]
    raw_parts = []
    for joint_name in joint_names:
        joint = human_data.get_joint(joint_name)
        if joint is None:
            continue
        angles = ", ".join(f"{v:+.1f}" for v in joint.state.angles)
        raw_parts.append(f"  {joint_name}: [{angles}] deg")

    raw_parts_cmd = []
    limited_parts = []
    final_parts = []
    for group in config.UPPER_BODY_GROUPS:
        raw_vals = raw_cmd.get(group, [])
        raw_formatted = ", ".join(f"{v:+.3f}" for v in raw_vals)
        raw_parts_cmd.append(f"  {group}: [{raw_formatted}]")

        limited_vals = limited_cmd.get(group, [])
        limited_formatted = ", ".join(f"{v:+.3f}" for v in limited_vals)
        clamp_flags = [
            "*" if abs(r - l) > 1e-6 else " "
            for r, l in zip(raw_vals, limited_vals)
        ]
        clamp_suffix = "".join(clamp_flags)
        limited_parts.append(f"  {group}: [{limited_formatted}]  clamp={clamp_suffix}")

        final_vals = cmd.get(group, [])
        final_formatted = ", ".join(f"{v:+.3f}" for v in final_vals)
        final_parts.append(f"  {group}: [{final_formatted}]")

    lines = ["[xsens]"]
    lines.extend(raw_parts or ["  <no joint data>"])
    lines.append("[mapped-raw]")
    lines.extend(raw_parts_cmd)
    lines.append("[mapped-limited]")
    lines.extend(limited_parts)
    lines.append("[cmd-sent]")
    lines.extend(final_parts)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
