"""
Measure keyboard_teleop WebSocket command latency against indory_isaac_sim.

The probe assumes ``tools/keyboard_teleop.py`` is already running. It sends one
WebSocket ``command`` message, matching the browser physical-key path, then
watches ``tf.links`` until the selected EE moves by a threshold.

With ``--hold-s > 0`` it keeps sending browser-like held-key nudge frames at
``--hold-hz`` using ``--speed`` m/s, which measures continuous key hold delta
instead of only the first key edge.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time
import urllib.request
from typing import Any

from aiohttp import ClientSession

from teleop_common import DEFAULT_SIM_HOST, TF_TARGET_NAMES, TfReader, norm3, round_list


def _http_json(url: str, *, timeout_s: float = 2.0) -> dict[str, Any]:
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _nudge_for_code(code: str, step: float) -> list[float]:
    if code == "KeyW":
        return [0.0, 0.0, +step]
    if code == "KeyS":
        return [0.0, 0.0, -step]
    if code == "KeyR":
        return [+step, 0.0, 0.0]
    if code == "KeyF":
        return [-step, 0.0, 0.0]
    raise ValueError(f"unsupported EE nudge code {code!r}")


def _nudge_distance(nudge: list[float]) -> float:
    return sum(abs(float(v)) for v in nudge[:3])


def _command_payload(code: str, nudge_m: float) -> dict[str, Any]:
    return {
        "type": "command",
        "mode": "jog",
        "active": True,
        "estop": False,
        "nudge": _nudge_for_code(code, nudge_m),
        "pan_delta": 0.0,
        "gripper_delta": 0.0,
        "reanchor": False,
    }


def _xyz_from_debug(debug: dict[str, Any] | None, key: str) -> list[float] | None:
    if not isinstance(debug, dict):
        return None
    value = debug.get(key)
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError):
        return None


def _safe_ratio(num: float | None, den: float | None) -> float | None:
    if num is None or den is None or den <= 1e-9:
        return None
    return num / den


async def _send_ws_command(
    base: str,
    robot_id: int,
    code: str,
    step: float,
    *,
    warm_s: float,
    hold_s: float,
    hold_hz: float,
    speed_mps: float,
) -> tuple[float, int, int, float]:
    ws_base = base.replace("http://", "ws://").replace("https://", "wss://")
    url = f"{ws_base}/ws?robot={robot_id}"
    payload = _command_payload(code, step)
    t0 = time.monotonic()
    sent_motion_commands = 0
    total_nudge_m = _nudge_distance(payload["nudge"])
    async with ClientSession() as session:
        async with session.ws_connect(url, heartbeat=20) as ws:
            await ws.send_json({"select_robot": int(robot_id)})
            if warm_s > 0.0:
                await asyncio.sleep(warm_s)
            t0_ns = time.monotonic_ns()
            t0 = time.monotonic()
            await ws.send_json(payload)
            sent_motion_commands += 1
            ws_send_rtt_ms = (time.monotonic() - t0) * 1000.0
            if hold_s > 0.0:
                period_s = 1.0 / max(1.0, float(hold_hz))
                deadline = time.monotonic() + float(hold_s)
                last = time.monotonic()
                while True:
                    sleep_s = min(period_s, max(0.0, deadline - time.monotonic()))
                    if sleep_s > 0.0:
                        await asyncio.sleep(sleep_s)
                    now = time.monotonic()
                    dt = max(0.0, now - last)
                    last = now
                    if dt > 0.0:
                        held_step = max(0.0, float(speed_mps)) * dt
                        if held_step > 0.0:
                            held = _command_payload(code, held_step)
                            await ws.send_json(held)
                            sent_motion_commands += 1
                            total_nudge_m += _nudge_distance(held["nudge"])
                    if now >= deadline:
                        break
            await ws.send_json(
                {
                    "type": "command",
                    "mode": "jog",
                    "active": True,
                    "estop": False,
                    "nudge": [0.0, 0.0, 0.0],
                    "pan_delta": 0.0,
                    "gripper_delta": 0.0,
                    "reanchor": False,
                }
            )
    return ws_send_rtt_ms, t0_ns, sent_motion_commands, total_nudge_m


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send one keyboard_teleop WebSocket command and measure tf.links movement.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--keyboard-url", default="http://127.0.0.1:8765")
    parser.add_argument("--sim-host", default=DEFAULT_SIM_HOST)
    parser.add_argument("--sim-pub-port", type=int, default=5555)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--side", choices=tuple(TF_TARGET_NAMES), default="right")
    parser.add_argument("--code", default="KeyR")
    parser.add_argument("--step", type=float, default=0.035)
    parser.add_argument("--move-threshold", type=float, default=0.002)
    parser.add_argument("--observe-s", type=float, default=2.0)
    parser.add_argument("--anchor-timeout", type=float, default=3.0)
    parser.add_argument(
        "--hold-s",
        type=float,
        default=0.0,
        help="Seconds to continue sending browser-like held-key nudge frames.",
    )
    parser.add_argument("--hold-hz", type=float, default=90.0)
    parser.add_argument("--speed", type=float, default=0.8, help="Held-key nudge speed in m/s.")
    parser.add_argument(
        "--warm-s",
        type=float,
        default=0.25,
        help="Seconds to keep the WebSocket open before timing the command send.",
    )
    args = parser.parse_args()

    base = args.keyboard_url.rstrip("/")
    health = _http_json(f"{base}/health")
    if not health.get("anchor_ready"):
        print("keyboard teleop anchor not ready", json.dumps(health, ensure_ascii=False))
        return 2
    send_drops_before = int(health.get("send_drops") or 0)

    reader = TfReader(args.sim_host, args.sim_pub_port, args.robot_id, args.side)
    try:
        start_pose = reader.next_pose(args.anchor_timeout)
        if start_pose is None:
            print(f"missing tf.links pose for {TF_TARGET_NAMES[args.side]}")
            return 2
        print("start", round_list(start_pose[:3]))

        ws_rtt_ms, send_start_ns, sent_motion_commands, requested_nudge_m = asyncio.run(
            _send_ws_command(
                base,
                args.robot_id,
                args.code,
                args.step,
                warm_s=args.warm_s,
                hold_s=max(0.0, args.hold_s),
                hold_hz=args.hold_hz,
                speed_mps=args.speed,
            )
        )
        print(
            "ws_send",
            json.dumps(
                {
                    "code": args.code,
                    "step": args.step,
                    "hold_s": args.hold_s,
                    "speed": args.speed,
                    "sent_motion_commands": sent_motion_commands,
                    "requested_nudge_m": round(requested_nudge_m, 6),
                }
            ),
            f"rtt_ms={ws_rtt_ms:.2f}",
        )

        is_hold_probe = args.hold_s > 0.0
        first_move_s, max_move, last_pose, samples = reader.wait_for_move(
            start_pose,
            timeout_s=args.observe_s,
            move_threshold_m=args.move_threshold,
            min_cmd_echo_stamp_ns=send_start_ns,
        )
        first_move_report_s = None if is_hold_probe else first_move_s
        print("last", None if last_pose is None else round_list(last_pose[:3]))
        decode_debug = reader.decode_debug_summary()
        if decode_debug is not None:
            print("decode_debug", json.dumps(decode_debug, ensure_ascii=False))
        if not is_hold_probe and reader.first_cmd_echo_latency_ms is not None:
            print(
                "cmd_echo",
                json.dumps(
                    {
                        "first_ms": round(reader.first_cmd_echo_latency_ms, 2),
                        "last_ms": None
                        if reader.last_cmd_echo_latency_ms is None
                        else round(reader.last_cmd_echo_latency_ms, 2),
                        "stamp_ns": reader.first_cmd_echo_stamp_ns,
                    },
                    ensure_ascii=False,
                ),
            )
        health_after = _http_json(f"{base}/health")
        send_drops_after = int(health_after.get("send_drops") or 0)
        send_drop_delta = max(0, send_drops_after - send_drops_before)
        requested_xyz = _xyz_from_debug(decode_debug, "requested_xyz")
        target_xyz = _xyz_from_debug(decode_debug, "target_xyz")
        requested_move_m = (
            None if requested_xyz is None else norm3(requested_xyz, start_pose)
        )
        target_move_m = None if target_xyz is None else norm3(target_xyz, start_pose)
        target_error_m = (
            None
            if target_xyz is None or last_pose is None
            else norm3(last_pose, target_xyz)
        )
        requested_error_m = (
            None
            if requested_xyz is None or last_pose is None
            else norm3(last_pose, requested_xyz)
        )
        target_ratio = _safe_ratio(max_move, target_move_m)
        requested_ratio = _safe_ratio(max_move, requested_move_m)
        print(
            "summary",
            json.dumps(
                {
                    "first_move_ms": None
                    if first_move_report_s is None
                    else round(first_move_report_s * 1000.0, 2),
                    "max_move_m": round(max_move, 6),
                    "samples": samples,
                    "ws_rtt_ms": round(ws_rtt_ms, 2),
                    "sent_motion_commands": sent_motion_commands,
                    "requested_nudge_m": round(requested_nudge_m, 6),
                    "requested_move_m": None
                    if requested_move_m is None
                    else round(requested_move_m, 6),
                    "projected_target_move_m": None
                    if target_move_m is None
                    else round(target_move_m, 6),
                    "target_error_m": None
                    if target_error_m is None
                    else round(target_error_m, 6),
                    "requested_error_m": None
                    if requested_error_m is None
                    else round(requested_error_m, 6),
                    "move_to_projected_ratio": None
                    if target_ratio is None
                    else round(target_ratio, 4),
                    "move_to_requested_ratio": None
                    if requested_ratio is None
                    else round(requested_ratio, 4),
                    "send_drop_delta": send_drop_delta,
                    "first_cmd_echo_ms": None
                    if is_hold_probe or reader.first_cmd_echo_latency_ms is None
                    else round(reader.first_cmd_echo_latency_ms, 2),
                    "latency_fields_disabled": bool(is_hold_probe),
                    "decode_debug": decode_debug,
                },
                ensure_ascii=False,
            ),
        )
        if is_hold_probe:
            return 0 if max_move >= args.move_threshold else 1
        if first_move_s is None:
            return 1
        return 0
    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
