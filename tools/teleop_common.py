"""
Shared helpers for xlerobot-webxr tools that talk directly to indory_isaac_sim.

This module is intentionally small and wire-format oriented. Browser/UI tools
can decide how to produce targets, but the ZMQ topics, tf.links parsing, schema
payload shape, and broad client-side workspace clamp should stay centralized.
"""

from __future__ import annotations

import math
import time
from typing import Any, Iterable, Mapping

import msgpack
import zmq


SCHEMA_VERSION_V11 = "xlerobot_v1.1"
SCHEMA_VERSION = "xlerobot_v1"
ARM_SIDES = ("right", "left")
TF_TARGET_NAMES = {"right": "gripper_right", "left": "gripper_left"}
DEFAULT_SIM_HOST = "100.80.87.68"
COMMAND_SOURCE_ROLES = ("teleop", "policy", "safety", "script")
COMMAND_PRIORITY_RANGE = (0, 100)
COMMAND_LEASE_MS_RANGE = (1, 60_000)
COMMAND_SOURCE_ID_MAX_LEN = 128

EE_REACH_RADIUS_M = 0.56
ARM_MOUNT_OFFSET = {
    "right": (-0.135, -0.133, 0.760),
    "left": (-0.135, +0.133, 0.760),
}


def pose_from_entry(entry: Mapping[str, Any]) -> list[float] | None:
    pose = entry.get("pose")
    if not isinstance(pose, (list, tuple)) or len(pose) != 7:
        return None
    try:
        out = [float(v) for v in pose]
    except (TypeError, ValueError):
        return None
    return out if all(math.isfinite(v) for v in out) else None


def unpack_payload(payload: bytes) -> dict[str, Any]:
    msg = msgpack.unpackb(payload, raw=False)
    return msg if isinstance(msg, dict) else {}


def extract_tf_poses(msg: Mapping[str, Any]) -> dict[str, list[float]]:
    updates: dict[str, list[float]] = {}
    for entry in msg.get("targets", []) or []:
        if not isinstance(entry, Mapping):
            continue
        name = entry.get("name")
        pose = pose_from_entry(entry)
        if pose is None:
            continue
        for side, target_name in TF_TARGET_NAMES.items():
            if name == target_name:
                updates[side] = pose
    return updates


def norm3(
    a: list[float] | tuple[float, ...],
    b: list[float] | tuple[float, ...],
) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def round_list(values: list[float] | tuple[float, ...], digits: int = 6) -> list[float]:
    return [round(float(v), digits) for v in values]


def summarize_vr_decode_debug(
    debug: Mapping[str, Any] | None,
    side: str,
) -> dict[str, Any] | None:
    if not isinstance(debug, Mapping):
        return None
    arms = debug.get("arms")
    if not isinstance(arms, Mapping):
        return None
    entry = arms.get(side)
    if not isinstance(entry, Mapping):
        return None
    out: dict[str, Any] = {
        "changed": bool(entry.get("changed_by_decoder", False)),
        "coarse_clamped": bool(entry.get("coarse_clamped", False)),
        "joint_projected": bool(entry.get("joint_limited_projected", False)),
        "solver": entry.get("solver"),
    }
    residual = entry.get("residual_m")
    if isinstance(residual, (int, float)) and math.isfinite(float(residual)):
        out["residual_m"] = round(float(residual), 6)
    projection_mode = entry.get("projection_mode")
    if isinstance(projection_mode, str):
        out["projection_mode"] = projection_mode
    for key in ("requested_pose_base", "target_pose_base"):
        pose = entry.get(key)
        if isinstance(pose, (list, tuple)) and len(pose) >= 3:
            out[key.replace("_pose_base", "_xyz")] = round_list(pose[:3])
    return out


def cmd_echo_latency_ms(
    msg: Mapping[str, Any],
    *,
    now_ns: int | None = None,
    min_stamp_ns: int = 0,
) -> float | None:
    echo = msg.get("cmd_echo_stamp_ns")
    if not isinstance(echo, int) or echo <= 0:
        return None
    if echo < int(min_stamp_ns):
        return None
    now = time.monotonic_ns() if now_ns is None else int(now_ns)
    latency_ms = (now - int(echo)) / 1e6
    if latency_ms < 0.0 or not math.isfinite(latency_ms):
        return None
    return latency_ms


def clamp_pose_to_workspace(
    side: str,
    pose: list[float],
    *,
    reach_radius_m: float = EE_REACH_RADIUS_M,
) -> list[float]:
    mount = ARM_MOUNT_OFFSET[side]
    delta = [float(pose[i]) - mount[i] for i in range(3)]
    radius = math.sqrt(sum(v * v for v in delta))
    if radius <= reach_radius_m or radius <= 1e-9:
        return pose
    scale = reach_radius_m / radius
    clamped = list(pose)
    for idx in range(3):
        clamped[idx] = mount[idx] + delta[idx] * scale
    return clamped


def default_relative_targets(sides: Iterable[str]) -> dict[str, dict[str, float]]:
    return {side: {"shoulder_pan": 0.0, "gripper": 0.0} for side in sides}


def build_command_payload(
    robot_id: int,
    targets_by_side: Mapping[str, list[float]] | None = None,
    relative_by_side: Mapping[str, Mapping[str, float]] | None = None,
    *,
    stamp_ns: int | None = None,
    source_id: str | None = None,
    source_role: str | None = None,
    priority: int | None = None,
    lease_ms: int | None = None,
) -> dict[str, Any]:
    targets = dict(targets_by_side or {})
    if relative_by_side is None:
        rel = default_relative_targets(targets.keys())
    else:
        rel = {
            side: {
                "shoulder_pan": float(values.get("shoulder_pan", 0.0)),
                "gripper": float(values.get("gripper", 0.0)),
            }
            for side, values in relative_by_side.items()
        }

    payload: dict[str, Any] = {
        "schema": SCHEMA_VERSION_V11,
        "stamp_ns": time.monotonic_ns() if stamp_ns is None else int(stamp_ns),
        "robot_id": int(robot_id),
        "frame": "body",
        "base_cmd_vel": [0.0, 0.0, 0.0],
        "arm_joint_relative_target": rel,
        "head_joint_relative_target": {"head_pan": 0.0, "head_tilt": 0.0},
    }
    if targets:
        payload["arm_ee_pose_target"] = {
            side: {"pose": list(pose), "mode": "absolute", "frame": "base"}
            for side, pose in targets.items()
        }
    add_command_metadata(
        payload,
        source_id=source_id,
        source_role=source_role,
        priority=priority,
        lease_ms=lease_ms,
    )
    return payload


def add_command_metadata(
    payload: dict[str, Any],
    *,
    source_id: str | None,
    source_role: str | None,
    priority: int | None,
    lease_ms: int | None,
) -> None:
    if source_id is None:
        if source_role is not None or priority is not None or lease_ms is not None:
            raise ValueError("command source metadata requires source_id")
        return
    if not isinstance(source_id, str) or not source_id:
        raise ValueError("source_id must be a non-empty string")
    if len(source_id) > COMMAND_SOURCE_ID_MAX_LEN:
        raise ValueError(f"source_id must be <= {COMMAND_SOURCE_ID_MAX_LEN} characters")
    payload["source_id"] = source_id

    if source_role is not None:
        if source_role not in COMMAND_SOURCE_ROLES:
            raise ValueError(
                f"source_role must be one of {COMMAND_SOURCE_ROLES}, got {source_role!r}"
            )
        payload["source_role"] = source_role
    if priority is not None:
        if not isinstance(priority, int) or isinstance(priority, bool):
            raise ValueError(f"priority must be int, got {type(priority).__name__}")
        lo, hi = COMMAND_PRIORITY_RANGE
        if not (lo <= priority <= hi):
            raise ValueError(f"priority must be in [{lo}, {hi}], got {priority}")
        payload["priority"] = priority
    if lease_ms is not None:
        if not isinstance(lease_ms, int) or isinstance(lease_ms, bool):
            raise ValueError(f"lease_ms must be int, got {type(lease_ms).__name__}")
        lo, hi = COMMAND_LEASE_MS_RANGE
        if not (lo <= lease_ms <= hi):
            raise ValueError(f"lease_ms must be in [{lo}, {hi}], got {lease_ms}")
        payload["lease_ms"] = lease_ms


def pack_command(payload: Mapping[str, Any]) -> bytes:
    return msgpack.packb(payload, use_bin_type=True)


def rpc_request(
    host: str,
    port: int,
    op: str,
    *,
    timeout_ms: int = 500,
    **kwargs: Any,
) -> dict[str, Any]:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
    sock.setsockopt(zmq.SNDTIMEO, int(timeout_ms))
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{host}:{port}")
    try:
        sock.send(msgpack.packb({"schema": SCHEMA_VERSION, "op": op, **kwargs}, use_bin_type=True))
        reply = msgpack.unpackb(sock.recv(), raw=False)
        return reply if isinstance(reply, dict) else {}
    finally:
        sock.close(linger=0)


def connect_tf_sub(
    ctx: zmq.Context,
    host: str,
    port: int,
    robot_id: int,
    *,
    timeout_ms: int | None = 20,
    rcvhwm: int = 8,
) -> zmq.Socket:
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, f"tf.links.{robot_id}".encode())
    if timeout_ms is not None:
        sub.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
    sub.setsockopt(zmq.RCVHWM, int(rcvhwm))
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(f"tcp://{host}:{port}")
    return sub


def connect_push(
    ctx: zmq.Context,
    host: str,
    port: int,
    *,
    sndhwm: int = 8,
) -> zmq.Socket:
    push = ctx.socket(zmq.PUSH)
    push.setsockopt(zmq.SNDHWM, int(sndhwm))
    push.setsockopt(zmq.LINGER, 0)
    push.connect(f"tcp://{host}:{port}")
    return push


def collect_poses(
    sub: zmq.Socket,
    robot_id: int,
    timeout_s: float,
    sample_side: str = "right",
) -> tuple[dict[str, list[float]], list[list[float]]]:
    latest: dict[str, list[float]] = {}
    samples: list[list[float]] = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            _topic, payload = sub.recv_multipart()
        except zmq.Again:
            continue
        try:
            msg = unpack_payload(payload)
        except Exception:
            continue
        if int(msg.get("robot_id", robot_id)) != robot_id:
            continue
        updates = extract_tf_poses(msg)
        for side, pose in updates.items():
            latest[side] = pose
            if side == sample_side:
                samples.append(pose)
    return latest, samples


def send_targets(
    push: zmq.Socket,
    robot_id: int,
    targets_by_side: Mapping[str, list[float]],
    relative_by_side: Mapping[str, Mapping[str, float]] | None = None,
) -> None:
    payload = build_command_payload(robot_id, targets_by_side, relative_by_side)
    push.send(pack_command(payload))


class TfReader:
    def __init__(self, host: str, port: int, robot_id: int, side: str) -> None:
        self.robot_id = int(robot_id)
        self.side = side
        self.last_decode_debug: dict[str, Any] | None = None
        self.first_cmd_echo_latency_ms: float | None = None
        self.last_cmd_echo_latency_ms: float | None = None
        self.first_cmd_echo_stamp_ns: int | None = None
        if side not in TF_TARGET_NAMES:
            raise ValueError(f"unknown arm side: {side}")
        self.ctx = zmq.Context.instance()
        self.sub = connect_tf_sub(self.ctx, host, port, robot_id)

    def close(self) -> None:
        self.sub.close(0)

    def next_pose(self, timeout_s: float) -> list[float] | None:
        deadline = time.monotonic() + max(0.0, timeout_s)
        latest: list[float] | None = None
        while time.monotonic() < deadline:
            try:
                _topic, payload = self.sub.recv_multipart()
            except zmq.Again:
                continue
            try:
                msg = unpack_payload(payload)
            except Exception:
                continue
            if int(msg.get("robot_id", self.robot_id)) != self.robot_id:
                continue
            debug = msg.get("vr_decode_debug")
            if isinstance(debug, dict):
                self.last_decode_debug = debug
            self._record_cmd_echo(msg)
            pose = extract_tf_poses(msg).get(self.side)
            if pose is not None:
                latest = pose
        return latest

    def wait_for_move(
        self,
        start_pose: list[float],
        *,
        timeout_s: float,
        move_threshold_m: float,
        min_cmd_echo_stamp_ns: int = 0,
    ) -> tuple[float | None, float, list[float] | None, int]:
        deadline = time.monotonic() + timeout_s
        first_move_s: float | None = None
        max_move = 0.0
        last_pose: list[float] | None = None
        samples = 0
        t0 = time.monotonic()
        self.first_cmd_echo_latency_ms = None
        self.last_cmd_echo_latency_ms = None
        self.first_cmd_echo_stamp_ns = None
        while time.monotonic() < deadline:
            try:
                _topic, payload = self.sub.recv_multipart()
            except zmq.Again:
                continue
            try:
                msg = unpack_payload(payload)
            except Exception:
                continue
            if int(msg.get("robot_id", self.robot_id)) != self.robot_id:
                continue
            debug = msg.get("vr_decode_debug")
            if isinstance(debug, dict):
                self.last_decode_debug = debug
            self._record_cmd_echo(msg, min_stamp_ns=min_cmd_echo_stamp_ns)
            pose = extract_tf_poses(msg).get(self.side)
            if pose is None:
                continue
            samples += 1
            last_pose = pose
            move = norm3(pose, start_pose)
            max_move = max(max_move, move)
            if first_move_s is None and move >= move_threshold_m:
                first_move_s = time.monotonic() - t0
        return first_move_s, max_move, last_pose, samples

    def decode_debug_summary(self) -> dict[str, Any] | None:
        return summarize_vr_decode_debug(self.last_decode_debug, self.side)

    def _record_cmd_echo(
        self,
        msg: Mapping[str, Any],
        *,
        min_stamp_ns: int = 0,
    ) -> None:
        latency_ms = cmd_echo_latency_ms(msg, min_stamp_ns=min_stamp_ns)
        if latency_ms is None:
            return
        echo = msg.get("cmd_echo_stamp_ns")
        if self.first_cmd_echo_latency_ms is None:
            self.first_cmd_echo_latency_ms = latency_ms
            self.first_cmd_echo_stamp_ns = int(echo)
        self.last_cmd_echo_latency_ms = latency_ms
