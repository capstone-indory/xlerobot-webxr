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
ARM_SIDES = ("right", "left")
TF_TARGET_NAMES = {"right": "gripper_right", "left": "gripper_left"}
DEFAULT_SIM_HOST = "100.80.87.68"

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
    return payload


def pack_command(payload: Mapping[str, Any]) -> bytes:
    return msgpack.packb(payload, use_bin_type=True)


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
    ) -> tuple[float | None, float, list[float] | None, int]:
        deadline = time.monotonic() + timeout_s
        first_move_s: float | None = None
        max_move = 0.0
        last_pose: list[float] | None = None
        samples = 0
        t0 = time.monotonic()
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
