"""
Direct one-shot EE nudge for indory_isaac_sim.

This bypasses WebXR, WebRTC, browser input, and the VR bridge. It reads the
current gripper poses from sim tf.links.<robot_id>, seeds both arm EE target
slots, nudges one selected arm, then reports the observed tf.links movement.

Typical run:

  python3 tools/sim_nudge.py --sim-host 100.80.87.68 --robot-id 0
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Any

import msgpack
import zmq


SCHEMA_VERSION_V11 = "xlerobot_v1.1"
ARM_SIDES = ("right", "left")
TF_TARGET_NAMES = {"right": "gripper_right", "left": "gripper_left"}
DEFAULT_SIM_HOST = "100.80.87.68"


def _pose_from_entry(entry: dict[str, Any]) -> list[float] | None:
    pose = entry.get("pose")
    if not isinstance(pose, (list, tuple)) or len(pose) != 7:
        return None
    try:
        out = [float(v) for v in pose]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in out):
        return None
    return out


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
            msg = msgpack.unpackb(payload, raw=False)
        except Exception:
            continue
        if int(msg.get("robot_id", robot_id)) != robot_id:
            continue
        for entry in msg.get("targets", []) or []:
            name = entry.get("name")
            pose = _pose_from_entry(entry)
            if pose is None:
                continue
            for side, target_name in TF_TARGET_NAMES.items():
                if name == target_name:
                    latest[side] = pose
                    if side == sample_side:
                        samples.append(pose)
    return latest, samples


def send_targets(
    push: zmq.Socket,
    robot_id: int,
    targets_by_side: dict[str, list[float]],
) -> None:
    payload = {
        "schema": SCHEMA_VERSION_V11,
        "stamp_ns": time.monotonic_ns(),
        "robot_id": robot_id,
        "frame": "body",
        "base_cmd_vel": [0.0, 0.0, 0.0],
        "arm_ee_pose_target": {
            side: {"pose": pose, "mode": "absolute", "frame": "base"}
            for side, pose in targets_by_side.items()
        },
        "arm_joint_relative_target": {
            side: {"shoulder_pan": 0.0, "gripper": 0.0}
            for side in targets_by_side.keys()
        },
        "head_joint_relative_target": {"head_pan": 0.0, "head_tilt": 0.0},
    }
    push.send(msgpack.packb(payload, use_bin_type=True))


def _round3(pose: list[float]) -> list[float]:
    return [round(v, 6) for v in pose[:3]]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Directly nudge an indory_isaac_sim arm EE target over ZMQ.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--sim-host", default=DEFAULT_SIM_HOST)
    parser.add_argument("--sim-pub-port", type=int, default=5555)
    parser.add_argument("--sim-pull-port", type=int, default=5556)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--side", choices=ARM_SIDES, default="right")
    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=-0.025)
    parser.add_argument("--prime-frames", type=int, default=120)
    parser.add_argument("--frames", type=int, default=180)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--observe-s", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--min-move", type=float, default=0.002)
    args = parser.parse_args()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, f"tf.links.{args.robot_id}".encode())
    sub.setsockopt(zmq.RCVTIMEO, 250)
    sub.connect(f"tcp://{args.sim_host}:{args.sim_pub_port}")

    push = ctx.socket(zmq.PUSH)
    push.setsockopt(zmq.SNDHWM, 8)
    push.setsockopt(zmq.LINGER, 0)
    push.connect(f"tcp://{args.sim_host}:{args.sim_pull_port}")

    try:
        poses, _samples = collect_poses(
            sub, args.robot_id, args.timeout, sample_side=args.side
        )
        if args.side not in poses:
            print(f"missing tf.links pose for {TF_TARGET_NAMES[args.side]}")
            return 2

        before = list(poses[args.side])
        print("have sides", sorted(poses.keys()))
        print("before", _round3(before))

        period = 1.0 / max(args.hz, 1.0)
        for _ in range(max(0, args.prime_frames)):
            send_targets(push, args.robot_id, poses)
            time.sleep(period)

        poses, _samples = collect_poses(
            sub, args.robot_id, min(args.timeout, 1.0), sample_side=args.side
        )
        if args.side not in poses:
            poses[args.side] = before
        start = list(poses[args.side])
        targets = {side: list(pose) for side, pose in poses.items()}
        target = targets[args.side]
        target[0] += args.dx
        target[1] += args.dy
        target[2] += args.dz
        print("target", _round3(target))

        for _ in range(max(1, args.frames)):
            send_targets(push, args.robot_id, targets)
            time.sleep(period)

        _poses, observed = collect_poses(
            sub, args.robot_id, args.observe_s, sample_side=args.side
        )
        if not observed:
            print(f"no post-move {args.side}-arm samples")
            return 2

        after = observed[-1]
        max_move = max(
            math.sqrt(sum((pose[idx] - start[idx]) ** 2 for idx in range(3)))
            for pose in observed
        )
        print("after ", _round3(after))
        print("delta ", _round3([after[idx] - start[idx] for idx in range(7)]))
        print("max_move_m", round(max_move, 6), "samples", len(observed))
        if max_move < args.min_move:
            print(f"move below threshold: {max_move:.6f} < {args.min_move:.6f}")
            return 1
        return 0
    finally:
        sub.close(0)
        push.close(0)


if __name__ == "__main__":
    raise SystemExit(main())
