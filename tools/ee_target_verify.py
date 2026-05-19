"""
Verify indory_isaac_sim EE target accuracy over a small waypoint scenario.

The scenario intentionally tests absolute EE targets, not just "did it move".
It reads the current gripper pose from tf.links.<robot_id>, treats that as an
anchor, sends anchor-relative target offsets, and records how closely the
measured tf.links pose converges to each target.

Typical run:

  python3 tools/ee_target_verify.py --sim-host 100.80.87.68 --robot-id 0
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import msgpack
import zmq


SCHEMA_VERSION_V11 = "xlerobot_v1.1"
ARM_SIDES = ("right", "left")
TF_TARGET_NAMES = {"right": "gripper_right", "left": "gripper_left"}
DEFAULT_SIM_HOST = "100.80.87.68"


@dataclass(frozen=True)
class Waypoint:
    name: str
    offset: tuple[float, float, float]


def pose_from_entry(entry: dict[str, Any]) -> list[float] | None:
    pose = entry.get("pose")
    if not isinstance(pose, (list, tuple)) or len(pose) != 7:
        return None
    try:
        out = [float(v) for v in pose]
    except (TypeError, ValueError):
        return None
    return out if all(math.isfinite(v) for v in out) else None


def norm3(a: list[float] | tuple[float, ...], b: list[float] | tuple[float, ...]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def round_list(values: list[float] | tuple[float, ...], digits: int = 6) -> list[float]:
    return [round(float(v), digits) for v in values]


def build_default_scenario(step_m: float, lateral_step_m: float) -> list[Waypoint]:
    s = float(step_m)
    y = float(lateral_step_m)
    return [
        Waypoint("center", (0.0, 0.0, 0.0)),
        Waypoint("x_plus", (+s, 0.0, 0.0)),
        Waypoint("center_after_x_plus", (0.0, 0.0, 0.0)),
        Waypoint("x_minus", (-s, 0.0, 0.0)),
        Waypoint("center_after_x_minus", (0.0, 0.0, 0.0)),
        Waypoint("z_plus", (0.0, 0.0, +s)),
        Waypoint("center_after_z_plus", (0.0, 0.0, 0.0)),
        Waypoint("z_minus", (0.0, 0.0, -s)),
        Waypoint("center_after_z_minus", (0.0, 0.0, 0.0)),
        Waypoint("y_plus", (0.0, +y, 0.0)),
        Waypoint("center_after_y_plus", (0.0, 0.0, 0.0)),
        Waypoint("y_minus", (0.0, -y, 0.0)),
        Waypoint("center_after_y_minus", (0.0, 0.0, 0.0)),
        Waypoint("x_plus_z_plus", (+s, 0.0, +s)),
        Waypoint("center_after_x_plus_z_plus", (0.0, 0.0, 0.0)),
        Waypoint("x_plus_z_minus", (+s, 0.0, -s)),
        Waypoint("center_after_x_plus_z_minus", (0.0, 0.0, 0.0)),
        Waypoint("x_minus_z_plus", (-s, 0.0, +s)),
        Waypoint("center_after_x_minus_z_plus", (0.0, 0.0, 0.0)),
        Waypoint("x_minus_z_minus", (-s, 0.0, -s)),
        Waypoint("return_center", (0.0, 0.0, 0.0)),
    ]


class SimEeVerifier:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ctx = zmq.Context.instance()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, f"tf.links.{args.robot_id}".encode())
        self.sub.setsockopt(zmq.RCVTIMEO, 20)
        self.sub.setsockopt(zmq.RCVHWM, 8)
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.connect(f"tcp://{args.sim_host}:{args.sim_pub_port}")

        self.push = self.ctx.socket(zmq.PUSH)
        self.push.setsockopt(zmq.SNDHWM, 8)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(f"tcp://{args.sim_host}:{args.sim_pull_port}")

        self.latest: dict[str, list[float]] = {}

    def close(self) -> None:
        self.sub.close(0)
        self.push.close(0)

    def drain_tf(self, timeout_s: float = 0.0) -> list[list[float]]:
        samples: list[list[float]] = []
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            if timeout_s > 0 and time.monotonic() >= deadline:
                return samples
            try:
                _topic, payload = self.sub.recv_multipart()
            except zmq.Again:
                if timeout_s <= 0:
                    return samples
                continue
            try:
                msg = msgpack.unpackb(payload, raw=False)
            except Exception:
                continue
            if int(msg.get("robot_id", self.args.robot_id)) != self.args.robot_id:
                continue
            for entry in msg.get("targets", []) or []:
                name = entry.get("name")
                pose = pose_from_entry(entry)
                if pose is None:
                    continue
                for side, target_name in TF_TARGET_NAMES.items():
                    if name == target_name:
                        self.latest[side] = pose
                        if side == self.args.side:
                            samples.append(pose)

    def wait_for_poses(self) -> dict[str, list[float]]:
        self.drain_tf(self.args.anchor_timeout)
        missing = [side for side in ARM_SIDES if side not in self.latest]
        if missing:
            raise RuntimeError(f"missing tf.links anchors: {missing}")
        return {side: list(pose) for side, pose in self.latest.items()}

    def send_targets(self, target_by_side: dict[str, list[float]]) -> None:
        payload = {
            "schema": SCHEMA_VERSION_V11,
            "stamp_ns": time.monotonic_ns(),
            "robot_id": self.args.robot_id,
            "frame": "body",
            "base_cmd_vel": [0.0, 0.0, 0.0],
            "arm_ee_pose_target": {
                side: {"pose": pose, "mode": "absolute", "frame": "base"}
                for side, pose in target_by_side.items()
            },
            "arm_joint_relative_target": {
                side: {"shoulder_pan": 0.0, "gripper": 0.0}
                for side in target_by_side.keys()
            },
            "head_joint_relative_target": {"head_pan": 0.0, "head_tilt": 0.0},
        }
        self.push.send(msgpack.packb(payload, use_bin_type=True))

    def compose_targets(self, side: str, target_xyz: tuple[float, float, float]) -> dict[str, list[float]]:
        targets: dict[str, list[float]] = {}
        for arm_side in ARM_SIDES:
            live = self.latest.get(arm_side)
            if live is None:
                continue
            pose = list(live)
            if arm_side == side:
                pose[0:3] = [float(v) for v in target_xyz]
            targets[arm_side] = pose
        return targets

    def prime(self, frames: int, hz: float) -> None:
        period = 1.0 / max(hz, 1.0)
        self.drain_tf(0.5)
        for _ in range(max(0, frames)):
            self.drain_tf(0.0)
            self.send_targets({side: list(pose) for side, pose in self.latest.items()})
            time.sleep(period)

    def run_waypoint(
        self,
        waypoint: Waypoint,
        anchor: list[float],
        duration_s: float,
        hz: float,
        tolerance_m: float,
    ) -> dict[str, Any]:
        target_xyz = tuple(anchor[i] + waypoint.offset[i] for i in range(3))
        period = 1.0 / max(hz, 1.0)
        t0 = time.monotonic()
        next_t = t0
        samples: list[tuple[float, list[float], float, float]] = []
        first_move_s: float | None = None
        first_within_s: float | None = None
        start_pose = list(self.latest[self.args.side])
        while time.monotonic() - t0 < duration_s:
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.005, next_t - now))
                continue
            next_t += period
            tf_samples = self.drain_tf(0.0)
            for pose in tf_samples:
                t_rel = time.monotonic() - t0
                target_error = norm3(pose, target_xyz)
                move = norm3(pose, start_pose)
                samples.append((t_rel, list(pose), target_error, move))
                if first_move_s is None and move >= self.args.move_threshold:
                    first_move_s = t_rel
                if first_within_s is None and target_error <= tolerance_m:
                    first_within_s = t_rel
            self.send_targets(self.compose_targets(self.args.side, target_xyz))

        tf_samples = self.drain_tf(self.args.observe_s)
        for pose in tf_samples:
            t_rel = time.monotonic() - t0
            target_error = norm3(pose, target_xyz)
            move = norm3(pose, start_pose)
            samples.append((t_rel, list(pose), target_error, move))
            if first_move_s is None and move >= self.args.move_threshold:
                first_move_s = t_rel
            if first_within_s is None and target_error <= tolerance_m:
                first_within_s = t_rel

        final_pose = samples[-1][1] if samples else list(self.latest[self.args.side])
        final_error = norm3(final_pose, target_xyz)
        min_error = min((s[2] for s in samples), default=final_error)
        max_move = max((s[3] for s in samples), default=0.0)
        status = "pass" if final_error <= tolerance_m else "fail"
        if status == "fail" and min_error <= tolerance_m:
            status = "drift"
        return {
            "name": waypoint.name,
            "status": status,
            "offset_xyz": round_list(waypoint.offset),
            "target_xyz": round_list(target_xyz),
            "start_xyz": round_list(start_pose[:3]),
            "final_xyz": round_list(final_pose[:3]),
            "final_error_m": round(final_error, 6),
            "min_error_m": round(min_error, 6),
            "max_move_m": round(max_move, 6),
            "first_move_s": None if first_move_s is None else round(first_move_s, 3),
            "first_within_s": None if first_within_s is None else round(first_within_s, 3),
            "sample_count": len(samples),
        }


def write_reports(records: list[dict[str, Any]], out_prefix: Path) -> tuple[Path, Path]:
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    jsonl_path = out_prefix.with_suffix(".jsonl")
    csv_path = out_prefix.with_suffix(".csv")
    with jsonl_path.open("w", encoding="utf-8") as fh:
        for record in records:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")
    fields = [
        "name",
        "status",
        "offset_xyz",
        "target_xyz",
        "start_xyz",
        "final_xyz",
        "final_error_m",
        "min_error_m",
        "max_move_m",
        "first_move_s",
        "first_within_s",
        "sample_count",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({field: record.get(field) for field in fields})
    return jsonl_path, csv_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an EE target accuracy verification scenario against indory_isaac_sim.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--sim-host", default=DEFAULT_SIM_HOST)
    parser.add_argument("--sim-pub-port", type=int, default=5555)
    parser.add_argument("--sim-pull-port", type=int, default=5556)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--side", choices=ARM_SIDES, default="right")
    parser.add_argument("--step", type=float, default=0.03)
    parser.add_argument("--lateral-step", type=float, default=0.02)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--settle-s", type=float, default=3.0)
    parser.add_argument("--observe-s", type=float, default=1.0)
    parser.add_argument("--tolerance", type=float, default=0.012)
    parser.add_argument("--move-threshold", type=float, default=0.002)
    parser.add_argument("--prime-frames", type=int, default=120)
    parser.add_argument("--anchor-timeout", type=float, default=3.0)
    parser.add_argument("--max-waypoints", type=int, default=0)
    parser.add_argument("--out", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    waypoints = build_default_scenario(args.step, args.lateral_step)
    if args.max_waypoints > 0:
        waypoints = waypoints[: args.max_waypoints]
    out_prefix = args.out
    if out_prefix is None:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        out_prefix = Path("tools") / "reports" / f"ee_target_verify_{stamp}"

    verifier = SimEeVerifier(args)
    records: list[dict[str, Any]] = []
    try:
        poses = verifier.wait_for_poses()
        anchor = list(poses[args.side])
        print("anchor", args.side, round_list(anchor[:3]))
        print("scenario_waypoints", len(waypoints), "tolerance_m", args.tolerance)
        verifier.prime(args.prime_frames, args.hz)
        for idx, waypoint in enumerate(waypoints, start=1):
            record = verifier.run_waypoint(
                waypoint,
                anchor,
                args.settle_s,
                args.hz,
                args.tolerance,
            )
            records.append(record)
            print(
                f"{idx:02d}/{len(waypoints)} {record['status']:>5} "
                f"{record['name']:<26} final_err={record['final_error_m']:.4f}m "
                f"min_err={record['min_error_m']:.4f}m first_move={record['first_move_s']}"
            )
    finally:
        verifier.close()

    jsonl_path, csv_path = write_reports(records, out_prefix)
    failed = [r for r in records if r["status"] != "pass"]
    print("report_jsonl", jsonl_path)
    print("report_csv", csv_path)
    print("summary", json.dumps({
        "total": len(records),
        "pass": len(records) - len(failed),
        "non_pass": len(failed),
        "worst": max(records, key=lambda r: r["final_error_m"]) if records else None,
    }, ensure_ascii=False))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
