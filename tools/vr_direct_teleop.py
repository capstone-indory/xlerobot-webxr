"""
Direct WebXR pose.0 -> indory_isaac_sim teleop bridge.

This is the local counterpart to tools/keyboard_teleop.py:

  WebXR page -> mac_proxy /ws -> ZMQ pose.<robot_id> -> this bridge -> sim :5556

The bridge reads current left/right gripper poses from tf.links.<robot_id>,
captures an anchor when the controller grip is held, retargets controller
position deltas into robot base-frame EE targets, and always sends both arm
EE target slots. Keeping the non-moving arm in the payload matches the sim
behavior verified by tools/sim_nudge.py.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import zmq

from teleop_common import (
    ARM_SIDES,
    build_command_payload,
    clamp_pose_to_workspace,
    extract_tf_poses,
    pack_command,
    unpack_payload,
)


log = logging.getLogger("vr_direct_teleop")

PAGE_SCHEMA = "xlerobot_v1.1.page"
BUTTON_ANCHOR = {"right": "a", "left": "x"}


def xr_delta_to_robot(delta: list[float], scale: float) -> list[float]:
    # WebXR: +X right, +Y up, -Z forward.
    # Robot base: +X forward, +Y left, +Z up.
    return [-delta[2] * scale, -delta[0] * scale, delta[1] * scale]


def pose7(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        return None
    try:
        out = [float(v) for v in value]
    except (TypeError, ValueError):
        return None
    return out if all(math.isfinite(v) for v in out) else None


def vec3_delta(now: list[float], anchor: list[float]) -> list[float]:
    return [now[i] - anchor[i] for i in range(3)]


def buttons(payload: dict[str, Any]) -> dict[str, Any]:
    raw = payload.get("buttons")
    return raw if isinstance(raw, dict) else {}


@dataclass
class Anchor:
    ctrl_pose: list[float]
    ee_pose: list[float]


@dataclass
class ArmState:
    anchor: Anchor | None = None
    prev_grip_held: bool = False
    prev_anchor_button: bool = False


@dataclass
class BridgeState:
    latest_ee: dict[str, list[float]] = field(default_factory=dict)
    arms: dict[str, ArmState] = field(
        default_factory=lambda: {side: ArmState() for side in ARM_SIDES}
    )
    latest_sample: dict[str, Any] | None = None
    last_sample_t: float | None = None
    warned_schema: bool = False
    first_right_pose: list[float] | None = None
    max_right_move_m: float = 0.0


class SyntheticPoseSource:
    def __init__(self, robot_id: int, radius_m: float, period_s: float) -> None:
        self.robot_id = int(robot_id)
        self.radius_m = float(radius_m)
        self.period_s = float(period_s)
        self.t0 = time.monotonic()

    def sample(self) -> dict[str, Any]:
        elapsed = time.monotonic() - self.t0
        phase = 2.0 * math.pi * elapsed / max(self.period_s, 0.1)
        # Move in XR Y/Z so the robot sees mostly Z and X movement.
        y = 0.6 * self.radius_m * math.sin(phase)
        z = -self.radius_m * math.cos(phase)
        return {
            "schema": PAGE_SCHEMA,
            "robot_id": self.robot_id,
            "t_page_ms": int(time.time() * 1000),
            "hmd": [0.0, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0],
            "right": {
                "pose": [0.0, y, z, 0.0, 0.0, 0.0, 1.0],
                "grip": 1.0,
                "trigger": 0.0,
                "buttons": {"a": 0, "b": 0, "x": 0, "y": 0, "thumb": 0, "menu": 0},
            },
            "left": {
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "grip": 0.0,
                "trigger": 0.0,
                "buttons": {"a": 0, "b": 0, "x": 0, "y": 0, "thumb": 0, "menu": 0},
            },
            "estop": False,
        }


class DirectVrBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ctx = zmq.Context.instance()
        self.state = BridgeState()
        self.pose_sub: zmq.Socket | None = None
        self.sim_sub: zmq.Socket | None = None
        self.push: zmq.Socket | None = None
        self.record_fh = None
        self.synthetic = (
            SyntheticPoseSource(
                args.robot_id,
                args.synthetic_radius_m,
                args.synthetic_period_s,
            )
            if args.source == "synthetic"
            else None
        )

    def start(self) -> None:
        if self.args.source == "zmq":
            self.pose_sub = self.ctx.socket(zmq.SUB)
            self.pose_sub.setsockopt(zmq.RCVHWM, 8)
            self.pose_sub.connect(f"tcp://{self.args.pose_host}:{self.args.pose_port}")
            self.pose_sub.setsockopt(
                zmq.SUBSCRIBE, f"pose.{self.args.robot_id}".encode()
            )
            log.info(
                "pose source: SUB tcp://%s:%d topic=pose.%d",
                self.args.pose_host,
                self.args.pose_port,
                self.args.robot_id,
            )
        else:
            log.info("pose source: synthetic")

        self.sim_sub = self.ctx.socket(zmq.SUB)
        self.sim_sub.setsockopt(zmq.RCVHWM, 8)
        self.sim_sub.connect(f"tcp://{self.args.sim_host}:{self.args.sim_pub_port}")
        self.sim_sub.setsockopt(zmq.SUBSCRIBE, f"tf.links.{self.args.robot_id}".encode())

        self.push = self.ctx.socket(zmq.PUSH)
        self.push.setsockopt(zmq.SNDHWM, 8)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(f"tcp://{self.args.sim_host}:{self.args.sim_pull_port}")
        log.info(
            "sim: SUB tcp://%s:%d, PUSH tcp://%s:%d",
            self.args.sim_host,
            self.args.sim_pub_port,
            self.args.sim_host,
            self.args.sim_pull_port,
        )

        if self.args.record:
            self.args.record.parent.mkdir(parents=True, exist_ok=True)
            self.record_fh = self.args.record.open("w", buffering=1)

    def close(self) -> None:
        for sock in (self.pose_sub, self.sim_sub, self.push):
            if sock is not None:
                sock.close(linger=0)
        if self.record_fh is not None:
            self.record_fh.close()

    def drain(self) -> None:
        if self.sim_sub is not None:
            self._drain_sim()
        if self.synthetic is not None:
            self.state.latest_sample = self.synthetic.sample()
            self.state.last_sample_t = time.monotonic()
        elif self.pose_sub is not None:
            self._drain_pose()

    def _drain_sim(self) -> None:
        assert self.sim_sub is not None
        while True:
            try:
                _topic, payload = self.sim_sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            try:
                msg = unpack_payload(payload)
            except Exception:
                continue
            updates = extract_tf_poses(msg)
            for side, pose in updates.items():
                self.state.latest_ee[side] = pose
                if side == "right":
                    self._track_right_motion(pose)

    def _drain_pose(self) -> None:
        assert self.pose_sub is not None
        while True:
            try:
                _topic, payload = self.pose_sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            try:
                sample = unpack_payload(payload)
            except Exception as exc:
                log.warning("pose decode failed: %s", exc)
                continue
            if sample.get("schema") != PAGE_SCHEMA and not self.state.warned_schema:
                log.warning("unexpected page schema %r", sample.get("schema"))
                self.state.warned_schema = True
            self.state.latest_sample = sample
            self.state.last_sample_t = time.monotonic()

    def _track_right_motion(self, pose: list[float]) -> None:
        if self.state.first_right_pose is None:
            self.state.first_right_pose = list(pose)
            return
        dist = math.sqrt(
            sum((pose[i] - self.state.first_right_pose[i]) ** 2 for i in range(3))
        )
        self.state.max_right_move_m = max(self.state.max_right_move_m, dist)

    def wait_for_ee(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._drain_sim()
            if self.args.side == "both":
                if all(side in self.state.latest_ee for side in ARM_SIDES):
                    return True
            elif self.args.side in self.state.latest_ee:
                return True
            time.sleep(0.02)
        return False

    def loop(self) -> int:
        self.start()
        try:
            if not self.wait_for_ee(self.args.anchor_timeout):
                log.error("no tf.links anchor for robot=%d side=%s", self.args.robot_id, self.args.side)
                return 2
            log.info("anchor ready; sides=%s", sorted(self.state.latest_ee.keys()))
            period = 1.0 / max(self.args.rate_hz, 1.0)
            next_t = time.monotonic()
            end_t = (
                time.monotonic() + self.args.duration_s
                if self.args.duration_s and self.args.duration_s > 0
                else None
            )
            count = 0
            last_log = time.monotonic()
            while True:
                now = time.monotonic()
                if end_t is not None and now >= end_t:
                    break
                if now < next_t:
                    time.sleep(min(0.01, next_t - now))
                    continue
                next_t += period
                self.drain()
                sent = self.tick(now)
                count += int(sent)
                if now - last_log >= 5.0:
                    log.info(
                        "sent %.1f Hz, max_right_move=%.5fm",
                        count / max(now - last_log, 1e-9),
                        self.state.max_right_move_m,
                    )
                    count = 0
                    last_log = now
            if end_t is not None:
                print("max_right_move_m", round(self.state.max_right_move_m, 6))
                return 0 if self.state.max_right_move_m >= self.args.min_move else 1
            return 0
        finally:
            self.close()

    def tick(self, now_s: float) -> bool:
        sample = self.state.latest_sample
        stale = (
            self.state.last_sample_t is None
            or (now_s - self.state.last_sample_t) * 1000.0 > self.args.stale_ms
        )
        estop = bool(sample.get("estop", False)) if isinstance(sample, dict) else False
        if stale or estop or sample is None:
            self._clear_for_hold(stale=stale, estop=estop)
            return False

        targets = self._hold_targets()
        rel = {
            side: {"shoulder_pan": 0.0, "gripper": 0.0}
            for side in targets.keys()
        }
        moving = False
        for side in ARM_SIDES:
            if self.args.side != "both" and side != self.args.side:
                self._release_side(side)
                continue
            target, gripper_delta = self._side_target(side, sample.get(side))
            if target is None:
                continue
            targets[side] = target
            rel.setdefault(side, {"shoulder_pan": 0.0, "gripper": 0.0})
            rel[side]["gripper"] = gripper_delta
            moving = True

        if not moving or not targets:
            return False
        self._send_targets(targets, rel)
        if self.record_fh is not None:
            self.record_fh.write(
                json.dumps(
                    {
                        "t": now_s,
                        "robot_id": self.args.robot_id,
                        "targets": sorted(targets.keys()),
                        "moving": moving,
                        "max_right_move_m": self.state.max_right_move_m,
                    }
                )
                + "\n"
            )
        return True

    def _clear_for_hold(self, *, stale: bool, estop: bool) -> None:
        if stale or estop:
            for arm in self.state.arms.values():
                arm.anchor = None
                arm.prev_grip_held = False
                arm.prev_anchor_button = False

    def _release_side(self, side: str) -> None:
        arm = self.state.arms[side]
        arm.anchor = None
        arm.prev_grip_held = False
        arm.prev_anchor_button = False

    def _side_target(
        self, side: str, side_payload: Any
    ) -> tuple[list[float] | None, float]:
        if not isinstance(side_payload, dict):
            self._release_side(side)
            return None, 0.0
        ctrl_pose = pose7(side_payload.get("pose"))
        ee_pose = self.state.latest_ee.get(side)
        if ctrl_pose is None or ee_pose is None:
            return None, 0.0

        grip = float(side_payload.get("grip", 0.0) or 0.0)
        held = grip >= self.args.grip_threshold
        anchor_key = BUTTON_ANCHOR[side]
        anchor_pressed = bool(buttons(side_payload).get(anchor_key, 0))
        arm = self.state.arms[side]
        anchor_edge = anchor_pressed and not arm.prev_anchor_button

        if not held:
            arm.anchor = None
            arm.prev_grip_held = False
            arm.prev_anchor_button = anchor_pressed
            return None, 0.0

        if not arm.prev_grip_held or arm.anchor is None or anchor_edge:
            arm.anchor = Anchor(ctrl_pose=list(ctrl_pose), ee_pose=list(ee_pose))
            log.info("captured %s anchor", side)

        arm.prev_grip_held = True
        arm.prev_anchor_button = anchor_pressed

        delta_xr = vec3_delta(ctrl_pose, arm.anchor.ctrl_pose)
        delta_robot = xr_delta_to_robot(delta_xr, self.args.position_scale)
        target = list(arm.anchor.ee_pose)
        for idx in range(3):
            target[idx] += delta_robot[idx]
        target = clamp_pose_to_workspace(side, target)
        # Keep orientation floating with measured tf.links orientation. This
        # avoids over-constraining the 4-DoF IK arm during translation checks.
        current = self.state.latest_ee.get(side)
        if isinstance(current, list) and len(current) == 7:
            target[3:7] = current[3:7]
        trigger = max(0.0, min(1.0, float(side_payload.get("trigger", 0.0) or 0.0)))
        return target, -trigger * self.args.gripper_per_tick

    def _hold_targets(self) -> dict[str, list[float]]:
        targets: dict[str, list[float]] = {}
        for side, pose in self.state.latest_ee.items():
            if side in ARM_SIDES:
                targets[side] = list(pose)
        return targets

    def _send_targets(
        self,
        targets: dict[str, list[float]],
        rel: dict[str, dict[str, float]],
    ) -> None:
        assert self.push is not None
        payload = build_command_payload(self.args.robot_id, targets, rel)
        self.push.send(pack_command(payload))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Directly bridge WebXR pose.<robot_id> messages to sim EE targets.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--source", choices=["zmq", "synthetic"], default="zmq")
    parser.add_argument("--pose-host", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=7001)
    parser.add_argument("--sim-host", default="100.80.87.68")
    parser.add_argument("--sim-pub-port", type=int, default=5555)
    parser.add_argument("--sim-pull-port", type=int, default=5556)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--side", choices=["right", "left", "both"], default="right")
    parser.add_argument("--rate-hz", type=float, default=60.0)
    parser.add_argument("--position-scale", type=float, default=1.0)
    parser.add_argument("--grip-threshold", type=float, default=0.5)
    parser.add_argument("--gripper-per-tick", type=float, default=0.01)
    parser.add_argument("--stale-ms", type=float, default=150.0)
    parser.add_argument("--anchor-timeout", type=float, default=3.0)
    parser.add_argument("--synthetic-radius-m", type=float, default=0.05)
    parser.add_argument("--synthetic-period-s", type=float, default=3.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--min-move", type=float, default=0.002)
    parser.add_argument("--record", type=Path, default=None)
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )
    try:
        return DirectVrBridge(args).loop()
    except KeyboardInterrupt:
        log.info("interrupted")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
