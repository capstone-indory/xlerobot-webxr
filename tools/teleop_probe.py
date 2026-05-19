"""
xlerobot-webxr / tools/teleop_probe.py

End-to-end teleop probe for the split Mac/Home setup.

This script does not need a Quest. It sends synthetic WebXR page payloads to
mac_proxy's /ws endpoint, verifies that mac_proxy republishes pose.<robot_id>
on ZMQ, and can verify that the running Home Server VR bridge turns the same
pose stream into visible sim arm/gripper motion.

Typical Home Server run while mac_proxy, indoory_isaac_sim, and
examples/vr_teleop_bridge.py are already running:

  python3 tools/teleop_probe.py --mac-host 100.81.219.12 --sim-host 127.0.0.1

Typical Mac run against a remote Home Server:

  python3 tools/teleop_probe.py --mac-host 127.0.0.1 --sim-host 100.80.87.68
"""

from __future__ import annotations

import argparse
import asyncio
import ssl
import time
from dataclasses import dataclass
from typing import Any

import aiohttp
import msgpack
import zmq


BUTTONS = {"a": 0, "b": 0, "x": 0, "y": 0, "thumb": 0, "menu": 0}
RIGHT_POSE = [0.10, 1.20, -0.35, 0.0, 0.0, 0.0, 1.0]


@dataclass
class ProbeConfig:
    mac_host: str
    sim_host: str
    robot_id: int
    page_port: int
    pose_port: int
    sim_pub_port: int
    sim_pull_port: int
    frames: int
    grip: float
    pose_trigger: float
    arm_move_m: float
    arm_threshold: float
    open_delta: float
    close_threshold: float
    open_threshold: float


def _ssl_unverified() -> ssl.SSLContext:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


def _page_payload(
    *,
    pose: list[float] | None = None,
    grip: float = 1.0,
    trigger: float = 0.0,
    estop: bool = False,
) -> dict[str, Any]:
    return {
        "t": time.perf_counter() * 1000.0,
        "hmd": [0.0, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0],
        "left": {
            "pose": None,
            "grip": 0.0,
            "trigger": 0.0,
            "buttons": dict(BUTTONS),
        },
        "right": {
            "pose": list(RIGHT_POSE if pose is None else pose),
            "grip": float(grip),
            "trigger": float(trigger),
            "buttons": dict(BUTTONS),
        },
        "estop": bool(estop),
    }


def _pack_v11_relative_gripper(robot_id: int, delta: float) -> bytes:
    return msgpack.packb(
        {
            "schema": "xlerobot_v1.1",
            "stamp_ns": time.monotonic_ns(),
            "robot_id": int(robot_id),
            "frame": "body",
            "base_cmd_vel": [0.0, 0.0, 0.0],
            "arm_joint_relative_target": {
                "right": {"shoulder_pan": 0.0, "gripper": float(delta)},
                "left": {"shoulder_pan": 0.0, "gripper": 0.0},
            },
            "head_joint_relative_target": {"head_pan": 0.0, "head_tilt": 0.0},
        },
        use_bin_type=True,
    )


def _open_sub(endpoint: str, topic: bytes) -> zmq.Socket:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 1000)
    sock.connect(endpoint)
    sock.setsockopt(zmq.SUBSCRIBE, topic)
    return sock


def _drain_sub(sock: zmq.Socket) -> list[tuple[bytes, dict[str, Any]]]:
    out: list[tuple[bytes, dict[str, Any]]] = []
    while True:
        try:
            topic, payload = sock.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return out
        try:
            out.append((topic, msgpack.unpackb(payload, raw=False)))
        except Exception:
            continue


async def _send_ws_frames(
    cfg: ProbeConfig,
    *,
    grip: float,
    trigger: float,
    frames: int | None = None,
    pose: list[float] | None = None,
) -> None:
    url = f"wss://{cfg.mac_host}:{cfg.page_port}/ws?robot={cfg.robot_id}"
    ssl_ctx = _ssl_unverified()
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(url, ssl=ssl_ctx, heartbeat=10) as ws:
            await ws.send_json({"select_robot": cfg.robot_id})
            for _ in range(cfg.frames if frames is None else frames):
                await ws.send_json(
                    _page_payload(pose=pose, grip=grip, trigger=trigger)
                )
                await asyncio.sleep(1.0 / 90.0)


async def _open_ws(cfg: ProbeConfig):
    url = f"wss://{cfg.mac_host}:{cfg.page_port}/ws?robot={cfg.robot_id}"
    ssl_ctx = _ssl_unverified()
    session = aiohttp.ClientSession()
    try:
        ws = await session.ws_connect(url, ssl=ssl_ctx, heartbeat=10)
    except Exception:
        await session.close()
        raise
    await ws.send_json({"select_robot": cfg.robot_id})
    return session, ws


async def _send_ws_phase(
    ws: aiohttp.ClientWebSocketResponse,
    *,
    pose: list[float],
    grip: float,
    trigger: float,
    estop: bool,
    frames: int,
    on_frame=None,
) -> None:
    for _ in range(frames):
        await ws.send_json(
            _page_payload(pose=pose, grip=grip, trigger=trigger, estop=estop)
        )
        if on_frame is not None:
            on_frame()
        await asyncio.sleep(1.0 / 90.0)


async def probe_mac_pose(cfg: ProbeConfig) -> dict[str, Any]:
    endpoint = f"tcp://{cfg.mac_host}:{cfg.pose_port}"
    topic = f"pose.{cfg.robot_id}".encode()
    sub = _open_sub(endpoint, topic)
    try:
        await asyncio.sleep(0.2)
        task = asyncio.create_task(
            _send_ws_frames(cfg, grip=cfg.grip, trigger=cfg.pose_trigger)
        )
        count = 0
        first: dict[str, Any] | None = None
        deadline = time.monotonic() + max(2.0, cfg.frames / 60.0)
        while time.monotonic() < deadline:
            for topic_b, msg in _drain_sub(sub):
                if topic_b != topic:
                    continue
                count += 1
                if first is None:
                    first = msg
            if task.done() and count >= cfg.frames:
                break
            await asyncio.sleep(0.005)
        await task
        return {"count": count, "first": first}
    finally:
        sub.close(linger=0)


def _latest_jaw(ctx: zmq.Context, cfg: ProbeConfig, duration_s: float) -> float | None:
    topic = f"proprio.{cfg.robot_id}".encode()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 1000)
    sub.connect(f"tcp://{cfg.sim_host}:{cfg.sim_pub_port}")
    sub.setsockopt(zmq.SUBSCRIBE, topic)
    sub.RCVTIMEO = 250
    deadline = time.monotonic() + duration_s
    last: float | None = None
    try:
        while time.monotonic() < deadline:
            try:
                _, payload = sub.recv_multipart()
            except zmq.Again:
                continue
            msg = msgpack.unpackb(payload, raw=False)
            names = msg.get("joint_names_pos") or []
            vals = msg.get("joint_pos") or []
            if "Jaw" not in names:
                continue
            idx = names.index("Jaw")
            if idx < len(vals):
                last = float(vals[idx])
    finally:
        sub.close(linger=0)
    return last


def _open_tf_sub(ctx: zmq.Context, cfg: ProbeConfig) -> zmq.Socket:
    topic = f"tf.links.{cfg.robot_id}".encode()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 1000)
    sub.connect(f"tcp://{cfg.sim_host}:{cfg.sim_pub_port}")
    sub.setsockopt(zmq.SUBSCRIBE, topic)
    return sub


def _drain_tf_ee(sock: zmq.Socket, *, target_name: str = "gripper_right") -> list[list[float]]:
    out: list[list[float]] = []
    while True:
        try:
            _, payload = sock.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return out
        try:
            msg = msgpack.unpackb(payload, raw=False)
        except Exception:
            continue
        for entry in msg.get("targets", []) or []:
            if entry.get("name") != target_name:
                continue
            pose = entry.get("pose")
            if isinstance(pose, (list, tuple)) and len(pose) == 7:
                out.append([float(v) for v in pose])


async def _collect_latest_tf(sock: zmq.Socket, duration_s: float) -> list[float] | None:
    latest: list[float] | None = None
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        for pose in _drain_tf_ee(sock):
            latest = pose
        await asyncio.sleep(0.01)
    return latest


def _xyz_dist(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None:
        return None
    return sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)) ** 0.5


async def probe_arm_via_bridge(cfg: ProbeConfig) -> dict[str, Any]:
    """Send a continuous clutch-held pose sequence and watch sim tf.links."""
    ctx = zmq.Context.instance()
    tf_sub = _open_tf_sub(ctx, cfg)
    moved_pose = list(RIGHT_POSE)
    moved_pose[2] -= float(cfg.arm_move_m)
    move_samples: list[list[float]] = []
    try:
        base = await _collect_latest_tf(tf_sub, 0.8)
        session, ws = await _open_ws(cfg)
        try:
            await _send_ws_phase(
                ws,
                pose=RIGHT_POSE,
                grip=0.0,
                trigger=0.0,
                estop=False,
                frames=max(15, cfg.frames // 4),
                on_frame=lambda: _drain_tf_ee(tf_sub),
            )
            anchor_latest: list[float] | None = None

            def drain_anchor() -> None:
                nonlocal anchor_latest
                for pose in _drain_tf_ee(tf_sub):
                    anchor_latest = pose

            await _send_ws_phase(
                ws,
                pose=RIGHT_POSE,
                grip=1.0,
                trigger=0.0,
                estop=False,
                frames=max(90, cfg.frames),
                on_frame=drain_anchor,
            )
            anchor = anchor_latest or await _collect_latest_tf(tf_sub, 0.1)

            moved_latest: list[float] | None = None

            def drain_move() -> None:
                nonlocal moved_latest
                for pose in _drain_tf_ee(tf_sub):
                    moved_latest = pose
                    move_samples.append(pose)

            await _send_ws_phase(
                ws,
                pose=moved_pose,
                grip=1.0,
                trigger=0.0,
                estop=False,
                frames=max(150, cfg.frames),
                on_frame=drain_move,
            )
            moved = moved_latest or await _collect_latest_tf(tf_sub, 0.1)

            returned_latest: list[float] | None = None

            def drain_return() -> None:
                nonlocal returned_latest
                for pose in _drain_tf_ee(tf_sub):
                    returned_latest = pose

            await _send_ws_phase(
                ws,
                pose=RIGHT_POSE,
                grip=1.0,
                trigger=0.0,
                estop=False,
                frames=max(90, cfg.frames),
                on_frame=drain_return,
            )
            returned = returned_latest or await _collect_latest_tf(tf_sub, 0.1)
            await _send_ws_phase(
                ws,
                pose=RIGHT_POSE,
                grip=0.0,
                trigger=0.0,
                estop=False,
                frames=15,
                on_frame=lambda: _drain_tf_ee(tf_sub),
            )
        finally:
            await ws.close()
            await session.close()
    finally:
        tf_sub.close(linger=0)

    max_move = None
    moved_best = moved
    if anchor is not None and move_samples:
        scored = [(_xyz_dist(anchor, pose), pose) for pose in move_samples]
        scored = [(d, p) for d, p in scored if d is not None]
        if scored:
            max_move, moved_best = max(scored, key=lambda item: item[0])
    if max_move is None:
        max_move = _xyz_dist(anchor, moved)
    return {
        "base": base,
        "anchor": anchor,
        "moved": moved,
        "moved_best": moved_best,
        "returned": returned,
        "max_move": max_move,
        "return_dist": _xyz_dist(anchor, returned),
    }


def _send_direct_gripper(
    ctx: zmq.Context,
    cfg: ProbeConfig,
    *,
    delta: float,
    duration_s: float,
    hz: float = 120.0,
) -> None:
    push = ctx.socket(zmq.PUSH)
    push.connect(f"tcp://{cfg.sim_host}:{cfg.sim_pull_port}")
    try:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            push.send(_pack_v11_relative_gripper(cfg.robot_id, delta))
            time.sleep(1.0 / hz)
    finally:
        push.close(linger=0)


async def probe_gripper_via_bridge(cfg: ProbeConfig) -> dict[str, Any]:
    """Prime Jaw open directly, then close it through Mac pose -> VR bridge."""
    ctx = zmq.Context.instance()
    base = _latest_jaw(ctx, cfg, 0.8)
    _send_direct_gripper(ctx, cfg, delta=cfg.open_delta, duration_s=1.2)
    opened = _latest_jaw(ctx, cfg, 0.8)
    # sim_server keeps the latest command per robot and re-applies it every
    # tick. Flush the direct-open relative delta before testing whether the
    # VR bridge itself emits trigger-driven close deltas.
    _send_direct_gripper(ctx, cfg, delta=0.0, duration_s=0.3)
    await _send_ws_frames(
        cfg,
        grip=1.0,
        trigger=1.0,
        frames=max(cfg.frames, 120),
        pose=RIGHT_POSE,
    )
    closed = _latest_jaw(ctx, cfg, 1.0)
    _send_direct_gripper(ctx, cfg, delta=0.0, duration_s=0.2)
    return {"base": base, "opened": opened, "closed": closed}


def _fmt(v: float | None) -> str:
    return "None" if v is None else f"{v:.5f}"


def _fmt_xyz(pose: list[float] | None) -> str:
    if pose is None:
        return "None"
    return "[" + ", ".join(f"{float(v):+.5f}" for v in pose[:3]) + "]"


async def main_async(args: argparse.Namespace) -> int:
    cfg = ProbeConfig(
        mac_host=args.mac_host,
        sim_host=args.sim_host,
        robot_id=args.robot_id,
        page_port=args.page_port,
        pose_port=args.pose_port,
        sim_pub_port=args.sim_pub_port,
        sim_pull_port=args.sim_pull_port,
        frames=args.frames,
        grip=args.grip,
        pose_trigger=args.pose_trigger,
        arm_move_m=args.arm_move_m,
        arm_threshold=args.arm_threshold,
        open_delta=args.open_delta,
        close_threshold=args.close_threshold,
        open_threshold=args.open_threshold,
    )

    ok_arm = True
    if not args.no_arm:
        print("[1] arm path: Mac pose -> VR bridge -> sim tf.links")
        arm = await probe_arm_via_bridge(cfg)
        base = arm["base"]
        anchor = arm["anchor"]
        moved = arm["moved_best"] or arm["moved"]
        returned = arm["returned"]
        print(f"    base_xyz={_fmt_xyz(base)}")
        print(f"    anchor_xyz={_fmt_xyz(anchor)} moved_xyz={_fmt_xyz(moved)}")
        print(
            f"    max_move={_fmt(arm['max_move'])} "
            f"return_dist={_fmt(arm['return_dist'])}"
        )
        ok_arm = arm["max_move"] is not None and arm["max_move"] >= cfg.arm_threshold
        if not ok_arm:
            print(
                "    FAIL: sim tf.links did not move enough; check that "
                "examples/vr_teleop_bridge.py is running with --source zmq, "
                "right.grip >= 0.5, and robot_id/topic match"
            )

    ok_close = True
    ok_open = True
    if args.gripper and not args.no_gripper:
        print("[2] gripper path: direct open -> Mac trigger close through VR bridge")
        grip = await probe_gripper_via_bridge(cfg)
        base = grip["base"]
        opened = grip["opened"]
        closed = grip["closed"]
        opened_delta = None if base is None or opened is None else opened - base
        closed_delta = None if opened is None or closed is None else opened - closed
        print(
            f"    jaw base={_fmt(base)} opened={_fmt(opened)} closed={_fmt(closed)}"
        )
        print(
            f"    opened_delta={_fmt(opened_delta)} "
            f"trigger_close_delta={_fmt(closed_delta)}"
        )
        ok_open = opened is not None and (
            opened >= cfg.open_threshold
            or (closed_delta is not None and closed_delta >= cfg.close_threshold)
        )
        ok_close = closed_delta is not None and closed_delta >= cfg.close_threshold
        if not ok_open:
            print("    FAIL: Jaw was not open enough before trigger-close phase")
        if not ok_close:
            print(
                "    FAIL: Mac trigger did not close Jaw through the running VR bridge; "
                "check bridge process, right.grip, estop, and trigger->gripper sign"
            )
    elif not args.no_gripper:
        print("[2] gripper path skipped (pass --gripper to run it)")

    print("[3] mac_proxy /ws -> ZMQ pose stream")
    mac = await probe_mac_pose(cfg)
    first = mac["first"] or {}
    right = first.get("right") or {}
    print(f"    count={mac['count']} topic=pose.{cfg.robot_id}")
    print(
        "    first="
        f"schema={first.get('schema')!r} robot_id={first.get('robot_id')} "
        f"frame={first.get('frame')!r} estop={first.get('estop')} "
        f"right.pose={right.get('pose') is not None} "
        f"right.grip={right.get('grip')} right.trigger={right.get('trigger')}"
    )
    ok_mac = (
        mac["count"] >= max(10, cfg.frames // 2)
        and first.get("schema") == "xlerobot_v1.1.page"
        and first.get("robot_id") == cfg.robot_id
        and first.get("frame") == "local-floor"
        and (first.get("right") or {}).get("pose") is not None
    )

    if args.no_gripper:
        print("    gripper check skipped")

    if args.no_arm:
        print("    arm check skipped")

    ok = ok_mac and ok_open and ok_close
    ok = ok and ok_arm
    print("PASS" if ok else "FAIL")
    return 0 if ok else 3


def main() -> int:
    p = argparse.ArgumentParser(
        description="Probe Mac /ws -> ZMQ pose and optional sim gripper teleop path."
    )
    p.add_argument("--mac-host", required=True, help="Mac LAN/Tailscale IP or 127.0.0.1")
    p.add_argument("--sim-host", default="127.0.0.1", help="Home Server sim host")
    p.add_argument("--robot-id", type=int, default=0)
    p.add_argument("--page-port", type=int, default=8443)
    p.add_argument("--pose-port", type=int, default=7001)
    p.add_argument("--sim-pub-port", type=int, default=5555)
    p.add_argument("--sim-pull-port", type=int, default=5556)
    p.add_argument("--frames", type=int, default=90)
    p.add_argument("--grip", type=float, default=1.0)
    p.add_argument(
        "--pose-trigger",
        type=float,
        default=0.0,
        help="trigger value used for the initial mac_proxy pose publish check",
    )
    p.add_argument(
        "--arm-move-m",
        type=float,
        default=0.10,
        help="WebXR -Z controller displacement used for the sim arm movement check",
    )
    p.add_argument(
        "--arm-threshold",
        type=float,
        default=0.002,
        help="minimum observed gripper_right tf movement in meters",
    )
    p.add_argument(
        "--no-arm",
        action="store_true",
        help="skip sim tf.links arm movement verification",
    )
    p.add_argument("--open-delta", type=float, default=0.01)
    p.add_argument("--open-threshold", type=float, default=0.01)
    p.add_argument("--close-threshold", type=float, default=0.01)
    p.add_argument(
        "--gripper",
        action="store_true",
        help="also verify trigger-close by opening Jaw directly, then closing through VR bridge",
    )
    p.add_argument(
        "--no-gripper",
        action="store_true",
        help="skip gripper trigger-close verification",
    )
    args = p.parse_args()
    return asyncio.run(main_async(args))


if __name__ == "__main__":
    raise SystemExit(main())
