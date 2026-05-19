"""
xlerobot-webxr / tools/sim_probe.py

Mac 에서 indoory_isaac_sim 서버까지의 네트워크/와이어 경로를 한 방에 검증.
sim 측 클라이언트 API 명세서 (§4-§5) 가 정의한 wire 만 가지고 pyzmq + msgpack
두 패키지로 통신한다 — sim/isaaclab 코드 import 안 함.

용도:
  - Tailscale 으로 Home Server (100.x.y.z) 의 sim 까지 라우팅이 살아있는지
  - sim 의 :5557 REP 가 응답하는지 (fleet_info, topic_list)
  - sim 의 :5555 PUB 에서 proprio.<i> 가 실제로 나오는지 + 대략의 Hz
  - 옵션: :5556 PULL 에 v1 zero-action 페이로드를 한 번 PUSH 해서 라우팅 확인

이 스크립트는 Mac proxy / VR bridge 와 무관. Tailscale 직결 검증만 한다.

사용:
  python3 tools/sim_probe.py 100.80.87.68
  python3 tools/sim_probe.py 100.80.87.68 --robot-id 0 --duration 3
  python3 tools/sim_probe.py 100.80.87.68 --push-zero    # PUSH 슬롯 살아있는지도 검증

요구:
  python3 -m pip install pyzmq msgpack
"""

from __future__ import annotations

import argparse
import sys
import time
from contextlib import contextmanager

import msgpack
import zmq


# ───────────────────────────────────────────────────────────────────────────
# helpers (스펙 §3/§5 의 wire spec 그대로)
# ───────────────────────────────────────────────────────────────────────────

@contextmanager
def _socket(ctx: zmq.Context, kind: int, host: str, port: int, **opts):
    s = ctx.socket(kind)
    for k, v in opts.items():
        s.setsockopt(getattr(zmq, k), v)
    s.connect(f"tcp://{host}:{port}")
    try:
        yield s
    finally:
        s.close(linger=0)


def rpc(req_sock: zmq.Socket, op: str, **kwargs) -> dict:
    """스펙 §5.1 — REQ/REP 동기 1:1. 응답 dict 반환."""
    req_sock.send(msgpack.packb({"schema": "xlerobot_v1", "op": op, **kwargs},
                                use_bin_type=True))
    return msgpack.unpackb(req_sock.recv(), raw=False)


def pack_command_zero(robot_id: int) -> bytes:
    """스펙 §3.1 v1 — 14-D 관절 0 + 3-D 베이스 속도 0 + frame=body."""
    return msgpack.packb({
        "schema":               "xlerobot_v1",
        "stamp_ns":             time.monotonic_ns(),
        "robot_id":             robot_id,
        "frame":                "body",
        "arm_joint_pos_target": [0.0] * 14,
        "base_cmd_vel":         [0.0, 0.0, 0.0],
    }, use_bin_type=True)


# ───────────────────────────────────────────────────────────────────────────
# probes
# ───────────────────────────────────────────────────────────────────────────

def probe_tcp(host: str, port: int, timeout: float = 2.0) -> bool:
    """plain TCP 가 닿는지 확인 — ZMQ 가 안 붙는 게 라우팅 문제인지 와이어 문제인지 가르기 위함."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False
    finally:
        s.close()


def probe_rpc(ctx: zmq.Context, host: str) -> tuple[bool, dict]:
    """:5557 REP 에 fleet_info + topic_list 두 번 호출."""
    info = {}
    with _socket(ctx, zmq.REQ, host, 5557,
                 RCVTIMEO=2000, SNDTIMEO=2000, LINGER=0) as sock:
        try:
            info["fleet_info"] = rpc(sock, "fleet_info")
            info["topic_list"] = rpc(sock, "topic_list")
            return True, info
        except zmq.Again:
            return False, {"error": "RPC timeout (2s) - sim did not respond or Tailscale blocked"}
        except zmq.ZMQError as e:
            return False, {"error": f"ZMQError: {e}"}


def probe_proprio(ctx: zmq.Context, host: str, robot_id: int, duration: float) -> dict:
    """:5555 PUB 에서 proprio.<robot_id> 를 duration 초 동안 받아본다.

    반환: {"count": <int>, "hz": <float>, "first": <dict|None>, "schema": <str|None>}
    """
    topic = f"proprio.{robot_id}".encode()
    with _socket(ctx, zmq.SUB, host, 5555,
                 RCVHWM=8, RCVTIMEO=int(duration * 1000) + 500) as sock:
        sock.setsockopt(zmq.SUBSCRIBE, topic)
        # slow-joiner 회피 — 짧게 대기 후 측정 시작
        time.sleep(0.2)
        count = 0
        first = None
        schema = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            try:
                topic_b, payload_b = sock.recv_multipart()
            except zmq.Again:
                break
            count += 1
            if first is None:
                first = msgpack.unpackb(payload_b, raw=False)
                schema = first.get("schema")
        elapsed = max(1e-9, time.monotonic() - t0)
        return {"count": count, "hz": count / elapsed, "first": first, "schema": schema}


def probe_push_zero(ctx: zmq.Context, host: str, robot_id: int) -> None:
    """:5556 PULL 에 zero-action 한 발 PUSH (스펙 §3.4 — 정지 명령은 [0,0,0]).

    PUSH 는 단방향이라 ACK 가 없다. 성공/실패는 zmq.ZMQError 잡기뿐.
    """
    with _socket(ctx, zmq.PUSH, host, 5556,
                 SNDHWM=4, SNDTIMEO=2000, LINGER=0) as sock:
        sock.send(pack_command_zero(robot_id))


# ───────────────────────────────────────────────────────────────────────────
# main
# ───────────────────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(description="indoory_isaac_sim connectivity and wire probe")
    p.add_argument("host", help="sim Tailscale/LAN IP, for example 100.80.87.68")
    p.add_argument("--robot-id", type=int, default=0)
    p.add_argument("--duration", type=float, default=3.0,
                   help="proprio measurement duration in seconds")
    p.add_argument("--push-zero", action="store_true",
                   help="send one zero-action payload to :5556 PULL")
    p.add_argument("--no-proprio", action="store_true",
                   help="skip PUB measurement and only check RPC")
    p.add_argument("--pretty", action="store_true",
                   help="use Unicode status symbols in terminal output")
    args = p.parse_args()

    ok_mark = "✓" if args.pretty else "[OK]"
    fail_mark = "✗" if args.pretty else "[FAIL]"
    arrow = "→" if args.pretty else "->"
    bullet = "·" if args.pretty else "-"
    times = "×" if args.pretty else "x"
    approx = "≈" if args.pretty else "~"
    dash = "—" if args.pretty else "-"

    host = args.host
    print(f"=== sim_probe {arrow} {host} ===")
    print()

    # 1) TCP reachability
    print("[1] TCP routing")
    for port, label in [(5555, "PUB sensors"), (5556, "PULL actions"), (5557, "REP rpc")]:
        ok = probe_tcp(host, port)
        mark = ok_mark if ok else fail_mark
        print(f"    {mark}  {host}:{port}  ({label})")
        if not ok and port == 5557:
            print(f"        {arrow} routing is unavailable; run 'tailscale ping <host>' first")
            return 2
    print()

    ctx = zmq.Context.instance()

    # 2) RPC fleet_info + topic_list
    print("[2] RPC :5557 (fleet_info, topic_list)")
    ok, info = probe_rpc(ctx, host)
    if not ok:
        print(f"    {fail_mark} {info.get('error')}")
        return 3
    fleet = info["fleet_info"]
    topics = info["topic_list"]
    print(f"    {ok_mark} fleet_info.ok        = {fleet.get('ok')}")
    print(f"    {ok_mark} fleet_info.num_robots= {fleet.get('num_robots')}")
    print(f"    {ok_mark} topic_list.ok        = {topics.get('ok')}")
    print(f"    {ok_mark} topic_list.topics    = {len(topics.get('topics', []))} entries")
    # 활성 토픽 카테고리별로 카운트
    cats: dict[str, int] = {}
    for t in topics.get("topics", []):
        base = t.rsplit(".", 1)[0]  # proprio.0 -> proprio, rgb.front.0 -> rgb.front
        cats[base] = cats.get(base, 0) + 1
    for base, n in sorted(cats.items()):
        print(f"          {bullet} {base:18s} {times} {n}")
    print()

    # 3) proprio.<i> 흐름 측정
    if not args.no_proprio:
        print(f"[3] PUB :5555 proprio.{args.robot_id} ({args.duration:.1f}s measurement)")
        if args.robot_id >= fleet.get("num_robots", 0):
            print(
                f"    {fail_mark} robot_id={args.robot_id} exceeds "
                f"fleet size={fleet.get('num_robots')} {dash} skipped"
            )
        else:
            r = probe_proprio(ctx, host, args.robot_id, args.duration)
            if r["count"] == 0:
                print(f"    {fail_mark} 0 messages {dash} check sim publish state and topic name")
            else:
                print(f"    {ok_mark} count={r['count']}  rate{approx}{r['hz']:.1f} Hz  schema={r['schema']!r}")
                p0 = r["first"]
                if p0:
                    bp = p0.get("base_pose")
                    print(f"          base_pose  = {bp}")
                    print(f"          base_twist = {p0.get('base_twist')}")
                    print(f"          joint_pos[0:3] = {p0.get('joint_pos', [])[:3]}")
        print()

    # 4) 옵션: zero-action PUSH
    if args.push_zero:
        print(f"[4] PUSH :5556 zero-action (robot_id={args.robot_id})")
        try:
            probe_push_zero(ctx, host, args.robot_id)
            print(f"    {ok_mark} send OK (PUSH has no ACK; verify through proprio changes)")
        except zmq.ZMQError as e:
            print(f"    {fail_mark} ZMQError: {e}")
        print()

    print("=== done ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
