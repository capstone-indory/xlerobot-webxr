"""
xlerobot-webxr / tools/run.py

XLerobot WebXR 스택 전체를 한 명령으로 띄우는 launcher.

내부적으로 두 자식 프로세스를 같이 관리한다:
  1) tools/mac_proxy.py        — 페이지 + /ws → ZMQ + WebRTC signaling + SFU
  2) tools/sim_video_bridge.py — sim head camera (rgb.front.<i>) → WebRTC track
       (또는 --video-source fake → fake_producer.py demo.mp4 loop)
       (또는 --video-source none → 영상 없이 텔레옵 wire 만)

설계 결정:
  • 각 컴포넌트는 자기완비 스크립트로 유지 (단독 실행 + 테스트 모두 가능).
  • launcher 는 그 위에 얇은 orchestrator — subprocess 로 띄우고, 로그 합치고,
    Ctrl+C / 비정상 종료를 깔끔히 회수만 한다.
  • mac_proxy 의 :signal-port 가 LISTEN 상태가 된 뒤에야 video publisher 를
    spawn 해서 'bridge 가 먼저 떠서 connect refused' 케이스를 막는다.

사용:
  python3 tools/run.py                           # 모든 default (sim @ 100.80.87.68)
  python3 tools/run.py --sim-robot-id 1
  python3 tools/run.py --sim-host 100.80.87.68 --sim-topic rgb.wrist
  python3 tools/run.py --video-source fake       # sim 미연결 시 demo.mp4 loop
  python3 tools/run.py --video-source none       # 영상 없이 mac_proxy 만
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import pathlib
import signal
import sys
from dataclasses import dataclass, field
from typing import List, Optional

ROOT = pathlib.Path(__file__).resolve().parent
PROXY = ROOT / "mac_proxy.py"
BRIDGE = ROOT / "sim_video_bridge.py"
FAKE = ROOT / "fake_producer.py"
DEMO_MP4 = ROOT / "webxr" / "assets" / "demo.mp4"

log = logging.getLogger("run")


# ===========================================================================
# Child process orchestration
# ===========================================================================

@dataclass
class ChildSpec:
    label: str               # 로그 prefix (예: "proxy ", "video ")
    argv: List[str]
    start_after_port: Optional[tuple[str, int]] = None  # (host, port) — 이게 LISTEN 된 뒤 spawn


@dataclass
class Child:
    spec: ChildSpec
    proc: asyncio.subprocess.Process
    log_task: Optional[asyncio.Task] = None


async def _stream_output(proc: asyncio.subprocess.Process, label: str) -> None:
    """child stdout 을 라인 단위로 [label] prefix 붙여 본 stdout 으로 forward."""
    assert proc.stdout is not None
    while True:
        line = await proc.stdout.readline()
        if not line:
            break
        try:
            sys.stdout.write(f"[{label}] " + line.decode(errors="replace"))
            sys.stdout.flush()
        except Exception:
            pass


async def _wait_port(host: str, port: int, timeout: float = 10.0) -> bool:
    """host:port 가 TCP accept 할 때까지 짧은 polling 으로 대기. 시간 초과 시 False."""
    deadline = asyncio.get_event_loop().time() + timeout
    while asyncio.get_event_loop().time() < deadline:
        try:
            _, w = await asyncio.open_connection(host, port)
            w.close()
            try:
                await w.wait_closed()
            except Exception:
                pass
            return True
        except OSError:
            await asyncio.sleep(0.2)
    return False


async def _spawn(spec: ChildSpec) -> Child:
    if spec.start_after_port is not None:
        host, port = spec.start_after_port
        ok = await _wait_port(host, port, timeout=15.0)
        if not ok:
            raise RuntimeError(
                f"timeout waiting for {host}:{port} (parent proxy 가 안 떴음)"
            )
    proc = await asyncio.create_subprocess_exec(
        *spec.argv,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    child = Child(spec=spec, proc=proc)
    child.log_task = asyncio.create_task(_stream_output(proc, spec.label))
    return child


async def _terminate(child: Child, timeout: float = 3.0) -> None:
    """SIGTERM 후 timeout 내 종료 안 되면 SIGKILL."""
    if child.proc.returncode is not None:
        return
    try:
        child.proc.terminate()
    except ProcessLookupError:
        return
    try:
        await asyncio.wait_for(child.proc.wait(), timeout=timeout)
    except asyncio.TimeoutError:
        log.warning("[%s] SIGTERM 후 %.1fs 무응답 → SIGKILL", child.spec.label.strip(), timeout)
        try:
            child.proc.kill()
            await child.proc.wait()
        except ProcessLookupError:
            pass


# ===========================================================================
# Spec builders
# ===========================================================================

def _proxy_spec(args: argparse.Namespace) -> ChildSpec:
    argv = [
        sys.executable, str(PROXY),
        "--host", args.host,
        "--port", str(args.port),
        "--signal-port", str(args.signal_port),
        "--zmq-addr", args.zmq_addr,
        "--webroot", str(ROOT / "webxr"),
        "--log-level", args.log_level,
    ]
    for s in args.stun:
        argv += ["--stun", s]
    return ChildSpec(label="proxy ", argv=argv)


def _sim_bridge_spec(args: argparse.Namespace) -> ChildSpec:
    # mac_proxy 의 signaling 이 떠 있어야 bridge 의 ws_connect 가 붙는다.
    # localhost 으로 wait — same machine 이라 host 가 0.0.0.0 bind 여도 127.0.0.1 으로 접근 가능.
    argv = [
        sys.executable, str(BRIDGE),
        "--sim-host", args.sim_host,
        "--sim-port", str(args.sim_port),
        "--robot-id", str(args.sim_robot_id),
        "--topic", args.sim_topic,
        "--mac-url", f"wss://localhost:{args.signal_port}/signaling/server",
        "--fps", str(args.video_fps),
        "--log-level", args.log_level,
    ]
    return ChildSpec(
        label="video ",
        argv=argv,
        start_after_port=("127.0.0.1", args.signal_port),
    )


def _fake_producer_spec(args: argparse.Namespace) -> ChildSpec:
    argv = [
        sys.executable, str(FAKE),
        "--url", f"wss://localhost:{args.signal_port}/signaling/server",
        "--source", str(DEMO_MP4),
        "--log-level", args.log_level,
    ]
    return ChildSpec(
        label="video ",
        argv=argv,
        start_after_port=("127.0.0.1", args.signal_port),
    )


# ===========================================================================
# Top level
# ===========================================================================

async def run(args: argparse.Namespace) -> int:
    specs: List[ChildSpec] = [_proxy_spec(args)]
    if args.video_source == "sim":
        specs.append(_sim_bridge_spec(args))
    elif args.video_source == "fake":
        specs.append(_fake_producer_spec(args))
    elif args.video_source == "none":
        pass
    else:
        log.error("unknown video-source: %s", args.video_source)
        return 2

    # spawn all (start_after_port 가 있는 자식은 그 포트 LISTEN 후 spawn)
    children: List[Child] = []
    try:
        for spec in specs:
            log.info("→ spawn [%s]: %s", spec.label.strip(), " ".join(spec.argv[1:]))
            child = await _spawn(spec)
            children.append(child)
    except Exception as e:  # noqa: BLE001
        log.error("spawn failed: %s", e)
        for c in children:
            await _terminate(c)
        return 2

    log.info("all children up (count=%d). Ctrl+C 로 종료.", len(children))

    # signal handler + watcher (한쪽이 죽으면 stop 발동)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop.set)
        except NotImplementedError:
            pass  # windows

    async def _watcher() -> None:
        wait_tasks = [asyncio.create_task(c.proc.wait()) for c in children]
        done, _ = await asyncio.wait(wait_tasks, return_when=asyncio.FIRST_COMPLETED)
        for c in children:
            if c.proc.returncode is not None:
                log.warning("child died: [%s] rc=%d",
                            c.spec.label.strip(), c.proc.returncode)
        stop.set()

    watcher_task = asyncio.create_task(_watcher())

    try:
        await stop.wait()
    finally:
        log.info("shutting down children…")
        # 순서 역순으로 종료 (video → proxy) — bridge 가 먼저 끊겨야 proxy 가 깨끗이 닫힘.
        for child in reversed(children):
            await _terminate(child)
        for child in children:
            if child.log_task and not child.log_task.done():
                child.log_task.cancel()
        if not watcher_task.done():
            watcher_task.cancel()

    rcs = [c.proc.returncode for c in children]
    log.info("exit codes: %s", rcs)
    # 정상(0) 또는 SIGTERM 으로 죽인 것(-15) 만 OK 로 본다.
    return 0 if all(rc in (0, -15, None) for rc in rcs) else 1


def main() -> int:
    p = argparse.ArgumentParser(
        description="XLerobot WebXR 전체 스택 (mac_proxy + 비디오 publisher) 한 번에",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # ── 영상 소스 선택 ───────────────────────────────────────────────────
    p.add_argument(
        "--video-source",
        choices=["sim", "fake", "none"],
        default="sim",
        help=("sim: tools/sim_video_bridge.py (indoory_isaac_sim 의 rgb.front.<i>) "
              "/ fake: tools/fake_producer.py (demo.mp4 loop) "
              "/ none: 영상 없이 mac_proxy 만"),
    )
    # ── mac_proxy 인자 (그대로 통과) ─────────────────────────────────────
    g_proxy = p.add_argument_group("mac_proxy (passed-through)")
    g_proxy.add_argument("--host", default="0.0.0.0")
    g_proxy.add_argument("--port", type=int, default=8443)
    g_proxy.add_argument("--signal-port", type=int, default=8444)
    g_proxy.add_argument("--zmq-addr", default="tcp://0.0.0.0:7001")
    g_proxy.add_argument("--stun", action="append", default=[],
                         help="추가 STUN URL. LAN 내에서는 보통 불필요.")
    # ── sim_video_bridge 인자 (video-source=sim 일 때만 의미) ────────────
    g_sim = p.add_argument_group("sim_video_bridge (video-source=sim)")
    g_sim.add_argument("--sim-host", default="100.80.87.68")
    g_sim.add_argument("--sim-port", type=int, default=5555)
    g_sim.add_argument("--sim-robot-id", type=int, default=0)
    g_sim.add_argument("--sim-topic", default="rgb.front",
                       help="rgb.front | rgb.wrist")
    g_sim.add_argument("--video-fps", type=int, default=30)
    # ── 공통 ───────────────────────────────────────────────────────────
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()
    if args.video_fps <= 0:
        p.error("--video-fps must be a positive integer")

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )

    try:
        rc = asyncio.run(run(args))
    except KeyboardInterrupt:
        rc = 130
    return rc


if __name__ == "__main__":
    sys.exit(main())
