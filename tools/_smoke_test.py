"""
xlerobot-webxr/tools/_smoke_test.py

mac_proxy + fake_producer + 가짜 Quest 를 한 프로세스 안에서 동시에 띄워서
선언된 책임이 실제로 동작하는지 검증한다. 통과 기준:

  1. /ws 가 pose 페이로드를 받아 ZMQ PUB 으로 정확한 토픽(b"pose.7")으로 publish
  2. /signaling/server 가 fake_producer 의 offer 를 받아 answer 응답
  3. /signaling/quest 가 가짜 Quest 의 offer 를 받아 answer 응답
  4. fake_producer 의 video track 이 Mac proxy 를 거쳐 quest sender 로 replace 됨
  5. 30초 안에 모든 단계가 끝나고 ZMQ 메시지 카운트가 0 보다 큼

이 스크립트는 dev 용 — CI 에 넣기 전에 의존성 헤비함 (aiortc, av) 검토 필요.
"""

from __future__ import annotations

import asyncio
import copy
import json
import logging
import os
import pathlib
import ssl
import subprocess
import sys
import time

import aiohttp
import msgpack
import zmq
import zmq.asyncio
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp

ROOT = pathlib.Path(__file__).resolve().parent
PROXY = ROOT / "mac_proxy.py"
PRODUCER = ROOT / "fake_producer.py"
DEMO_MP4 = ROOT / "webxr" / "assets" / "demo.mp4"
POSE_FIXTURE = ROOT / "fixtures" / "page_pose_v1_1.json"

# 포트는 일반 기본값과 충돌 안 나도록 시프트
PROXY_PORT = 18443
SIGNAL_PORT = 18444
ZMQ_ADDR = "tcp://127.0.0.1:17001"

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
log = logging.getLogger("smoke")


def _ssl_unverified() -> ssl.SSLContext:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


async def _wait_port(host: str, port: int, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            r, w = await asyncio.open_connection(host, port)
            w.close(); await w.wait_closed()
            return
        except OSError:
            await asyncio.sleep(0.2)
    raise TimeoutError(f"{host}:{port} did not open in {timeout}s")


async def _zmq_subscriber(stop: asyncio.Event) -> dict:
    """ZMQ SUB on pose.7 — count messages, snapshot first one."""
    ctx = zmq.asyncio.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect(ZMQ_ADDR)
    sub.setsockopt(zmq.SUBSCRIBE, b"pose.7")
    stats = {"count": 0, "first": None, "last": None}
    while not stop.is_set():
        try:
            parts = await asyncio.wait_for(sub.recv_multipart(), timeout=0.5)
        except asyncio.TimeoutError:
            continue
        topic, body = parts
        msg = msgpack.unpackb(body, raw=False)
        stats["count"] += 1
        stats["last"] = msg
        if stats["first"] is None:
            stats["first"] = msg
    sub.close(linger=0)
    return stats


async def _fake_quest_pose(stop: asyncio.Event) -> int:
    """Quest 가 보내는 pose 90Hz 흐름을 흉내냄. select_robot=7 부터 보냄."""
    url = f"wss://127.0.0.1:{PROXY_PORT}/ws"
    ssl_ctx = _ssl_unverified()
    sent = 0
    with POSE_FIXTURE.open("r", encoding="utf-8") as f:
        fixture = json.load(f)
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(url, ssl=ssl_ctx) as ws:
            await ws.send_json({"select_robot": 7})
            t0 = time.monotonic()
            while not stop.is_set() and time.monotonic() - t0 < 4.0:
                payload = copy.deepcopy(fixture)
                payload["t"] = int((time.monotonic() - t0) * 1000)
                await ws.send_json(payload)
                sent += 1
                await asyncio.sleep(1 / 90)
    log.info("fake quest pose sent: %d", sent)
    return sent


async def _fake_quest_signaling(stop: asyncio.Event) -> dict:
    """가짜 Quest WebRTC consumer — /signaling/quest 에 붙어 video track 을 받는다."""
    result = {"track_seen": False, "rtp_frames": 0, "answer_received": False}
    url = f"wss://127.0.0.1:{SIGNAL_PORT}/signaling/quest"
    pc = RTCPeerConnection(RTCConfiguration())
    pc.addTransceiver("video", direction="recvonly")

    track_event = asyncio.Event()

    @pc.on("track")
    def _on_track(track):
        log.info("fake-quest: ontrack kind=%s", track.kind)
        result["track_seen"] = True
        track_event.set()

        async def _drain():
            try:
                while not stop.is_set():
                    try:
                        await asyncio.wait_for(track.recv(), timeout=0.5)
                        result["rtp_frames"] += 1
                    except asyncio.TimeoutError:
                        continue
                    except Exception:
                        break
            finally:
                log.info("fake-quest: track drain ended")

        asyncio.create_task(_drain())

    ssl_ctx = _ssl_unverified()
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(url, ssl=ssl_ctx) as ws:
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            # wait ICE
            done = asyncio.Event()
            @pc.on("icegatheringstatechange")
            def _gw():
                if pc.iceGatheringState == "complete":
                    done.set()
            try:
                await asyncio.wait_for(done.wait(), timeout=2.0)
            except asyncio.TimeoutError:
                pass
            await ws.send_json({"type": "offer", "sdp": pc.localDescription.sdp})
            log.info("fake-quest: offer sent")
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                data = json.loads(msg.data)
                if data.get("type") == "answer":
                    await pc.setRemoteDescription(
                        RTCSessionDescription(sdp=data["sdp"], type="answer")
                    )
                    result["answer_received"] = True
                    log.info("fake-quest: answer set")
                    break
            # wait until either track shows up or stop
            try:
                await asyncio.wait_for(track_event.wait(), timeout=10.0)
            except asyncio.TimeoutError:
                log.warning("fake-quest: no track within 10s")
            deadline = time.monotonic() + 10.0
            while (
                not stop.is_set()
                and result["rtp_frames"] < 5
                and time.monotonic() < deadline
            ):
                await asyncio.sleep(0.2)
    await pc.close()
    return result


async def main() -> int:
    if not DEMO_MP4.exists():
        log.error("demo.mp4 missing at %s", DEMO_MP4)
        return 1

    # Start mac_proxy as a subprocess
    log.info("starting mac_proxy")
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    proxy_proc = await asyncio.create_subprocess_exec(
        sys.executable, str(PROXY),
        "--host", "127.0.0.1",
        "--port", str(PROXY_PORT),
        "--signal-port", str(SIGNAL_PORT),
        "--zmq-addr", ZMQ_ADDR,
        "--log-level", "INFO",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
        env=env,
    )

    async def _stream_logs(proc, name):
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            sys.stdout.write(f"[{name}] " + line.decode(errors="replace"))

    asyncio.create_task(_stream_logs(proxy_proc, "proxy"))

    # Wait for ports
    try:
        await _wait_port("127.0.0.1", PROXY_PORT)
        await _wait_port("127.0.0.1", SIGNAL_PORT)
    except TimeoutError as e:
        log.error("port wait failed: %s", e)
        proxy_proc.terminate()
        return 2
    log.info("proxy is listening")
    # Give ZMQ a moment to bind too
    await asyncio.sleep(0.5)

    stop = asyncio.Event()
    zmq_task = asyncio.create_task(_zmq_subscriber(stop))
    quest_pose_task = asyncio.create_task(_fake_quest_pose(stop))
    quest_signal_task = asyncio.create_task(_fake_quest_signaling(stop))

    # Start fake_producer as another subprocess
    log.info("starting fake_producer")
    prod_proc = await asyncio.create_subprocess_exec(
        sys.executable, str(PRODUCER),
        "--url", f"wss://127.0.0.1:{SIGNAL_PORT}/signaling/server",
        "--source", str(DEMO_MP4),
        "--log-level", "INFO",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
        env=env,
    )
    asyncio.create_task(_stream_logs(prod_proc, "prod "))

    pose_count = await quest_pose_task
    try:
        sig_result = await asyncio.wait_for(quest_signal_task, timeout=30.0)
    except asyncio.TimeoutError:
        log.error("fake quest signaling timed out")
        sig_result = {"track_seen": False, "rtp_frames": 0, "answer_received": False}
    finally:
        stop.set()
        if not quest_signal_task.done():
            quest_signal_task.cancel()
    zmq_stats = await zmq_task

    # Shutdown
    proxy_proc.terminate()
    prod_proc.terminate()
    try:
        await asyncio.wait_for(proxy_proc.wait(), timeout=3.0)
    except asyncio.TimeoutError:
        proxy_proc.kill()
    try:
        await asyncio.wait_for(prod_proc.wait(), timeout=3.0)
    except asyncio.TimeoutError:
        prod_proc.kill()

    # ===========================================================================
    # Verdict
    # ===========================================================================
    print()
    print("====== SMOKE TEST RESULT ======")
    print(f"pose sent by fake quest         : {pose_count}")
    print(f"zmq messages on b'pose.7'       : {zmq_stats['count']}")
    print(f"  first robot_id                : {(zmq_stats['first'] or {}).get('robot_id')}")
    print(f"  first schema                  : {(zmq_stats['first'] or {}).get('schema')}")
    print(f"  first frame                   : {(zmq_stats['first'] or {}).get('frame')}")
    print(f"quest signaling answer received : {sig_result['answer_received']}")
    print(f"quest video track received      : {sig_result['track_seen']}")
    print(f"quest RTP frames drained        : {sig_result['rtp_frames']}")
    print("===============================")

    ok = (
        zmq_stats["count"] > 50
        and (zmq_stats["first"] or {}).get("robot_id") == 7
        and sig_result["answer_received"]
        and sig_result["track_seen"]
        and sig_result["rtp_frames"] > 0
        and (zmq_stats["first"] or {}).get("schema") == "xlerobot_v1.1.page"
        and (zmq_stats["first"] or {}).get("frame") == "local-floor"
        and (zmq_stats["first"] or {}).get("hmd") == [0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0]
        and (zmq_stats["first"] or {}).get("right", {}).get("buttons", {}).get("a") == 1
    )
    print("PASS" if ok else "FAIL")
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
