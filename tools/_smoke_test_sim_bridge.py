"""sim_video_bridge.py smoke test.

sim 서버가 없는 환경에서도 다음이 동작하는지 확인:
  - SUB 가 빈 큐로 살아있는 상태에서
  - mac_proxy 의 /signaling/server 에 offer/answer 가 완료되고
  - 가짜 Quest consumer 가 video track 을 받으면
  - (sim 미연결이므로) 'waiting for sim head camera...' 검은 프레임이 흐름
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import pathlib
import ssl
import sys
import time

import aiohttp
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

ROOT = pathlib.Path(__file__).resolve().parent
PROXY = ROOT / "mac_proxy.py"
BRIDGE = ROOT / "sim_video_bridge.py"

PROXY_PORT = 18443
SIGNAL_PORT = 18444
ZMQ_ADDR = "tcp://127.0.0.1:17001"

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
log = logging.getLogger("smoke-sim")


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


async def _fake_quest_consumer(stop: asyncio.Event) -> dict:
    result = {"track_seen": False, "rtp_frames": 0, "answer": False}
    url = f"wss://127.0.0.1:{SIGNAL_PORT}/signaling/quest"
    pc = RTCPeerConnection(RTCConfiguration())
    pc.addTransceiver("video", direction="recvonly")
    track_event = asyncio.Event()

    @pc.on("track")
    def _on_track(track):
        log.info("quest: ontrack kind=%s", track.kind)
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
                pass

        asyncio.create_task(_drain())

    ssl_ctx = _ssl_unverified()
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(url, ssl=ssl_ctx) as ws:
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
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
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                data = json.loads(msg.data)
                if data.get("type") == "answer":
                    await pc.setRemoteDescription(
                        RTCSessionDescription(sdp=data["sdp"], type="answer")
                    )
                    result["answer"] = True
                    break
            try:
                await asyncio.wait_for(track_event.wait(), timeout=10.0)
            except asyncio.TimeoutError:
                log.warning("quest: no track within 10s")
            deadline = time.monotonic() + 15.0
            while (
                not stop.is_set()
                and result["rtp_frames"] < 5
                and time.monotonic() < deadline
            ):
                await asyncio.sleep(0.2)
    await pc.close()
    return result


async def main() -> int:
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}

    async def _stream(proc, name):
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            sys.stdout.write(f"[{name}] " + line.decode(errors="replace"))

    log.info("start mac_proxy")
    proxy_proc = await asyncio.create_subprocess_exec(
        sys.executable, str(PROXY),
        "--host", "127.0.0.1",
        "--port", str(PROXY_PORT),
        "--signal-port", str(SIGNAL_PORT),
        "--zmq-addr", ZMQ_ADDR,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
        env=env,
    )
    asyncio.create_task(_stream(proxy_proc, "proxy "))

    try:
        await _wait_port("127.0.0.1", PROXY_PORT)
        await _wait_port("127.0.0.1", SIGNAL_PORT)
    except TimeoutError as e:
        log.error("ports not ready: %s", e)
        proxy_proc.terminate()
        return 2

    stop = asyncio.Event()
    quest_task = asyncio.create_task(_fake_quest_consumer(stop))

    log.info("start sim_video_bridge (pointing at non-existent sim — SUB will be empty)")
    bridge_proc = await asyncio.create_subprocess_exec(
        sys.executable, str(BRIDGE),
        "--sim-host", "127.0.0.1",
        "--sim-port", "9", "--robot-id", "0",   # 닫힌 포트 → SUB 가 들어오는 게 없음
        "--mac-url", f"wss://127.0.0.1:{SIGNAL_PORT}/signaling/server",
        "--fps", "30",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
        env=env,
    )
    asyncio.create_task(_stream(bridge_proc, "bridge"))

    try:
        res = await asyncio.wait_for(quest_task, timeout=35.0)
    except asyncio.TimeoutError:
        log.error("quest consumer timed out")
        res = {"track_seen": False, "rtp_frames": 0, "answer": False}
    finally:
        stop.set()
        if not quest_task.done():
            quest_task.cancel()

    proxy_proc.terminate()
    bridge_proc.terminate()
    for proc in [proxy_proc, bridge_proc]:
        try:
            await asyncio.wait_for(proc.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            proc.kill()

    print()
    print("====== SIM-BRIDGE SMOKE RESULT ======")
    print(f"answer received        : {res['answer']}")
    print(f"video track received   : {res['track_seen']}")
    print(f"RTP frames drained     : {res['rtp_frames']}")
    print("=====================================")
    ok = res["answer"] and res["track_seen"] and res["rtp_frames"] > 0
    print("PASS" if ok else "FAIL")
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
