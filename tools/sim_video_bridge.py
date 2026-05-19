"""
xlerobot-webxr / tools/sim_video_bridge.py

indoory_isaac_sim 서버의 head camera (`rgb.front.<i>`) JPEG 스트림을 받아
aiortc VideoStreamTrack 으로 변환한 뒤 mac_proxy 의 /signaling/server WSS 에
WebRTC publisher 로 붙는다. Quest 페이지의 video plane 이 자동 swap.

구조 (fake_producer 와 거의 동일, source 만 다름):

    sim @ 100.x.y.z              this script              mac_proxy             page
    ─────────────────             ──────────              ─────────             ────
    ZMQ PUB :5555     ───────►   SUB rgb.front.<i>
                                  cv2.imdecode (BGR uint8)
                                  → av.VideoFrame
                                  → SimCameraTrack       ────► /signaling/server
                                                                 (selective fwd)  ────► video plane

페이로드 디코드는 스펙 §4.2.3 그대로:
    cv2.imdecode(np.frombuffer(msg["data"], np.uint8), cv2.IMREAD_COLOR)  → BGR
    av.VideoFrame.from_ndarray(img, format="bgr24")

rgb.front 의 디폴트 publish rate 는 10 Hz (sensors.yaml). SimCameraTrack.recv() 는
--fps 값에 맞춰 직접 pacing/PTS 를 계산한다. 30 fps 로 두면 같은 프레임이 약 3 회
듀플 송출되고, --fps 60 은 RTP timestamp 와 sleep interval 모두 60 fps 로 돈다.

CLI:
    python3 tools/sim_video_bridge.py                      # 100.80.87.68 / robot 0 / rgb.front
    python3 tools/sim_video_bridge.py --robot-id 1 --topic rgb.wrist
    python3 tools/sim_video_bridge.py --sim-host 100.80.87.68 \\
        --mac-url wss://localhost:8444/signaling/server

요구:
    python3 -m pip install pyzmq msgpack aiortc av aiohttp opencv-python numpy
"""

from __future__ import annotations

import argparse
import asyncio
import fractions
import json
import logging
import ssl
import sys
import time
from typing import Optional

import aiohttp
import cv2
import msgpack
import numpy as np
import zmq
import zmq.asyncio
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from av import VideoFrame

try:
    from aiortc.sdp import candidate_from_sdp
except ImportError:  # pragma: no cover
    from aiortc.rtcicetransport import candidate_from_sdp  # type: ignore

log = logging.getLogger("sim_video_bridge")


# ===========================================================================
# VideoStreamTrack subclass — sim JPEG ZMQ → av.VideoFrame
# ===========================================================================

class SimCameraTrack(VideoStreamTrack):
    """sim 의 rgb.<name>.<i> JPEG SUB 을 백그라운드 task 로 빨아당겨 항상
    최신 BGR ndarray 한 장을 보유. recv() 가 호출되면 그걸 av.VideoFrame 으로
    포장해서 돌려준다. 들어온 게 없으면 'waiting for sim frame' 더미 프레임.
    """

    def __init__(self, sub: zmq.asyncio.Socket, topic_b: bytes, fps: int = 30):
        super().__init__()
        self._sub = sub
        self._topic = topic_b
        self._fps = fps
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_recv_t = 0.0
        self._stats = {"recv": 0, "decoded": 0, "decode_err": 0,
                       "last_w": 0, "last_h": 0, "last_kb": 0.0}
        self._black = self._make_placeholder()
        # async recv 루프는 별도 task. close 시 cancel.
        self._receiver_task = asyncio.create_task(self._receiver())
        # aiortc VideoStreamTrack.next_timestamp() 는 기본 30fps cadence 라서,
        # --fps 를 실제 RTP pacing 에 반영하려고 recv() 에서 직접 clock 을 돈다.
        self._clock_rate = 90000
        self._time_base = fractions.Fraction(1, self._clock_rate)
        self._frame_period = 1.0 / max(1, fps)
        self._started_at: Optional[float] = None
        self._frame_index = 0

    @staticmethod
    def _make_placeholder() -> np.ndarray:
        """sim frame 이 아직 안 들어왔을 때 보여줄 1280x720 검은 프레임."""
        img = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.putText(
            img, "waiting for sim head camera...",
            (140, 360),
            cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 200, 100), 3, cv2.LINE_AA,
        )
        cv2.putText(
            img, "(check sim_video_bridge log for SUB stats)",
            (200, 440),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (180, 180, 180), 2, cv2.LINE_AA,
        )
        return img

    async def _receiver(self) -> None:
        """SUB recv 루프. JPEG 들어올 때마다 디코드해서 self._latest_bgr 에 캐시.

        스펙 §4.2.3 의 디코드 패턴 그대로:
            cv2.imdecode(np.frombuffer(msg["data"], np.uint8), cv2.IMREAD_COLOR)
        """
        while True:
            try:
                parts = await self._sub.recv_multipart()
            except asyncio.CancelledError:
                break
            except Exception as e:  # noqa: BLE001
                log.warning("sub recv error: %s", e)
                await asyncio.sleep(0.1)
                continue
            if len(parts) != 2:
                continue
            _topic, payload = parts
            self._stats["recv"] += 1
            try:
                msg = msgpack.unpackb(payload, raw=False)
            except Exception as e:  # noqa: BLE001
                log.warning("msgpack decode: %s", e)
                continue
            if msg.get("schema") != "xlerobot_v1":
                continue
            if msg.get("encoding") != "jpeg":
                continue
            jpeg = msg.get("data")
            if not jpeg:
                continue
            try:
                img = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
            except Exception as e:  # noqa: BLE001
                self._stats["decode_err"] += 1
                log.warning("cv2.imdecode: %s", e)
                continue
            if img is None:
                self._stats["decode_err"] += 1
                continue
            self._latest_bgr = img
            self._latest_recv_t = time.monotonic()
            self._stats["decoded"] += 1
            h, w = img.shape[:2]
            self._stats["last_w"], self._stats["last_h"] = w, h
            self._stats["last_kb"] = len(jpeg) / 1024.0

    async def recv(self) -> VideoFrame:
        if self._started_at is None:
            self._started_at = time.monotonic()
        target_t = self._started_at + self._frame_index * self._frame_period
        delay = target_t - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        pts = int(round(self._frame_index * self._clock_rate / max(1, self._fps)))
        self._frame_index += 1
        img = self._latest_bgr if self._latest_bgr is not None else self._black
        frame = VideoFrame.from_ndarray(img, format="bgr24")
        frame.pts = pts
        frame.time_base = self._time_base
        return frame

    def stop(self) -> None:
        self._receiver_task.cancel()


# ===========================================================================
# Signaling client (fake_producer 와 동일한 흐름)
# ===========================================================================

def _ssl_unverified() -> ssl.SSLContext:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


def _parse_candidate_message(data: dict):
    cand_str: str = data.get("candidate") or ""
    if not cand_str:
        return None
    stripped = cand_str
    if stripped.startswith("candidate:"):
        stripped = stripped[len("candidate:"):]
    try:
        cand = candidate_from_sdp(stripped)
    except Exception:
        cand = candidate_from_sdp(cand_str)
    cand.sdpMid = data.get("sdpMid")
    cand.sdpMLineIndex = data.get("sdpMLineIndex")
    return cand


async def _await_ice_complete(pc: RTCPeerConnection, timeout_s: float = 3.0) -> None:
    if pc.iceGatheringState == "complete":
        return
    done = asyncio.Event()

    @pc.on("icegatheringstatechange")
    def _on():
        if pc.iceGatheringState == "complete":
            done.set()

    try:
        await asyncio.wait_for(done.wait(), timeout=timeout_s)
    except asyncio.TimeoutError:
        log.warning("ICE gather timeout (%.1fs)", timeout_s)


async def _stats_logger(track: SimCameraTrack, period_s: float = 5.0) -> None:
    """5초 윈도 SUB / decode 통계 로깅."""
    last = dict(track._stats)
    last_t = time.monotonic()
    while True:
        try:
            await asyncio.sleep(period_s)
        except asyncio.CancelledError:
            break
        now = time.monotonic()
        dt = max(1e-9, now - last_t)
        d_recv = track._stats["recv"] - last["recv"]
        d_decoded = track._stats["decoded"] - last["decoded"]
        log.info(
            "sim rx: %5.1f Hz (recv) %5.1f Hz (decoded)  last=%dx%d  ~%.1f KB",
            d_recv / dt, d_decoded / dt,
            track._stats["last_w"], track._stats["last_h"], track._stats["last_kb"],
        )
        last = dict(track._stats)
        last_t = now


# ===========================================================================
# main
# ===========================================================================

async def run(
    sim_host: str,
    sim_port: int,
    robot_id: int,
    topic_base: str,
    mac_url: str,
    fps: int,
) -> None:
    # 1) ZMQ SUB from sim
    ctx = zmq.asyncio.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 8)
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(f"tcp://{sim_host}:{sim_port}")
    topic = f"{topic_base}.{robot_id}".encode()
    sub.setsockopt(zmq.SUBSCRIBE, topic)
    log.info("ZMQ SUB tcp://%s:%d  topic=%s", sim_host, sim_port, topic.decode())

    # 2) Track + stats logger
    track = SimCameraTrack(sub, topic, fps=fps)
    stats_task = asyncio.create_task(_stats_logger(track))

    # 3) RTCPeerConnection — Mac proxy 의 /signaling/server 가 answerer
    pc = RTCPeerConnection(RTCConfiguration())
    pc.addTrack(track)

    @pc.on("connectionstatechange")
    def _on_state():
        log.info("pc state=%s", pc.connectionState)

    # 4) Signaling
    ssl_ctx = _ssl_unverified()
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(mac_url, ssl=ssl_ctx, heartbeat=30) as ws:
            log.info("signaling connected: %s", mac_url)
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            await _await_ice_complete(pc)
            await ws.send_json({"type": "offer", "sdp": pc.localDescription.sdp})
            log.info("offer sent (%d bytes)", len(pc.localDescription.sdp))
            try:
                async for msg in ws:
                    if msg.type != aiohttp.WSMsgType.TEXT:
                        continue
                    data = json.loads(msg.data)
                    t = data.get("type")
                    if t == "answer":
                        await pc.setRemoteDescription(
                            RTCSessionDescription(sdp=data["sdp"], type="answer")
                        )
                        log.info("answer received & set (state=%s)", pc.connectionState)
                    elif t == "candidate" and data.get("candidate"):
                        cand = _parse_candidate_message(data)
                        if cand:
                            try:
                                await pc.addIceCandidate(cand)
                            except Exception as e:  # noqa: BLE001
                                log.warning("addIceCandidate failed: %s", e)
                    elif t == "bye":
                        break
            except aiohttp.WSServerHandshakeError as e:
                log.error("signaling handshake failed: %s", e)
            log.info("signaling stream ended")

    stats_task.cancel()
    track.stop()
    try:
        await pc.close()
    except Exception:
        pass
    sub.close(linger=0)


def main() -> int:
    p = argparse.ArgumentParser(
        description="indoory_isaac_sim head camera → mac_proxy → Quest page"
    )
    p.add_argument("--sim-host", default="100.80.87.68",
                   help="sim Tailscale/LAN IP (default: 100.80.87.68)")
    p.add_argument("--sim-port", type=int, default=5555,
                   help="sim PUB 포트 (스펙 §1)")
    p.add_argument("--robot-id", type=int, default=0)
    p.add_argument("--topic", default="rgb.front",
                   help="rgb.front | rgb.wrist (스펙 §4.2.3). default rgb.front")
    p.add_argument("--mac-url",
                   default="wss://localhost:8444/signaling/server",
                   help="mac_proxy signaling endpoint")
    p.add_argument("--fps", type=int, default=30,
                   help="WebRTC send framerate. sim publish 가 10Hz 라 30 이면 ~3 듀플")
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()
    if args.fps <= 0:
        p.error("--fps must be a positive integer")

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )
    logging.getLogger("aioice").setLevel(logging.WARNING)
    logging.getLogger("aiortc").setLevel(logging.WARNING)

    try:
        asyncio.run(run(
            sim_host=args.sim_host,
            sim_port=args.sim_port,
            robot_id=args.robot_id,
            topic_base=args.topic,
            mac_url=args.mac_url,
            fps=args.fps,
        ))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
