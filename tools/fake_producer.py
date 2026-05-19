"""
xlerobot-webxr / tools/fake_producer.py

M3 검증용 가짜 Home Server. mac_proxy 의 /signaling/server WSS 에 client 로 붙어
demo.mp4 를 video track 으로 publish 한다 — Home Server WebRTC 통합(M8) 이 나오기
전까지 Mac proxy 의 selective forwarder + Quest 까지의 미디어 경로를 단독으로
검증할 수 있게 해준다.

검증 시나리오 (계획서 §8 M3):
  1) Mac:    python3 tools/mac_proxy.py
  2) Server: python3 tools/fake_producer.py            # 이 스크립트
  3) Quest:  https://<mac-lan-ip>:8443/?robot=0
     → 페이지의 video plane 에 demo.mp4 가 WebRTC 트랙으로 흘러와야 함
     (페이지가 WebRTC 트랙을 잡으면 demo.mp4 자체 재생을 그 트랙으로 교체)

실제 Home Server (M8) 가 올라오면 이 스크립트는 더 이상 필요 없다.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import pathlib
import ssl
import sys
import time

import aiohttp
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaPlayer

try:
    from aiortc.sdp import candidate_from_sdp
except ImportError:  # pragma: no cover
    from aiortc.rtcicetransport import candidate_from_sdp  # type: ignore

log = logging.getLogger("fake_producer")


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _unverified_ssl_context() -> ssl.SSLContext:
    """mac_proxy 의 자기서명 cert 를 받아들이기 위한 비검증 컨텍스트.

    프로덕션 Home Server 는 §10 의 Caddy + LE 인증서를 신뢰하면 되므로 이런
    완화는 불필요. fake_producer 는 dev 전용.
    """
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


# ---------------------------------------------------------------------------
# main flow
# ---------------------------------------------------------------------------

async def run(url: str, source: pathlib.Path) -> None:
    if not source.exists():
        log.error("source file not found: %s", source)
        sys.exit(2)
    log.info("source video: %s", source)

    # MediaPlayer 가 매번 demo.mp4 를 loop 으로 재생 (~10s loop).
    # transcode 는 aiortc 가 자동 처리 (libavcodec 사용).
    player = MediaPlayer(str(source), loop=True)
    if player.video is None:
        log.error("MediaPlayer 가 video track 을 못 만듦. ffmpeg/PyAV 설치 확인")
        sys.exit(2)

    pc = RTCPeerConnection(RTCConfiguration())
    pc.addTrack(player.video)

    @pc.on("connectionstatechange")
    def _on_state():
        log.info("pc state=%s", pc.connectionState)

    # aiohttp client → mac_proxy WSS
    ssl_ctx = _unverified_ssl_context()
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(url, ssl=ssl_ctx, heartbeat=30) as ws:
            log.info("signaling connected: %s", url)

            # 1) create offer + send
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            await _await_ice_complete(pc)
            await ws.send_json({"type": "offer", "sdp": pc.localDescription.sdp})
            log.info("offer sent (%d bytes)", len(pc.localDescription.sdp))

            # 2) await answer + trickle candidates from peer
            async for msg in ws:
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                data = json.loads(msg.data)
                t = data.get("type")
                if t == "answer":
                    answer = RTCSessionDescription(sdp=data["sdp"], type="answer")
                    await pc.setRemoteDescription(answer)
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

            log.info("signaling stream ended")
    try:
        await pc.close()
    except Exception:
        pass


def main() -> int:
    p = argparse.ArgumentParser(description="Fake Home Server video producer for M3 loopback")
    p.add_argument(
        "--url",
        default="wss://localhost:8444/signaling/server",
        help="mac_proxy signaling endpoint",
    )
    default_video = (
        pathlib.Path(__file__).resolve().parent / "webxr" / "assets" / "demo.mp4"
    )
    p.add_argument(
        "--source",
        default=str(default_video),
        help="video file to loop and publish",
    )
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )
    logging.getLogger("aioice").setLevel(logging.WARNING)
    logging.getLogger("aiortc").setLevel(logging.WARNING)

    try:
        asyncio.run(run(args.url, pathlib.Path(args.source).resolve()))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
