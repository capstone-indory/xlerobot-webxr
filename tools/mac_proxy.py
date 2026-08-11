"""
xlerobot-webxr / tools/mac_proxy.py

XLerobot WebXR 텔레옵 시스템의 Mac 쪽 데몬 — 계획서 §4.2 전체 구현.

이 한 프로세스가 담당하는 책임 (M3+):

  1. HTTPS 페이지 호스팅            :8443   (tools/webxr/index.html, assets/)
  2. WSS pose endpoint              :8443/ws
       → 페이지에서 90Hz JSON 수신 → robot_id 메타 부착 → ZMQ PUB 으로 forward
  3. WSS WebRTC signaling           :8444/signaling/{quest,server}
       → offer/answer/ICE 교환 (Quest = consumer, Home Server = producer)
  4. ZMQ PUB pose                   tcp://*:7001  (Tailscale 인터페이스에 publish)
       토픽: b"pose.<robot_id>"  페이로드: msgpack(dict)
  5. WebRTC selective forwarder     (aiortc.MediaRelay)
       Home Server 가 publish 한 video track 을 Quest 의 sendonly transceiver
       에 replaceTrack 으로 붙임. 단일 viewer 시나리오 (계획서 §4.2 "ICE
       relay + selective forwarder"). 본격 SFU 는 multi-driver 단계로 연기.
  6. ICE host candidate            aiortc 가 모든 인터페이스에서 자동 수집.
       기동 시 en0(LAN) + utun*(Tailscale) 둘 다 잡혔는지 확인하고 경고 출력.

서버 미연결 (Home Server WebRTC 트랙이 안 들어옴) 상태에서도 페이지는 동작하고,
demo.mp4 fallback 으로 video plane 이 계속 보인다. Home Server 가 늦게 붙어도
replaceTrack 으로 자동 교체 — 페이지 새로고침 불필요.

기동
----
    cd Indory/xlerobot-webxr
    pip install -r tools/requirements.txt
    python3 tools/mac_proxy.py

기본값
  --host          0.0.0.0
  --port          8443       페이지 + /ws pose
  --signal-port   8444       /signaling/{quest,server}
  --zmq-addr      tcp://0.0.0.0:7001   ZMQ PUB pose
  --webroot       tools/webxr          정적 자산 루트
  --stun          (없음, LAN 내 host candidate 로 충분)

운영 환경(§10) 에서는 Caddy + Let's Encrypt DNS-01 을 앞단에 두고
이 데몬은 :8443/:8444 를 localhost 로만 bind 한 뒤 reverse proxy 받아도 됨.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import pathlib
import ssl
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, Optional

import msgpack
import zmq
import zmq.asyncio
from aiohttp import WSMsgType, web

log = logging.getLogger("mac_proxy")

RTCConfiguration = None
RTCIceCandidate = None
RTCIceServer = None
RTCPeerConnection = None
RTCSessionDescription = None
MediaRelay = None
candidate_from_sdp = None
_WEBRTC_IMPORT_ERROR: Optional[BaseException] = None


def _load_webrtc_deps() -> None:
    """Import aiortc/PyAV only when WebRTC signaling is enabled.

    Pose-only VR teleop only needs HTTPS `/ws` plus ZMQ pose forwarding. Keeping
    WebRTC optional lets a Mac run controller teleop even when PyAV wheels are
    unavailable for its Python/macOS combination.
    """
    global RTCConfiguration
    global RTCIceCandidate
    global RTCIceServer
    global RTCPeerConnection
    global RTCSessionDescription
    global MediaRelay
    global candidate_from_sdp
    global _WEBRTC_IMPORT_ERROR

    if RTCPeerConnection is not None:
        return
    try:
        from aiortc import (  # type: ignore
            RTCConfiguration as _RTCConfiguration,
            RTCIceCandidate as _RTCIceCandidate,
            RTCIceServer as _RTCIceServer,
            RTCPeerConnection as _RTCPeerConnection,
            RTCSessionDescription as _RTCSessionDescription,
        )
        from aiortc.contrib.media import MediaRelay as _MediaRelay  # type: ignore

        try:
            from aiortc.sdp import candidate_from_sdp as _candidate_from_sdp  # type: ignore
        except ImportError:  # pragma: no cover
            from aiortc.rtcicetransport import (  # type: ignore
                candidate_from_sdp as _candidate_from_sdp,
            )
    except ImportError as exc:
        _WEBRTC_IMPORT_ERROR = exc
        raise RuntimeError(
            "WebRTC signaling requires aiortc/PyAV. Re-run with --pose-only "
            "for controller teleop without video, or install tools/requirements.txt."
        ) from exc

    RTCConfiguration = _RTCConfiguration
    RTCIceCandidate = _RTCIceCandidate
    RTCIceServer = _RTCIceServer
    RTCPeerConnection = _RTCPeerConnection
    RTCSessionDescription = _RTCSessionDescription
    MediaRelay = _MediaRelay
    candidate_from_sdp = _candidate_from_sdp


# ===========================================================================
# Config & cert
# ===========================================================================

@dataclass
class Config:
    host: str = "0.0.0.0"
    port: int = 8443
    signal_port: int = 8444
    zmq_addr: str = "tcp://0.0.0.0:7001"
    webroot: pathlib.Path = pathlib.Path(__file__).resolve().parent / "webxr"
    cert_dir: pathlib.Path = pathlib.Path(__file__).resolve().parent / "webxr"
    stun_urls: list = field(default_factory=list)
    pose_only: bool = False
    # 기동 시 categorize_interfaces() 결과를 담아두는 자리.
    # _run() 의 접속 URL 배너에서 사용.
    detected_ifaces: Dict[str, list] = field(default_factory=dict)


def ensure_dev_cert(cert_dir: pathlib.Path) -> tuple[pathlib.Path, pathlib.Path]:
    """tools/webxr/dev-cert.pem 과 dev-key.pem 이 없으면 한 번 생성한다.

    serve.py 와 같은 파일을 공유 — M0b 단계에서 이미 생성됐다면 재사용.
    """
    cert = cert_dir / "dev-cert.pem"
    key = cert_dir / "dev-key.pem"
    if cert.exists() and key.exists():
        return cert, key
    cert_dir.mkdir(parents=True, exist_ok=True)
    log.info("generating self-signed cert at %s", cert)
    subprocess.run(
        [
            "openssl", "req", "-x509", "-newkey", "rsa:2048", "-sha256",
            "-days", "365", "-nodes",
            "-keyout", str(key), "-out", str(cert),
            "-subj", "/CN=xlerobot-webxr-dev",
            "-addext", "subjectAltName=DNS:localhost,IP:127.0.0.1",
        ],
        check=True,
    )
    return cert, key


def make_ssl_context(cert: pathlib.Path, key: pathlib.Path) -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=str(cert), keyfile=str(key))
    return ctx


# ===========================================================================
# Interface enumeration  — sanity check that Tailscale utun is up
# ===========================================================================

def enumerate_interfaces() -> Dict[str, list]:
    """psutil 이 있으면 인터페이스 목록을 dict 로 돌려준다. 없으면 빈 dict.

    aiortc 가 ICE candidate gather 시 어떤 인터페이스를 사용하는지 직접 제어할
    수는 없지만 (시스템 라우팅 따라감), 기동 시 사용자에게 "Tailscale 인터페이스
    가 안 잡혀 있으면 WebRTC 가 Home Server 까지 못 닿음" 을 미리 경고해주기 위함.
    """
    try:
        import psutil
    except ImportError:
        return {}
    out: Dict[str, list] = {}
    for name, addrs in psutil.net_if_addrs().items():
        ipv4 = [a.address for a in addrs if a.family.name == "AF_INET"]
        if ipv4:
            out[name] = ipv4
    return out


def _outbound_ip_fallback() -> Optional[str]:
    """psutil 미설치 시 — UDP 소켓 connect 트릭으로 기본 outbound 인터페이스 IP 추정.

    실제 패킷은 안 나간다 (connect 만 함). DNS 조회 없이 OS 라우팅 테이블에서
    8.8.8.8 으로 갈 때 어느 로컬 IP 를 쓸지 골라준다 — 보통 LAN.
    """
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.settimeout(0.5)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        return ip if ip and ip != "0.0.0.0" else None
    except OSError:
        return None
    finally:
        s.close()


def _filter_routable(ips: list) -> list:
    """169.254.* (APIPA), 127.* (loopback) 제거 — Quest/Server 가 닿을 수 없는 IP."""
    return [ip for ip in ips if not (ip.startswith("169.254.") or ip.startswith("127."))]


def categorize_interfaces() -> Dict[str, list]:
    """{ 'lan': [...], 'tailscale': [...], 'loopback': [...], 'other': [...] } 형태로 반환."""
    ifaces = enumerate_interfaces()
    cat: Dict[str, list] = {"lan": [], "tailscale": [], "loopback": [], "other": []}
    if not ifaces:
        # psutil 없으면 outbound IP 추정만
        fallback = _outbound_ip_fallback()
        if fallback:
            cat["lan"].append(("(unknown)", fallback))
        return cat
    for name, ips in sorted(ifaces.items()):
        for ip in _filter_routable(ips):
            if name.startswith(("en", "eth", "wlan", "wlp", "enp")):
                cat["lan"].append((name, ip))
            elif name.startswith(("utun", "tailscale")):
                cat["tailscale"].append((name, ip))
            elif name == "lo" or name.startswith("lo"):
                cat["loopback"].append((name, ip))
            else:
                cat["other"].append((name, ip))
    return cat


def check_interfaces() -> Dict[str, list]:
    """카테고리 결과를 돌려주고 동시에 사용자에게 보여준다."""
    cat = categorize_interfaces()
    if not any(cat.values()):
        log.warning("psutil unavailable + outbound IP 추정도 실패 — 인터페이스 진단 스킵")
        log.warning("권장: python3 -m pip install psutil")
        return cat
    log.info("network interfaces (routable ipv4):")
    for kind in ("lan", "tailscale", "other"):
        for name, ip in cat[kind]:
            tag = {"lan": "LAN", "tailscale": "Tailscale", "other": "?"}[kind]
            log.info("  [%s]  %-12s  %s", tag, name, ip)
    if not cat["lan"]:
        log.warning("LAN 인터페이스 미감지 — Quest 가 Mac 에 못 붙을 수 있음")
    if not cat["tailscale"]:
        log.warning(
            "Tailscale 인터페이스(utun*) 미감지 — Home Server 측이 Mac 의 utun host "
            "candidate 로 못 붙음. 'sudo tailscale up' 확인 필요 (§6.2 위험: DERP fallback)"
        )
    return cat


# ===========================================================================
# ZMQ pose publisher
# ===========================================================================

class ZMQPosePublisher:
    """WS 로 들어온 pose 페이로드를 ZMQ PUB 으로 forward.

    토픽 prefix 가 b"pose.<robot_id>" 인 이유: 계획서 §10 "VR bridge instance 가
    --robot-id 2 로 시작 → 해당 robot 만 SUB". subscribe(b"pose.2") 로 정확
    필터링 가능.

    페이로드 직렬화는 msgpack. 계획서 §3 의 wire schema 와 동일한 dict 모양:
      {schema, stamp_ns, robot_id, frame, hmd, left, right, estop, t_page_ms}

    참고: 페이지가 보내는 raw payload (§4.1) 는 robot/sim 좌표계로 아직 변환
    되어 있지 않다. 그 변환은 VR bridge 책임 (§4.3, §4.4). 이 publisher 는
    page payload + robot_id 메타만 붙여 그대로 publish 한다.
    """

    def __init__(self, addr: str):
        self.addr = addr
        self.ctx = zmq.asyncio.Context.instance()
        self.sock: Optional[zmq.asyncio.Socket] = None
        self.send_drops = 0

    async def start(self) -> None:
        self.sock = self.ctx.socket(zmq.PUB)
        # Controller pose is a live measurement. If a subscriber falls behind,
        # drop stale poses instead of queueing old controller positions.
        self.sock.setsockopt(zmq.SNDHWM, 1)
        self.sock.setsockopt(zmq.SNDTIMEO, 0)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(self.addr)
        log.info("zmq PUB bound: %s", self.addr)
        # PUB-SUB slow-joiner: subscriber 가 늦게 붙으면 초기 메시지를 놓침.
        # 다행히 VR bridge 는 라이브 stream 만 보면 되므로 OK.

    async def close(self) -> None:
        if self.sock:
            self.sock.close(linger=0)
            self.sock = None

    async def publish(self, robot_id: int, payload: dict) -> None:
        if not self.sock:
            return
        topic = f"pose.{robot_id}".encode()
        # bridge 가 stamp_ns 로 round-trip 측정한다 (§10 latency monitoring).
        # 페이지가 보낸 t (ms) 를 그대로 남기고, 추가로 mac 도착 시각도 기록.
        body = {
            "schema": "xlerobot_v1.1.page",   # bridge 가 v1.1 변환할 입력 표시
            "stamp_ns": time.monotonic_ns(),
            "robot_id": robot_id,
            "frame": payload.get("frame", "local-floor"),
            "t_page_ms": payload.get("t"),
            "hmd": payload.get("hmd"),
            "left": payload.get("left"),
            "right": payload.get("right"),
            "estop": bool(payload.get("estop", False)),
        }
        try:
            await self.sock.send_multipart(
                [topic, msgpack.packb(body, use_bin_type=True)],
                flags=zmq.NOBLOCK,
            )
        except zmq.Again:
            self.send_drops += 1
        except zmq.ZMQError as e:
            log.warning("zmq publish failed: %s", e)


# ===========================================================================
# Pose WebSocket — page → mac → ZMQ
# ===========================================================================

DEFAULT_ROBOT_ID = 0


class PoseWSHandler:
    """단일 페이지의 /ws 연결을 처리.

    페이지가 보내는 메시지 형식:
      첫 메시지 (옵션):    {"select_robot": <int>}
      이후 매 frame (90Hz): {"t":<ms>, "hmd":[...], "left":{...}, "right":{...}, "estop":bool}

    URL 쿼리 ?robot=N 도 같이 받는다 (둘 다 오면 select_robot 우선).
    """

    def __init__(self, publisher: ZMQPosePublisher):
        self.publisher = publisher

    async def handle(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=30, max_msg_size=64 * 1024)
        await ws.prepare(request)
        peer = request.transport.get_extra_info("peername")
        client_id = uuid.uuid4().hex[:8]
        # URL 쿼리에서 robot_id 후보를 미리 읽어둠. select_robot 메시지가 오면 덮어쓴다.
        try:
            robot_id = int(request.query.get("robot", DEFAULT_ROBOT_ID))
        except ValueError:
            robot_id = DEFAULT_ROBOT_ID
        if robot_id < 0:
            robot_id = DEFAULT_ROBOT_ID
        log.info("pose ws[%s] connect from %s, robot_id=%d (initial)", client_id, peer, robot_id)

        last_log_t = 0.0
        msg_count = 0
        estop_state_logged = False

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        payload = json.loads(msg.data)
                    except json.JSONDecodeError:
                        log.warning("pose ws[%s] non-json: %r", client_id, msg.data[:80])
                        continue
                    if "select_robot" in payload:
                        try:
                            new_id = int(payload["select_robot"])
                        except (TypeError, ValueError):
                            continue
                        if new_id < 0:
                            continue
                        if new_id != robot_id:
                            log.info("pose ws[%s] robot_id %d -> %d", client_id, robot_id, new_id)
                            robot_id = new_id
                        continue
                    msg_count += 1
                    estop = bool(payload.get("estop", False))
                    if estop != estop_state_logged:
                        log.info("pose ws[%s] estop %s (robot=%d)", client_id, estop, robot_id)
                        estop_state_logged = estop
                    # publish to ZMQ
                    await self.publisher.publish(robot_id, payload)
                    # rate-limited heartbeat log
                    now = time.monotonic()
                    if now - last_log_t >= 5.0:
                        rate = msg_count / max(1e-9, (now - last_log_t))
                        log.info("pose ws[%s] %.1f Hz (robot=%d)", client_id, rate, robot_id)
                        last_log_t = now
                        msg_count = 0
                elif msg.type == WSMsgType.ERROR:
                    log.warning("pose ws[%s] error: %s", client_id, ws.exception())
        finally:
            log.info("pose ws[%s] disconnect", client_id)
        return ws


# ===========================================================================
# WebRTC selective forwarder
# ===========================================================================

class MediaHub:
    """Quest 쪽과 Home Server 쪽 RTCPeerConnection 을 모두 들고 다니는 매개체.

    단일 viewer (Quest 1대) + 단일 producer (Home Server 1대) 시나리오.
    Multi-driver / multi-camera 는 MVP 외 (계획서 §4.2 비고).

    동작:
      - Quest signaling 핸들러가 새 RTCPeerConnection 을 만들고 video sendonly
        transceiver 를 추가 → 빈 sender 가 됨. 페이지의 receive 는 일단 무 track.
      - Home Server signaling 핸들러가 또 다른 RTCPeerConnection 을 만들고
        on('track') 이벤트로 들어오는 비디오를 받는다.
      - 첫 server track 이 들어오는 순간 MediaRelay 로 감싸서 active quest
        sender 들에 replaceTrack 한다.
      - Quest 가 나중에 붙어도 그 시점 active 한 server track 으로 replaceTrack.
    """

    def __init__(self):
        _load_webrtc_deps()
        self._relay = MediaRelay()
        self._server_track = None  # 가장 최근 들어온 raw track (relay 의 source)
        self._quest_senders: Dict[str, "object"] = {}  # session_id -> RTCRtpSender

    def register_quest_sender(self, session_id: str, sender) -> None:
        self._quest_senders[session_id] = sender
        if self._server_track is not None:
            relayed = self._relay.subscribe(self._server_track)
            asyncio.create_task(self._safe_replace(sender, relayed,
                                                  f"quest[{session_id}] ← server (cached)"))

    def unregister_quest(self, session_id: str) -> None:
        self._quest_senders.pop(session_id, None)

    def on_server_track(self, track) -> None:
        """Home Server peer 에서 새 track 이 들어왔을 때 호출됨."""
        log.info("media: server track received: %s kind=%s", track.id, track.kind)
        self._server_track = track
        # 이미 연결된 quest sender 들에 즉시 replace
        for sid, sender in list(self._quest_senders.items()):
            relayed = self._relay.subscribe(track)
            asyncio.create_task(self._safe_replace(sender, relayed,
                                                  f"quest[{sid}] ← server (live)"))

    def on_server_track_ended(self) -> None:
        log.info("media: server track ended")
        self._server_track = None
        for sender in self._quest_senders.values():
            asyncio.create_task(self._safe_replace(sender, None, "quest ← (no track)"))

    @staticmethod
    async def _safe_replace(sender, track, label: str) -> None:
        # aiortc 의 RTCRtpSender.replaceTrack 은 버전마다 sync/async 가 갈린다.
        # 1.14.x: sync def (returns None)
        # 일부 fork / 미래 버전: async def (returns coroutine)
        # 둘 다 받기 위해 호출 결과가 awaitable 이면 await, 아니면 무시.
        try:
            result = sender.replaceTrack(track)
            if asyncio.iscoroutine(result) or asyncio.isfuture(result):
                await result
            log.info("media: replaceTrack ok — %s", label)
        except Exception as e:  # noqa: BLE001
            log.warning("media: replaceTrack fail (%s): %s", label, e)


# ===========================================================================
# Signaling protocol over WebSocket
# ===========================================================================
#
# Both /signaling/quest and /signaling/server use the same wire protocol:
#   client → server: {"type":"offer"|"answer", "sdp": "..."}
#                    {"type":"candidate", "candidate":"a=candidate:...", "sdpMid":"0", "sdpMLineIndex":0}
#                    {"type":"candidate", "candidate":null}       # end-of-candidates
#                    {"type":"bye"}
#   server → client: same shapes
#
# Mac (this proxy) is the answerer in both directions (Quest browser sends
# offer for receive-only; Home Server sends offer for send-only). Mac gathers
# all ICE candidates from all interfaces (aiortc default) and bakes them into
# the answer SDP (non-trickle outbound). Inbound candidates from the remote
# can be trickled (we accept them anytime).
#
# Why non-trickle outbound: aiortc 의 ICE 는 setLocalDescription 호출 이후에
# 비동기로 candidate 를 gather 한다. 'icegatheringstatechange == "complete"'
# 를 기다린 뒤 localDescription.sdp 를 보내면 모든 candidate 가 들어 있다.
# trickle 송신은 코드 추가 부담 대비 이득이 거의 없음 (LAN 환경).


def _build_pc(stun_urls: list) -> RTCPeerConnection:
    ice_servers = [RTCIceServer(urls=u) for u in stun_urls] if stun_urls else []
    cfg = RTCConfiguration(iceServers=ice_servers) if ice_servers else RTCConfiguration()
    return RTCPeerConnection(configuration=cfg)


def _sdp_candidate_count(sdp: Optional[str]) -> int:
    if not sdp:
        return 0
    return sum(1 for line in sdp.splitlines() if line.startswith("a=candidate:"))


async def _await_ice_complete(pc: RTCPeerConnection, timeout_s: float = 3.0) -> None:
    """ICE gathering 완료를 기다린다. 타임아웃 후엔 갖고 있는 후보만으로 진행."""
    if pc.iceGatheringState == "complete":
        return
    done = asyncio.Event()

    @pc.on("icegatheringstatechange")
    def _on_change():
        if pc.iceGatheringState == "complete":
            done.set()

    try:
        await asyncio.wait_for(done.wait(), timeout=timeout_s)
    except asyncio.TimeoutError:
        candidate_count = _sdp_candidate_count(
            pc.localDescription.sdp if pc.localDescription else None
        )
        log.warning(
            "ICE gathering timeout (%.1fs) — state=%s candidates=%d",
            timeout_s, pc.iceGatheringState, candidate_count,
        )


def _parse_candidate_message(data: dict):
    """browser-style {candidate, sdpMid, sdpMLineIndex} → aiortc RTCIceCandidate.

    aiortc 의 candidate_from_sdp 는 'candidate:...' 의 prefix 가 있거나 없거나
    둘 다 받아주는 버전도 있고 strict 한 버전도 있어서 양쪽 모두 시도한다.
    """
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


async def _signaling_loop(
    ws: web.WebSocketResponse,
    pc: RTCPeerConnection,
    label: str,
    on_close=None,
) -> None:
    """ws ↔ pc 의 시그널링을 한 사이클 진행하고 ws 가 닫힐 때까지 ICE 를 중계."""
    log.info("signaling[%s] start", label)
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                log.warning("signaling[%s] non-json", label)
                continue
            t = data.get("type")
            if t == "offer":
                offer = RTCSessionDescription(sdp=data["sdp"], type="offer")
                await pc.setRemoteDescription(offer)
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                await _await_ice_complete(pc)
                await ws.send_json({
                    "type": "answer",
                    "sdp": pc.localDescription.sdp,
                })
                log.info("signaling[%s] answer sent (state=%s)",
                         label, pc.connectionState)
            elif t == "candidate" and data.get("candidate"):
                cand = _parse_candidate_message(data)
                if cand:
                    try:
                        await pc.addIceCandidate(cand)
                    except Exception as e:  # noqa: BLE001
                        log.warning("signaling[%s] addIceCandidate failed: %s", label, e)
            elif t == "bye":
                break
            else:
                log.debug("signaling[%s] unknown msg: %s", label, t)
    finally:
        log.info("signaling[%s] end (state=%s)", label, pc.connectionState)
        try:
            await pc.close()
        except Exception:
            pass
        if on_close:
            try:
                on_close()
            except Exception:
                pass


class QuestSignalingHandler:
    """Quest 페이지 (consumer) 와의 시그널링."""

    def __init__(self, hub: MediaHub, stun_urls: list):
        self.hub = hub
        self.stun_urls = stun_urls

    async def handle(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        session_id = uuid.uuid4().hex[:8]
        pc = _build_pc(self.stun_urls)
        # send-only video transceiver — track 은 server peer 가 붙으면 replace
        transceiver = pc.addTransceiver("video", direction="sendonly")
        self.hub.register_quest_sender(session_id, transceiver.sender)

        @pc.on("connectionstatechange")
        def _on_state():
            log.info("quest[%s] pc state=%s", session_id, pc.connectionState)

        await _signaling_loop(
            ws, pc, f"quest:{session_id}",
            on_close=lambda: self.hub.unregister_quest(session_id),
        )
        return ws


class ServerSignalingHandler:
    """Home Server (producer) 와의 시그널링."""

    def __init__(self, hub: MediaHub, stun_urls: list):
        self.hub = hub
        self.stun_urls = stun_urls

    async def handle(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        session_id = uuid.uuid4().hex[:8]
        pc = _build_pc(self.stun_urls)

        @pc.on("track")
        def _on_track(track):
            log.info("server[%s] on_track kind=%s id=%s", session_id, track.kind, track.id)
            if track.kind == "video":
                self.hub.on_server_track(track)

                @track.on("ended")
                def _on_ended():
                    log.info("server[%s] track ended", session_id)
                    self.hub.on_server_track_ended()

        @pc.on("connectionstatechange")
        def _on_state():
            log.info("server[%s] pc state=%s", session_id, pc.connectionState)

        await _signaling_loop(ws, pc, f"server:{session_id}")
        return ws


# ===========================================================================
# Aiohttp app wiring
# ===========================================================================

async def _index_handler(request: web.Request) -> web.FileResponse:
    webroot: pathlib.Path = request.app["webroot"]
    return web.FileResponse(webroot / "index.html")


def make_page_app(webroot: pathlib.Path, pose_handler: PoseWSHandler) -> web.Application:
    app = web.Application()
    app["webroot"] = webroot
    app.router.add_get("/", _index_handler)
    app.router.add_get("/ws", pose_handler.handle)
    app.router.add_static("/assets", path=str(webroot / "assets"), show_index=False)
    # 정적 자산을 더 노출하고 싶을 때 대비
    app.router.add_static("/static", path=str(webroot), show_index=False)
    return app


async def _signaling_index(request: web.Request) -> web.Response:
    """`:8444` root 에 stub HTML 을 둔다.

    이유: Quest Browser 의 cert 예외는 origin(host:port) 단위라서, 페이지가
    :8443 에서 동작하는 데 cert 가 accept 됐어도 :8444 의 WSS 는 별도 origin
    이라 따로 accept 해야 한다. WebSocket 핸드셰이크 자체는 cert 경고 UI 를
    띄우지 못해서, 사용자가 미리 https://<host>:8444/ 를 방문해 cert 예외를
    처리해 둬야 시그널링이 붙는다. 이 stub 이 그 방문 페이지.
    """
    return web.Response(
        text=(
            "<!doctype html><meta charset='utf-8'>"
            "<title>xlerobot-webxr signaling :8444</title>"
            "<style>body{font-family:system-ui,sans-serif;padding:32px;max-width:520px;"
            "background:#0a0e14;color:#e6e6e6;line-height:1.6}"
            "code{background:#1e293b;padding:2px 6px;border-radius:4px}"
            "h1{color:#67e8f9}</style>"
            "<h1>xlerobot-webxr · signaling endpoint</h1>"
            "<p>이 페이지가 보이면 <code>:8444</code> 의 자기서명 cert 가 정상적으로 "
            "이 브라우저에 등록되었습니다. <strong>이 탭은 닫고</strong> 메인 페이지 "
            "<code>https://&lt;host&gt;:8443/</code> 으로 돌아가 WebXR 텔레옵을 시작하세요.</p>"
            "<p style='color:#94a3b8'>이 포트는 WSS 시그널링(<code>/signaling/quest</code>, "
            "<code>/signaling/server</code>)만 서빙합니다. WebSocket 핸드셰이크 자체는 "
            "cert 경고 UI 를 띄울 수 없어서, 처음 1회 이 root 페이지를 방문해 cert 예외를 "
            "걸어두는 단계가 필요합니다 — 이 페이지의 유일한 존재 이유입니다.</p>"
        ),
        content_type="text/html",
    )


def make_signaling_app(hub: MediaHub, stun_urls: list) -> web.Application:
    _load_webrtc_deps()
    app = web.Application()
    q = QuestSignalingHandler(hub, stun_urls)
    s = ServerSignalingHandler(hub, stun_urls)
    app.router.add_get("/", _signaling_index)
    app.router.add_get("/signaling/quest", q.handle)
    app.router.add_get("/signaling/server", s.handle)
    return app


# ===========================================================================
# main
# ===========================================================================

async def _run(cfg: Config) -> None:
    cert, key = ensure_dev_cert(cfg.cert_dir)
    ssl_ctx = make_ssl_context(cert, key)

    publisher = ZMQPosePublisher(cfg.zmq_addr)
    await publisher.start()

    pose_handler = PoseWSHandler(publisher)

    page_app = make_page_app(cfg.webroot, pose_handler)

    runners = []
    apps = [(page_app, cfg.port, "page+pose")]
    if cfg.pose_only:
        log.info("pose-only mode: WebRTC signaling/video disabled")
    else:
        hub = MediaHub()
        sig_app = make_signaling_app(hub, cfg.stun_urls)
        apps.append((sig_app, cfg.signal_port, "signaling"))

    for app, port, label in apps:
        runner = web.AppRunner(app, access_log=None)
        await runner.setup()
        site = web.TCPSite(runner, cfg.host, port, ssl_context=ssl_ctx)
        await site.start()
        log.info("listening: https://%s:%d  (%s)", cfg.host, port, label)
        runners.append(runner)

    # ───── 실제 IP 를 박은 접속 URL 출력 ──────────────────────────────────────
    # check_interfaces() 결과를 cfg.detected_ifaces 로 받아두고 여기서 출력.
    lan_ips = [ip for _, ip in cfg.detected_ifaces.get("lan", [])]
    ts_ips  = [ip for _, ip in cfg.detected_ifaces.get("tailscale", [])]
    log.info("============================================================")
    log.info("  Quest 3 에서 접속할 URL  (LAN — Wi-Fi 같은 공유기)")
    if lan_ips:
        for ip in lan_ips:
            if cfg.pose_only:
                log.info("    →  https://%s:%d/?robot=0   ← 메인 페이지", ip, cfg.port)
            else:
                log.info("    1) https://%s:%d/   ← cert 예외 1회 수락 (signaling 포트)",
                         ip, cfg.signal_port)
                log.info("    2) https://%s:%d/?robot=0   ← 메인 페이지", ip, cfg.port)
    else:
        log.info("    →  (LAN IP 미감지 — Wi-Fi/이더넷 연결 확인 필요)")
    log.info("")
    if cfg.pose_only:
        log.info("  Home Server WebRTC signaling")
        log.info("    →  disabled (--pose-only)")
    else:
        log.info("  Home Server 가 접속할 URL  (Tailscale)")
        if ts_ips:
            for ip in ts_ips:
                log.info("    →  wss://%s:%d/signaling/server", ip, cfg.signal_port)
        else:
            log.info("    →  (Tailscale IP 미감지 — 'sudo tailscale up')")
    log.info("")
    log.info("  ZMQ pose PUB")
    log.info("    →  %s   topic=b'pose.<robot_id>'", cfg.zmq_addr)
    log.info("============================================================")

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    import signal as _signal
    for s in (_signal.SIGINT, _signal.SIGTERM):
        try:
            loop.add_signal_handler(s, stop.set)
        except NotImplementedError:
            pass  # windows
    try:
        await stop.wait()
    finally:
        log.info("shutting down")
        for runner in runners:
            await runner.cleanup()
        await publisher.close()


def main() -> int:
    p = argparse.ArgumentParser(
        description="XLerobot WebXR Mac proxy daemon (계획서 §4.2)"
    )
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8443, help="HTTPS port for page+pose WS")
    p.add_argument("--signal-port", type=int, default=8444, help="HTTPS port for WebRTC signaling")
    p.add_argument("--zmq-addr", default="tcp://0.0.0.0:7001")
    p.add_argument("--webroot",
                   default=str(pathlib.Path(__file__).resolve().parent / "webxr"),
                   help="static webroot (where index.html lives)")
    p.add_argument("--stun", action="append", default=[],
                   help="STUN URL(s), e.g. stun:stun.l.google.com:19302. LAN 내라 보통 불필요.")
    p.add_argument(
        "--pose-only",
        action="store_true",
        help="serve page+/ws and ZMQ pose only; skip aiortc/WebRTC video signaling",
    )
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )
    logging.getLogger("aiortc").setLevel(logging.WARNING)
    logging.getLogger("aioice").setLevel(logging.WARNING)

    # 환경 미스매치를 즉시 식별할 수 있도록 인터프리터 경로를 첫 줄에 명시.
    # 'pip install X' 가 성공한 env 와 'python3 mac_proxy.py' 의 env 가 갈리는
    # 사고가 흔한데, 다음 줄을 비교하면 한 번에 진단됨.
    log.info("python   : %s", sys.executable)
    log.info("version  : %s", sys.version.split()[0])

    cfg = Config(
        host=args.host,
        port=args.port,
        signal_port=args.signal_port,
        zmq_addr=args.zmq_addr,
        webroot=pathlib.Path(args.webroot).resolve(),
        cert_dir=pathlib.Path(args.webroot).resolve(),
        stun_urls=args.stun,
        pose_only=args.pose_only,
    )

    cfg.detected_ifaces = check_interfaces()

    try:
        asyncio.run(_run(cfg))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
