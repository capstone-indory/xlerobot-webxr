"""
xlerobot-webxr / tools/webxr/serve.py

M0b 단계용 최소 HTTPS 정적 호스팅 서버.

목적
----
Quest 브라우저는 WebXR `immersive-vr` 세션을 secure context(HTTPS)에서만 허용한다.
계획서 §10 "HTTPS 인증서"는 운영용으로 Caddy + Let's Encrypt DNS-01을 권장하지만,
M0b 단계의 hello-world 검증에는 LAN 안에서 도는 자기서명 인증서면 충분하다.

이 스크립트는:
  1) 같은 폴더의 dev-cert.pem / dev-key.pem 이 없으면 한 번 생성한다 (openssl 호출).
  2) aiohttp 로 8443 포트에 HTTPS 정적 서버를 띄운다.
  3) /ws 경로에 WebSocket echo 엔드포인트를 둔다 — M0b 단계에선 단순 echo,
     M4 통합 시점에는 Mac proxy(§4.2)가 이 엔드포인트를 대체한다.

실행
----
    cd Indory/xlerobot-webxr/tools/webxr
    python3 serve.py --host 0.0.0.0 --port 8443

Quest 에서 접속
--------------
    https://<Mac의 LAN IP>:8443/
    (인증서 경고 → "advanced" → "proceed". 자기서명이라 매 디바이스 1회 필요)

의존성
------
    pip install aiohttp

운영 환경(M5+)에서는 이 서버 대신 Mac proxy(§4.2 aiohttp + pyzmq + aiortc)가
같은 페이지를 호스팅하고 WSS 를 ZMQ PUB 로 변환해 Tailscale 측에 publish 한다.
"""

import argparse
import asyncio
import json
import logging
import pathlib
import ssl
import subprocess
import sys
import time

from aiohttp import WSMsgType, web

ROOT = pathlib.Path(__file__).resolve().parent
CERT_PATH = ROOT / "dev-cert.pem"
KEY_PATH = ROOT / "dev-key.pem"

log = logging.getLogger("webxr-serve")


# ---------------------------------------------------------------------------
# Self-signed cert
# ---------------------------------------------------------------------------

def ensure_dev_cert() -> None:
    """Generate a self-signed cert valid for LAN use if not present.

    Quest Browser refuses to enter immersive-vr on HTTP. We need HTTPS even
    in dev. mkcert is more polished but the Quest user-CA store is fiddly
    (계획서 §10), so for M0b we use the simplest possible path: openssl
    self-signed, browser prompts the user to accept once.
    """
    if CERT_PATH.exists() and KEY_PATH.exists():
        return
    log.info("generating self-signed cert at %s", CERT_PATH)
    cmd = [
        "openssl", "req", "-x509", "-newkey", "rsa:2048", "-sha256",
        "-days", "365", "-nodes",
        "-keyout", str(KEY_PATH), "-out", str(CERT_PATH),
        "-subj", "/CN=xlerobot-webxr-dev",
        "-addext", "subjectAltName=DNS:localhost,IP:127.0.0.1",
    ]
    subprocess.run(cmd, check=True)


# ---------------------------------------------------------------------------
# WebSocket echo (M0b sanity)
# ---------------------------------------------------------------------------

async def ws_handler(request: web.Request) -> web.WebSocketResponse:
    """
    M0b 단계 echo. 클라이언트가 pose 페이로드를 보내면 서버 로그에 찍고,
    매 1초마다 `{"server_time_ms": ...}` 핑백을 보낸다.

    M4 통합 시점에는 Mac proxy 가 이 핸들러 자리를 대체한다 (§4.2):
    수신한 pose 페이로드를 ZMQ PUB `tcp://0.0.0.0:7001` 로 forward.
    """
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    peer = request.transport.get_extra_info("peername")
    log.info("ws connect from %s", peer)

    async def heartbeat():
        while not ws.closed:
            await asyncio.sleep(1.0)
            try:
                await ws.send_json({"server_time_ms": int(time.time() * 1000)})
            except ConnectionResetError:
                break

    hb_task = asyncio.create_task(heartbeat())
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    log.warning("non-json ws msg: %r", msg.data[:80])
                    continue
                t = payload.get("t")
                left_buttons = (payload.get("left") or {}).get("buttons") or {}
                right_buttons = (payload.get("right") or {}).get("buttons") or {}
                estop = payload.get("estop", False)
                # only log when something interesting happens — pose flood would be noise
                if estop or any(left_buttons.values()) or any(right_buttons.values()):
                    log.info("t=%s estop=%s L=%s R=%s", t, estop, left_buttons, right_buttons)
            elif msg.type == WSMsgType.ERROR:
                log.warning("ws error: %s", ws.exception())
    finally:
        hb_task.cancel()
        log.info("ws disconnect from %s", peer)
    return ws


# ---------------------------------------------------------------------------
# Static + app wiring
# ---------------------------------------------------------------------------

async def index(request: web.Request) -> web.FileResponse:
    return web.FileResponse(ROOT / "index.html")


def make_app() -> web.Application:
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    # serve everything else (mp4, js, etc.) statically from this directory.
    # NOTE: aiohttp's static needs a name; we route prefix '/static'.
    app.router.add_static("/static", path=str(ROOT), show_index=False)
    # also expose mp4/js/etc. at root so index.html can use relative paths
    app.router.add_static("/assets", path=str(ROOT / "assets"), show_index=False)
    return app


def make_ssl_context() -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=str(CERT_PATH), keyfile=str(KEY_PATH))
    return ctx


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8443)
    p.add_argument("--log-level", default="INFO")
    args = p.parse_args()

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )

    ensure_dev_cert()
    app = make_app()
    ssl_ctx = make_ssl_context()
    log.info("serving on https://%s:%s  (cert=%s)", args.host, args.port, CERT_PATH.name)
    log.info("from Quest:    https://<this-mac-LAN-ip>:%s/", args.port)
    log.info("ws endpoint:   wss://<this-mac-LAN-ip>:%s/ws", args.port)
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_ctx, access_log=None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
