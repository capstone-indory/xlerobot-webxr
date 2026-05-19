"""Inspect local WebRTC ICE host candidates.

This is a fast preflight for the Mac proxy path: aiortc should publish host
candidates for the LAN interface used by Quest and, when Tailscale is up, for a
100.64.0.0/10 tailnet address used by the Home Server.
"""

from __future__ import annotations

import argparse
import asyncio
import ipaddress
import re
import sys
from dataclasses import dataclass
from typing import Optional


_CANDIDATE_RE = re.compile(r"^a=candidate:(?P<body>.+)$")
_TAILSCALE_NET = ipaddress.ip_network("100.64.0.0/10")


@dataclass
class IceCandidate:
    foundation: str
    component: str
    protocol: str
    priority: str
    host: str
    port: str
    typ: str
    raw: str


def _parse_candidate_line(line: str) -> Optional[IceCandidate]:
    m = _CANDIDATE_RE.match(line.strip())
    if not m:
        return None
    parts = m.group("body").split()
    if len(parts) < 8 or parts[6] != "typ":
        return None
    return IceCandidate(
        foundation=parts[0],
        component=parts[1],
        protocol=parts[2].lower(),
        priority=parts[3],
        host=parts[4],
        port=parts[5],
        typ=parts[7],
        raw=line.strip(),
    )


def _classify_host(host: str) -> str:
    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        return "name"
    if ip.is_loopback:
        return "loopback"
    if isinstance(ip, ipaddress.IPv4Address) and ip in _TAILSCALE_NET:
        return "tailscale"
    if ip.is_private:
        return "lan"
    return "public"


async def _await_ice_complete(pc, timeout_s: float) -> None:
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
        pass


async def _gather_candidates(stun_urls: list[str], timeout_s: float) -> list[IceCandidate]:
    try:
        from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection
    except ModuleNotFoundError as e:
        raise RuntimeError(
            "missing dependency 'aiortc'; run: python3 -m pip install -r tools/requirements.txt"
        ) from e

    ice_servers = [RTCIceServer(urls=url) for url in stun_urls]
    cfg = RTCConfiguration(iceServers=ice_servers) if ice_servers else RTCConfiguration()
    pc = RTCPeerConnection(configuration=cfg)
    try:
        pc.createDataChannel("ice-probe")
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await _await_ice_complete(pc, timeout_s)
        sdp = pc.localDescription.sdp if pc.localDescription else ""
        candidates = []
        for line in sdp.splitlines():
            cand = _parse_candidate_line(line)
            if cand is not None:
                candidates.append(cand)
        return candidates
    finally:
        await pc.close()


def _print_summary(candidates: list[IceCandidate]) -> tuple[bool, bool]:
    rows = []
    for cand in candidates:
        cls = _classify_host(cand.host)
        rows.append((cls, cand))

    if not rows:
        print("[FAIL] no ICE candidates found in local SDP")
        return False, False

    print("ICE host candidates")
    for cls, cand in rows:
        print(
            f"  [{cls.upper():9s}] {cand.host}:{cand.port} "
            f"{cand.protocol} typ={cand.typ}"
        )

    lan_ok = any(cls == "lan" and cand.typ == "host" for cls, cand in rows)
    tailscale_ok = any(cls == "tailscale" and cand.typ == "host" for cls, cand in rows)
    print()
    print(f"[{'OK' if lan_ok else 'FAIL'}] LAN host candidate present")
    print(f"[{'OK' if tailscale_ok else 'FAIL'}] Tailscale 100.64.0.0/10 host candidate present")
    if not tailscale_ok:
        print("[HINT] run 'tailscale up', then check firewall and utun interface state")
    return lan_ok, tailscale_ok


def main() -> int:
    p = argparse.ArgumentParser(description="Dump local WebRTC ICE host candidates")
    p.add_argument("--stun", action="append", default=[],
                   help="optional STUN URL, for example stun:stun.l.google.com:19302")
    p.add_argument("--timeout", type=float, default=3.0,
                   help="ICE gathering timeout in seconds")
    p.add_argument("--allow-missing-tailscale", action="store_true",
                   help="return success even when no Tailscale candidate is present")
    args = p.parse_args()

    try:
        candidates = asyncio.run(_gather_candidates(args.stun, args.timeout))
    except RuntimeError as e:
        print(f"[FAIL] {e}", file=sys.stderr)
        return 1
    lan_ok, tailscale_ok = _print_summary(candidates)
    if lan_ok and (tailscale_ok or args.allow_missing_tailscale):
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
