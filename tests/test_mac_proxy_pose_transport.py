from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAC_PROXY = ROOT / "tools" / "mac_proxy.py"


def test_mac_proxy_pose_pub_is_freshness_oriented():
    source = MAC_PROXY.read_text()

    assert "self.sock.setsockopt(zmq.SNDHWM, 1)" in source
    assert "self.sock.setsockopt(zmq.SNDTIMEO, 0)" in source
    assert "self.sock.setsockopt(zmq.LINGER, 0)" in source
    assert "flags=zmq.NOBLOCK" in source
    assert "except zmq.Again:" in source
    assert "self.send_drops += 1" in source


def test_mac_proxy_has_pose_only_without_top_level_aiortc_import():
    source = MAC_PROXY.read_text()

    assert "--pose-only" in source
    assert "def _load_webrtc_deps()" in source
    assert "from aiortc import (" not in source.split("def _load_webrtc_deps()", 1)[0]
    assert "pose-only mode: WebRTC signaling/video disabled" in source


def test_webxr_pose_payload_has_single_timestamp_field():
    source = (ROOT / "tools" / "webxr" / "index.html").read_text()

    assert source.count("t: Math.round(now)") == 1
    assert "ws.send(JSON.stringify({" in source
    assert "const POSE_SEND_HZ = 90" in source
