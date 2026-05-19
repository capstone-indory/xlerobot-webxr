"""
Measure browser-keyboard teleop command latency against indory_isaac_sim.

The probe assumes ``tools/keyboard_teleop.py`` is already running. It sends one
HTTP ``/api/nudge`` request through that server, then watches ``tf.links`` until
the selected EE moves by a threshold.

Typical run:

  python3 tools/keyboard_latency_probe.py \
    --keyboard-url http://127.0.0.1:8765 \
    --sim-host 100.80.87.68 --robot-id 0 --code KeyR
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.parse
import urllib.request
from typing import Any

from teleop_common import DEFAULT_SIM_HOST, TF_TARGET_NAMES, TfReader, round_list


def _http_json(
    url: str,
    *,
    data: dict[str, Any] | None = None,
    timeout_s: float = 2.0,
) -> dict[str, Any]:
    body = None
    headers = {"Accept": "application/json"}
    if data is not None:
        body = urllib.parse.urlencode(data).encode("utf-8")
        headers["Content-Type"] = "application/x-www-form-urlencoded"
    req = urllib.request.Request(
        url,
        data=body,
        headers=headers,
        method="POST" if data is not None else "GET",
    )
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send one keyboard_teleop nudge and measure tf.links movement latency.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--keyboard-url", default="http://127.0.0.1:8765")
    parser.add_argument("--sim-host", default=DEFAULT_SIM_HOST)
    parser.add_argument("--sim-pub-port", type=int, default=5555)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--side", choices=tuple(TF_TARGET_NAMES), default="right")
    parser.add_argument("--code", default="KeyR", help="Keyboard code sent to /api/nudge.")
    parser.add_argument("--step", type=float, default=0.05)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--move-threshold", type=float, default=0.002)
    parser.add_argument("--observe-s", type=float, default=2.0)
    parser.add_argument("--anchor-timeout", type=float, default=3.0)
    args = parser.parse_args()

    base = args.keyboard_url.rstrip("/")
    health = _http_json(f"{base}/health")
    if not health.get("anchor_ready"):
        print("keyboard teleop anchor not ready", json.dumps(health, ensure_ascii=False))
        return 2

    reader = TfReader(args.sim_host, args.sim_pub_port, args.robot_id, args.side)
    try:
        start_pose = reader.next_pose(args.anchor_timeout)
        if start_pose is None:
            print(f"missing tf.links pose for {TF_TARGET_NAMES[args.side]}")
            return 2
        print("start", round_list(start_pose[:3]))

        sent_t = time.monotonic()
        reply = _http_json(
            f"{base}/api/nudge",
            data={
                "code": args.code,
                "step": args.step,
                "frames": args.frames,
                "hz": args.hz,
            },
        )
        post_rtt_ms = (time.monotonic() - sent_t) * 1000.0
        print(
            "post_reply",
            json.dumps(reply, ensure_ascii=False),
            f"rtt_ms={post_rtt_ms:.2f}",
        )

        first_move_s, max_move, last_pose, samples = reader.wait_for_move(
            start_pose,
            timeout_s=args.observe_s,
            move_threshold_m=args.move_threshold,
        )
        print("last", None if last_pose is None else round_list(last_pose[:3]))
        print(
            "summary",
            json.dumps(
                {
                    "first_move_ms": None
                    if first_move_s is None
                    else round(first_move_s * 1000.0, 2),
                    "max_move_m": round(max_move, 6),
                    "samples": samples,
                    "post_rtt_ms": round(post_rtt_ms, 2),
                },
                ensure_ascii=False,
            ),
        )
        if first_move_s is None:
            return 1
        return 0
    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
