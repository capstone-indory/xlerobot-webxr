from __future__ import annotations

import asyncio
import importlib.util
import inspect
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

SPEC = importlib.util.spec_from_file_location(
    "keyboard_teleop", ROOT / "tools" / "keyboard_teleop.py"
)
keyboard_teleop = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = keyboard_teleop
SPEC.loader.exec_module(keyboard_teleop)

import teleop_common


def _cfg(max_offset_m: float = keyboard_teleop.DEFAULT_MAX_OFFSET_M):
    return keyboard_teleop.Config(
        host="127.0.0.1",
        port=8765,
        sim_host="127.0.0.1",
        sim_pub_port=5555,
        sim_pull_port=5556,
        sim_rep_port=5557,
        robot_id=0,
        side="right",
        anchor_timeout_s=0.1,
        prime_frames=0,
        prime_interval_s=0.0,
        max_offset_m=max_offset_m,
        command_rate_hz=60.0,
        maintain_s=1.0,
        feedback_rate_hz=90.0,
        source_id="keyboard:test",
        source_role="teleop",
        priority=10,
        lease_ms=1000,
    )


def _teleop():
    teleop = keyboard_teleop.DirectSimTeleop(_cfg())
    pose = [0.310208, -0.132998, 0.898983, 0.0, 0.0, 0.0, 1.0]
    teleop.anchor_ee["right"] = list(pose)
    teleop.latest_ee["right"] = list(pose)
    return teleop


def test_target_pose_accumulates_nudge_from_anchor():
    teleop = _teleop()
    state = keyboard_teleop.CommandState(active=True, mode="jog")
    state.nudge = [0.05, 0.0, 0.0]

    pose = teleop._target_pose(state)

    assert pose[:3] == pytest.approx([0.360208, -0.132998, 0.898983])
    assert teleop.target_offsets["right"] == pytest.approx([0.05, 0.0, 0.0])


def test_target_pose_clamps_to_sim_workspace_radius():
    teleop = _teleop()
    state = keyboard_teleop.CommandState(active=True, mode="jog")
    state.nudge = [0.50, 0.0, 0.0]

    pose = teleop._target_pose(state)
    mount = teleop_common.ARM_MOUNT_OFFSET["right"]
    radius = sum((pose[i] - mount[i]) ** 2 for i in range(3)) ** 0.5

    assert radius <= teleop_common.EE_REACH_RADIUS_M + 1e-9
    assert pose[0] > 0.40
    assert teleop.target_offsets["right"][0] == pytest.approx(
        pose[0] - teleop.anchor_ee["right"][0]
    )


def test_default_max_offset_can_use_far_back_workspace():
    teleop = _teleop()
    state = keyboard_teleop.CommandState(active=True, mode="jog")
    state.nudge = [-0.73, 0.08, -0.005]

    pose = teleop._target_pose(state)

    assert pose[:3] == pytest.approx([-0.419792, -0.052998, 0.893983])
    assert teleop.target_offsets["right"] == pytest.approx([-0.73, 0.08, -0.005])


def test_common_payload_builds_v11_absolute_ee_command():
    pose = [0.31, -0.13, 0.90, 0.0, 0.0, 0.0, 1.0]

    payload = teleop_common.build_command_payload(0, {"right": pose}, stamp_ns=123)

    assert payload["schema"] == "xlerobot_v1.1"
    assert payload["stamp_ns"] == 123
    assert payload["robot_id"] == 0
    assert payload["arm_ee_pose_target"]["right"] == {
        "pose": pose,
        "mode": "absolute",
        "frame": "base",
    }
    assert payload["arm_joint_relative_target"]["right"] == {
        "shoulder_pan": 0.0,
        "gripper": 0.0,
    }


def test_common_payload_preserves_source_metadata():
    payload = teleop_common.build_command_payload(
        0,
        stamp_ns=123,
        source_id="keyboard:test",
        source_role="teleop",
        priority=10,
        lease_ms=1000,
    )

    assert payload["source_id"] == "keyboard:test"
    assert payload["source_role"] == "teleop"
    assert payload["priority"] == 10
    assert payload["lease_ms"] == 1000


def test_zero_motion_send_uses_stateless_hold_heartbeat(monkeypatch: pytest.MonkeyPatch):
    teleop = _teleop()
    teleop.push = object()
    sent: list[dict] = []

    async def _capture(payload):
        sent.append(payload)
        return True

    monkeypatch.setattr(teleop, "_send_payload", _capture)

    asyncio.run(teleop.send(keyboard_teleop.CommandState(active=True, mode="jog")))

    assert len(sent) == 1
    payload = sent[0]
    assert "arm_ee_pose_target" not in payload
    assert payload["arm_joint_relative_target"]["right"] == {
        "shoulder_pan": 0.0,
        "gripper": 0.0,
    }
    assert "source_id" not in payload
    assert "priority" not in payload


def test_zero_motion_during_active_source_lease_sends_no_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
):
    teleop = _teleop()
    teleop.push = object()
    teleop.source_lease_until_s = keyboard_teleop.time.monotonic() + 10.0
    sent: list[dict] = []

    async def _capture(payload):
        sent.append(payload)
        return True

    monkeypatch.setattr(teleop, "_send_payload", _capture)

    asyncio.run(teleop.send(keyboard_teleop.CommandState(active=True, mode="jog")))

    assert sent == []


def test_motion_send_uses_source_metadata(monkeypatch: pytest.MonkeyPatch):
    teleop = _teleop()
    teleop.push = object()
    sent: list[dict] = []

    async def _capture(payload):
        sent.append(payload)
        return True

    monkeypatch.setattr(teleop, "_send_payload", _capture)
    state = keyboard_teleop.CommandState(active=True, mode="jog")
    state.nudge = [0.05, 0.0, 0.0]

    asyncio.run(teleop.send(state))

    assert len(sent) == 1
    payload = sent[0]
    assert "arm_ee_pose_target" in payload
    assert payload["source_id"] == "keyboard:test"
    assert payload["source_role"] == "teleop"
    assert teleop.source_lease_until_s > keyboard_teleop.time.monotonic()


def test_common_payload_rejects_partial_source_metadata():
    with pytest.raises(ValueError, match="requires source_id"):
        teleop_common.build_command_payload(0, priority=10)


def test_common_tf_pose_parser_rejects_nonfinite_values():
    msg = {
        "targets": [
            {"name": "gripper_right", "pose": [1, 2, 3, 0, 0, 0, 1]},
            {"name": "gripper_left", "pose": [1, 2, float("nan"), 0, 0, 0, 1]},
        ]
    }

    poses = teleop_common.extract_tf_poses(msg)

    assert poses == {"right": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]}


def test_common_summarizes_vr_decode_debug_for_probe_output():
    debug = {
        "arms": {
            "right": {
                "changed_by_decoder": True,
                "coarse_clamped": False,
                "joint_limited_projected": True,
                "solver": "urdf_bounded_least_squares",
                "projection_mode": "directional_segment",
                "residual_m": 0.056789,
                "requested_pose_base": [0.405208, -0.132998, 0.898983, 0, 0, 0, 1],
                "target_pose_base": [0.348662, -0.132998, 0.8964, 0, 0, 0, 1],
            }
        }
    }

    summary = teleop_common.summarize_vr_decode_debug(debug, "right")

    assert summary == {
        "changed": True,
        "coarse_clamped": False,
        "joint_projected": True,
        "solver": "urdf_bounded_least_squares",
        "projection_mode": "directional_segment",
        "residual_m": 0.056789,
        "requested_xyz": [0.405208, -0.132998, 0.898983],
        "target_xyz": [0.348662, -0.132998, 0.8964],
    }


def test_common_cmd_echo_latency_filters_stale_or_future_echoes():
    assert teleop_common.cmd_echo_latency_ms(
        {"cmd_echo_stamp_ns": 1_000_000},
        now_ns=6_000_000,
    ) == pytest.approx(5.0)
    assert teleop_common.cmd_echo_latency_ms(
        {"cmd_echo_stamp_ns": 1_000_000},
        now_ns=6_000_000,
        min_stamp_ns=2_000_000,
    ) is None
    assert teleop_common.cmd_echo_latency_ms(
        {"cmd_echo_stamp_ns": 7_000_000},
        now_ns=6_000_000,
    ) is None


def test_latency_probes_report_cmd_echo_separately_from_motion():
    ws_probe = (ROOT / "tools" / "keyboard_ws_latency_probe.py").read_text()
    http_probe = (ROOT / "tools" / "keyboard_latency_probe.py").read_text()

    assert "first_cmd_echo_ms" in ws_probe
    assert "min_cmd_echo_stamp_ns=send_start_ns" in ws_probe
    assert "--hold-s" in ws_probe
    assert "sent_motion_commands" in ws_probe
    assert "send_drop_delta" in ws_probe
    assert "latency_fields_disabled" in ws_probe
    assert "move_to_projected_ratio" in ws_probe
    assert "target_error_m" in ws_probe
    assert "first_cmd_echo_ms" in http_probe
    assert "min_cmd_echo_stamp_ns=sent_start_ns" in http_probe


def test_browser_loop_does_not_double_throttle_with_timeout_and_raf():
    assert 'query.get("hz") || "90"' in keyboard_teleop.HTML
    assert 'query.get("step") || "0.100"' in keyboard_teleop.HTML
    assert 'query.get("speed") || "0.800"' in keyboard_teleop.HTML
    assert 'query.get("max_offset") || "0.800"' in keyboard_teleop.HTML
    assert "requestAnimationFrame(loop);" in keyboard_teleop.HTML
    assert "setTimeout(() => requestAnimationFrame(loop)" not in keyboard_teleop.HTML


def test_browser_keydown_keyup_send_immediately_without_waiting_for_raf():
    assert "let shouldSend = false;" in keyboard_teleop.HTML
    assert "if (shouldSend) {" in keyboard_teleop.HTML
    assert "sendCommand(0);" in keyboard_teleop.HTML
    assert 'window.addEventListener("keyup"' in keyboard_teleop.HTML


def test_browser_keydown_executes_immediate_websocket_command_path():
    if shutil.which("node") is None:
        pytest.skip("node is required for the browser JS execution smoke")
    match = re.search(r"<script>\n(?P<script>.*?)\n</script>", keyboard_teleop.HTML, re.S)
    assert match is not None
    harness = r"""
const vm = require("node:vm");
const script = require("node:fs").readFileSync(0, "utf8");
const listeners = {};
const elementListeners = {};
const elements = new Map();
const sends = [];
const wsInstances = [];
let nowMs = 1000;
let rafCount = 0;

function makeElement(id) {
  const el = {
    id,
    textContent: "",
    className: "",
    innerHTML: "",
    scrollTop: 0,
    scrollHeight: 0,
    classList: { toggle() {}, add() {}, remove() {} },
    focus() {},
    addEventListener(type, cb) {
      const key = `${id}:${type}`;
      if (!elementListeners[key]) elementListeners[key] = [];
      elementListeners[key].push(cb);
    },
  };
  return el;
}

function getElement(id) {
  if (!elements.has(id)) elements.set(id, makeElement(id));
  return elements.get(id);
}

class MockWebSocket {
  static OPEN = 1;
  constructor(url) {
    this.url = url;
    this.readyState = MockWebSocket.OPEN;
    wsInstances.push(this);
  }
  send(data) {
    sends.push(JSON.parse(data));
  }
}

const sandbox = {
  console,
  JSON,
  URLSearchParams,
  WebSocket: MockWebSocket,
  location: { protocol: "http:", host: "127.0.0.1:8765", search: "?robot=0&hz=90" },
  document: {
    body: makeElement("body"),
    getElementById: getElement,
  },
  performance: { now: () => nowMs },
  setInterval() {},
  setTimeout() {},
  requestAnimationFrame(cb) {
    rafCount += 1;
    sandbox.__lastRaf = cb;
  },
  window: {
    addEventListener(type, cb) {
      if (!listeners[type]) listeners[type] = [];
      listeners[type].push(cb);
    },
  },
};
sandbox.globalThis = sandbox;

function dispatchWindow(type, event) {
  for (const cb of listeners[type] || []) cb(event);
}

vm.runInNewContext(script, sandbox, { filename: "keyboard_teleop.html" });
if (wsInstances.length !== 1) throw new Error(`expected one websocket, got ${wsInstances.length}`);
wsInstances[0].onopen();
sends.length = 0;

let keydownPrevented = false;
const rafBeforeKeydown = rafCount;
dispatchWindow("keydown", {
  code: "KeyR",
  repeat: false,
  preventDefault() { keydownPrevented = true; },
});
const rafAfterKeydown = rafCount;
nowMs += 5;
dispatchWindow("keyup", {
  code: "KeyR",
  repeat: false,
  preventDefault() {},
});

process.stdout.write(JSON.stringify({
  url: wsInstances[0].url,
  keydownPrevented,
  rafBeforeKeydown,
  rafAfterKeydown,
  sendCount: sends.length,
  keydownPayload: sends[0],
  keyupPayload: sends[1],
}));
"""
    result = subprocess.run(
        ["node", "-e", harness],
        input=match.group("script"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    out = json.loads(result.stdout)

    assert out["url"] == "ws://127.0.0.1:8765/ws?robot=0"
    assert out["keydownPrevented"] is True
    assert out["rafAfterKeydown"] == out["rafBeforeKeydown"]
    assert out["sendCount"] == 2
    assert out["keydownPayload"]["type"] == "command"
    assert out["keydownPayload"]["nudge"] == pytest.approx([0.1, 0.0, 0.0])
    assert out["keydownPayload"]["target_offset"] == pytest.approx([0.1, 0.0, 0.0])
    assert out["keyupPayload"]["type"] == "command"
    assert out["keyupPayload"]["nudge"] == pytest.approx([0.0, 0.0, 0.0])


def test_browser_held_key_uses_raf_speed_delta_without_key_repeat():
    if shutil.which("node") is None:
        pytest.skip("node is required for the browser JS execution smoke")
    match = re.search(r"<script>\n(?P<script>.*?)\n</script>", keyboard_teleop.HTML, re.S)
    assert match is not None
    harness = r"""
const vm = require("node:vm");
const script = require("node:fs").readFileSync(0, "utf8");
const listeners = {};
const elements = new Map();
const sends = [];
const wsInstances = [];
let nowMs = 1000;
let rafCount = 0;

function makeElement(id) {
  return {
    id,
    textContent: "",
    className: "",
    innerHTML: "",
    scrollTop: 0,
    scrollHeight: 0,
    classList: { toggle() {}, add() {}, remove() {} },
    focus() {},
    addEventListener() {},
  };
}

function getElement(id) {
  if (!elements.has(id)) elements.set(id, makeElement(id));
  return elements.get(id);
}

class MockWebSocket {
  static OPEN = 1;
  constructor(url) {
    this.url = url;
    this.readyState = MockWebSocket.OPEN;
    wsInstances.push(this);
  }
  send(data) {
    sends.push(JSON.parse(data));
  }
}

const sandbox = {
  console,
  JSON,
  URLSearchParams,
  WebSocket: MockWebSocket,
  location: { protocol: "http:", host: "127.0.0.1:8765", search: "?robot=0&hz=90&speed=0.800&step=0.100" },
  document: { body: makeElement("body"), getElementById: getElement },
  performance: { now: () => nowMs },
  setInterval() {},
  setTimeout() {},
  requestAnimationFrame(cb) {
    rafCount += 1;
    sandbox.__lastRaf = cb;
  },
  window: {
    addEventListener(type, cb) {
      if (!listeners[type]) listeners[type] = [];
      listeners[type].push(cb);
    },
  },
};
sandbox.globalThis = sandbox;

function dispatchWindow(type, event) {
  for (const cb of listeners[type] || []) cb(event);
}
function tick(deltaMs) {
  nowMs += deltaMs;
  sandbox.__lastRaf(nowMs);
}

vm.runInNewContext(script, sandbox, { filename: "keyboard_teleop.html" });
wsInstances[0].onopen();
sends.length = 0;
dispatchWindow("keydown", {
  code: "KeyF",
  repeat: false,
  preventDefault() {},
});
tick(12);
tick(12);
dispatchWindow("keyup", {
  code: "KeyF",
  repeat: false,
  preventDefault() {},
});
const sendsAfterKeyup = sends.length;
tick(12);
tick(12);

process.stdout.write(JSON.stringify({
  sendCount: sends.length,
  sendsAfterKeyup,
  immediate: sends[0],
  held1: sends[1],
  held2: sends[2],
  keyup: sends[3],
  rafCount,
}));
"""
    result = subprocess.run(
        ["node", "-e", harness],
        input=match.group("script"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    out = json.loads(result.stdout)

    assert out["sendCount"] == 4
    assert out["immediate"]["nudge"] == pytest.approx([-0.1, 0.0, 0.0])
    assert out["held1"]["nudge"] == pytest.approx([-0.0096, 0.0, 0.0])
    assert out["held2"]["nudge"] == pytest.approx([-0.0096, 0.0, 0.0])
    assert out["held2"]["target_offset"] == pytest.approx([-0.1192, 0.0, 0.0])
    assert out["keyup"]["nudge"] == pytest.approx([0.0, 0.0, 0.0])
    assert out["sendCount"] == out["sendsAfterKeyup"]
    assert out["rafCount"] >= 3


def test_api_nudge_does_not_sleep_through_hold_frames():
    source = inspect.getsource(keyboard_teleop.api_nudge)

    assert "for _ in range(frames - 1):" not in source
    assert "await asyncio.sleep(sleep_s)" not in source


def test_keyboard_push_path_is_freshness_oriented_nonblocking():
    start_source = inspect.getsource(keyboard_teleop.DirectSimTeleop.start)
    send_source = inspect.getsource(keyboard_teleop.DirectSimTeleop._send_payload)

    assert "zmq.SNDHWM, 1" in start_source
    assert "zmq.SNDTIMEO, 0" in start_source
    assert "zmq.IMMEDIATE, 1" in start_source
    assert "flags=zmq.NOBLOCK" in send_source
    assert "self.send_drops += 1" in send_source


def test_browser_loop_sends_only_pending_or_held_commands():
    assert "function hasPendingCommand()" in keyboard_teleop.HTML
    assert "function hasHeldCommand()" in keyboard_teleop.HTML
    assert "&& hasPendingCommand()" in keyboard_teleop.HTML


def test_keyboard_gripper_defaults_use_responsive_delta():
    assert keyboard_teleop.DEFAULT_GRIPPER_STEP == pytest.approx(0.040)
    assert 'query.get("grip_step") || "0.040"' in keyboard_teleop.HTML
    assert "DEFAULT_GRIPPER_STEP" in inspect.getsource(keyboard_teleop.api_nudge)


def test_keyboard_zero_hold_heartbeat_is_throttled_and_source_free():
    source = inspect.getsource(keyboard_teleop.DirectSimTeleop.maintain_loop)
    hold_source = inspect.getsource(keyboard_teleop.DirectSimTeleop._send_hold)
    prime_source = inspect.getsource(keyboard_teleop.DirectSimTeleop._send_pose_hold)

    assert "ZERO_HOLD_HEARTBEAT_HZ" in source
    assert "**self._source_metadata()" not in hold_source
    assert "**self._source_metadata()" not in prime_source


def test_keyboard_configures_feedback_stream_rate_at_startup():
    source = inspect.getsource(keyboard_teleop.DirectSimTeleop.configure_feedback_streams)
    start_source = inspect.getsource(keyboard_teleop.DirectSimTeleop.start)

    assert '"set_stream_rate"' in source
    assert 'f"tf.links.{self.cfg.robot_id}"' in source
    assert 'f"proprio.{self.cfg.robot_id}"' in source
    assert "self.configure_feedback_streams()" in start_source


def test_websocket_handler_does_not_block_on_prime_before_receiving_commands():
    source = inspect.getsource(keyboard_teleop.WsHandler.handle)

    assert "await self.teleop.prime" not in source
