"""
xlerobot-webxr / tools/keyboard_teleop.py

Non-VR browser keyboard teleop for indory_isaac_sim.

The page sends simple keyboard state to this local web server. The server reads
the current EE pose from sim tf.links.<robot_id>, builds xlerobot_v1.1
arm_ee_pose_target commands, and pushes them directly to the sim PULL port.
No Quest, WebXR session, Mac proxy, WebRTC, or VR bridge is started.

Typical run:

  python3 tools/keyboard_teleop.py --sim-host 100.80.87.68 --robot-id 0

Then open:

  http://127.0.0.1:8765/?robot=0
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any

import msgpack
import zmq
import zmq.asyncio
from aiohttp import WSMsgType, web


log = logging.getLogger("keyboard_teleop")

SCHEMA_VERSION_V11 = "xlerobot_v1.1"
TF_TARGET_NAMES = {"right": "gripper_right", "left": "gripper_left"}
ARM_SIDES = ("right", "left")
DEFAULT_SIM_HOST = "100.80.87.68"
DEFAULT_ROBOT_ID = 0
DEFAULT_STEP_M = 0.050
DEFAULT_SPEED_MPS = 0.240
DEFAULT_MAX_OFFSET_M = 0.300
EE_REACH_RADIUS_M = 0.56
ARM_MOUNT_OFFSET = {
    "right": (-0.135, -0.133, 0.760),
    "left": (-0.135, +0.133, 0.760),
}


HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<title>XLerobot Keyboard Teleop</title>
<style>
  :root {
    color-scheme: dark;
    --bg: #0b0d10;
    --panel: #151b22;
    --panel2: #0f141a;
    --line: #2a3441;
    --text: #e8edf2;
    --muted: #8ea0b4;
    --accent: #31c48d;
    --warn: #f59e0b;
    --stop: #ef4444;
  }
  * { box-sizing: border-box; }
  body { margin: 0; min-height: 100vh; background: var(--bg); color: var(--text);
    font-family: ui-sans-serif, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
  body:focus { outline: none; }
  main { width: min(980px, 100%); margin: 0 auto; padding: 24px; }
  header { display: flex; justify-content: space-between; align-items: center; gap: 16px;
    padding-bottom: 16px; border-bottom: 1px solid var(--line); }
  h1 { margin: 0; font-size: 23px; letter-spacing: 0; }
  .status { display: flex; gap: 8px; flex-wrap: wrap; justify-content: flex-end; }
  .pill { border: 1px solid var(--line); background: var(--panel); color: var(--muted);
    border-radius: 8px; padding: 7px 10px; font: 13px ui-monospace, SFMono-Regular, monospace; }
  .pill.ok { background: var(--accent); border-color: var(--accent); color: #052e1f; }
  .pill.warn { background: var(--warn); border-color: var(--warn); color: #231400; }
  .pill.stop { background: var(--stop); border-color: var(--stop); color: white; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; margin-top: 18px; }
  section { border: 1px solid var(--line); background: var(--panel); border-radius: 8px; padding: 16px; }
  h2 { margin: 0 0 14px 0; font-size: 15px; color: var(--muted); }
  .pad { display: grid; grid-template-columns: repeat(4, minmax(54px, 1fr)); gap: 8px; max-width: 360px; }
  .key { height: 52px; border: 1px solid var(--line); border-radius: 8px; background: var(--panel2);
    color: var(--muted); display: grid; place-items: center; font: 16px ui-monospace, SFMono-Regular, monospace;
    cursor: pointer; user-select: none; touch-action: none; }
  .key.on { background: #16382d; color: var(--text); border-color: var(--accent); }
  .span2 { grid-column: span 2; }
  .controls { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 14px; }
  button { border: 1px solid var(--line); border-radius: 8px; background: var(--panel2); color: var(--text);
    padding: 10px 12px; font-weight: 650; cursor: pointer; }
  button.active { background: var(--accent); color: #052e1f; border-color: var(--accent); }
  button.stop { background: var(--stop); color: white; border-color: var(--stop); }
  .metrics { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
  .metric { background: var(--panel2); border: 1px solid var(--line); border-radius: 8px; padding: 12px; }
  .label { color: var(--muted); font-size: 12px; margin-bottom: 6px; }
  .value { font: 15px ui-monospace, SFMono-Regular, monospace; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .log { margin-top: 16px; height: 172px; overflow: auto; background: #06080a; border: 1px solid var(--line);
    border-radius: 8px; padding: 10px; color: var(--muted); font: 12px ui-monospace, SFMono-Regular, monospace; }
  @media (max-width: 780px) {
    main { padding: 16px; }
    header { align-items: flex-start; flex-direction: column; }
    .status { justify-content: flex-start; }
    .grid { grid-template-columns: 1fr; }
  }
</style>
</head>
<body tabindex="0">
<main>
  <header>
    <h1>XLerobot Keyboard Teleop</h1>
    <div class="status">
      <div id="conn" class="pill">ws: closed</div>
      <div id="sim" class="pill">sim: -</div>
      <div id="anchor" class="pill warn">anchor: wait</div>
      <div id="rate" class="pill">tx: 0 Hz</div>
      <div id="estop" class="pill ok">ready</div>
    </div>
  </header>

  <div class="grid">
    <section>
      <h2>Right Arm</h2>
      <form id="padForm" method="post" action="/api/nudge" class="pad">
        <div></div><button id="KeyW" name="code" value="KeyW" class="key" type="submit" aria-label="jog up">W</button><button id="KeyR" name="code" value="KeyR" class="key" type="submit" aria-label="jog forward">R</button><div></div>
        <button id="KeyA" name="code" value="KeyA" class="key" type="submit" aria-label="pan left">A</button><button id="KeyS" name="code" value="KeyS" class="key" type="submit" aria-label="jog down">S</button><button id="KeyD" name="code" value="KeyD" class="key" type="submit" aria-label="pan right">D</button><button id="KeyF" name="code" value="KeyF" class="key" type="submit" aria-label="jog back">F</button>
        <button id="KeyZ" name="code" value="KeyZ" class="key span2" type="submit" aria-label="close gripper">Z close</button><button id="KeyX" name="code" value="KeyX" class="key span2" type="submit" aria-label="open gripper">X open</button>
        <button id="Digit0" name="code" value="Digit0" class="key span2" type="submit" aria-label="re-anchor">0 re-anchor</button><button id="Space" class="key span2" type="button" aria-label="stream toggle">Space</button>
      </form>
      <div class="controls">
        <button id="activeBtn" class="active">stream on</button>
        <button id="resetBtn">re-anchor</button>
        <button id="stopBtn" class="stop">estop</button>
      </div>
    </section>

    <section>
      <h2>Command</h2>
      <div class="metrics">
        <div class="metric"><div class="label">jog xyz</div><div id="jog" class="value">0, 0, 0</div></div>
        <div class="metric"><div class="label">ee xyz</div><div id="ee" class="value">-, -, -</div></div>
        <div class="metric"><div class="label">pan delta</div><div id="pan" class="value">0.000</div></div>
        <div class="metric"><div class="label">gripper delta</div><div id="gripper" class="value">0.000</div></div>
        <div class="metric"><div class="label">input</div><div id="input" class="value">idle</div></div>
        <div class="metric"><div class="label">frames</div><div id="frames" class="value">0</div></div>
      </div>
      <div id="log" class="log"></div>
    </section>
  </div>
</main>

<script>
const query = new URLSearchParams(location.search);
const robotId = Number.parseInt(query.get("robot") || "0", 10);
const sendHz = Number.parseFloat(query.get("hz") || "60");
const stepM = Number.parseFloat(query.get("step") || "0.050");
const speedMps = Number.parseFloat(query.get("speed") || "0.240");
const maxOffset = Number.parseFloat(query.get("max_offset") || "0.300");
const panStep = Number.parseFloat(query.get("pan_step") || "0.050");
const gripStep = Number.parseFloat(query.get("grip_step") || "0.010");
const maxPan = Number.parseFloat(query.get("max_pan") || "0.700");
const motionKeys = new Set(["KeyW", "KeyS", "KeyR", "KeyF"]);
const pressed = new Set();
const queuedNudge = [0, 0, 0];
const targetOffset = [0, 0, 0];
let lastNudge = [0, 0, 0];
let lastInput = "idle";
let latestEe = null;
let active = true;
let estop = false;
let reanchor = false;
let ws = null;
let frames = 0;
let sentInWindow = 0;
let lastTick = performance.now();

const $ = (id) => document.getElementById(id);
window.addEventListener("load", () => document.body.focus());
window.addEventListener("pointerdown", () => document.body.focus(), true);
function log(msg) {
  const t = new Date().toLocaleTimeString();
  $("log").innerHTML += `<div>[${t}] ${msg}</div>`;
  $("log").scrollTop = $("log").scrollHeight;
}
function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}
function wsUrl() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws?robot=${Number.isFinite(robotId) ? robotId : 0}`;
}
function connect() {
  ws = new WebSocket(wsUrl());
  ws.onopen = () => {
    $("conn").textContent = "ws: open";
    $("conn").className = "pill ok";
    ws.send(JSON.stringify({select_robot: Number.isFinite(robotId) ? robotId : 0}));
    log("connected");
  };
  ws.onclose = () => {
    $("conn").textContent = "ws: closed";
    $("conn").className = "pill warn";
    setTimeout(connect, 700);
  };
  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === "status") {
        $("sim").textContent = `sim: ${msg.sim_host}`;
        $("anchor").textContent = msg.anchor_ready ? "anchor: ready" : "anchor: wait";
        $("anchor").className = msg.anchor_ready ? "pill ok" : "pill warn";
        latestEe = Array.isArray(msg.ee_xyz) ? msg.ee_xyz : latestEe;
      }
    } catch (_) {}
  };
}
function updateKeys() {
  for (const code of ["KeyW","KeyA","KeyS","KeyD","KeyR","KeyF","KeyZ","KeyX","Digit0","Space"]) {
    const el = $(code);
    if (el) el.classList.toggle("on", pressed.has(code));
  }
}
function moveStep() {
  const gain = (pressed.has("ShiftLeft") || pressed.has("ShiftRight")) ? 2.5 : 1.0;
  return stepM * gain;
}
function moveSpeed() {
  const gain = (pressed.has("ShiftLeft") || pressed.has("ShiftRight")) ? 3.0 : 1.0;
  return speedMps * gain;
}
function applyNudgeVector(next) {
  const applied = [0, 0, 0];
  if (next.some(v => v !== 0)) {
    for (let i = 0; i < 3; i += 1) {
      const before = targetOffset[i];
      targetOffset[i] = clamp(before + next[i], -maxOffset, maxOffset);
      applied[i] = targetOffset[i] - before;
    }
    active = true;
  }
  return applied;
}
function queueNudge(code) {
  const move = moveStep();
  const next = [0, 0, 0];
  if (code === "KeyW") next[2] = +move;
  if (code === "KeyS") next[2] = -move;
  if (code === "KeyR") next[0] = +move;
  if (code === "KeyF") next[0] = -move;
  const applied = applyNudgeVector(next);
  if (applied.some(v => v !== 0)) {
    for (let i = 0; i < 3; i += 1) queuedNudge[i] += applied[i];
    lastInput = code;
  }
}
function heldNudge(dt) {
  if (dt <= 0) return [0, 0, 0];
  const move = moveSpeed() * dt;
  const next = [0, 0, 0];
  if (pressed.has("KeyW")) next[2] += move;
  if (pressed.has("KeyS")) next[2] -= move;
  if (pressed.has("KeyR")) next[0] += move;
  if (pressed.has("KeyF")) next[0] -= move;
  return applyNudgeVector(next);
}
function tapInput(code) {
  if (code === "Space") {
    active = !active;
    lastInput = code;
    return false;
  } else if (code === "Digit0") {
    reanchor = true;
    targetOffset[0] = 0; targetOffset[1] = 0; targetOffset[2] = 0;
    lastInput = code;
    return false;
  } else if (code === "KeyA" || code === "KeyD" || code === "KeyZ" || code === "KeyX") {
    pressed.add(code);
    active = true;
    lastInput = code;
    return true;
  } else {
    queueNudge(code);
    return false;
  }
}
function panDelta() {
  let delta = 0;
  if (pressed.has("KeyA")) delta -= panStep;
  if (pressed.has("KeyD")) delta += panStep;
  return clamp(delta, -maxPan, maxPan);
}
function gripperDelta() {
  let delta = 0;
  if (pressed.has("KeyZ")) delta -= gripStep;
  if (pressed.has("KeyX")) delta += gripStep;
  return delta;
}
function sendCommand(dt = 0) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const nudge = queuedNudge.slice();
  const held = heldNudge(dt);
  for (let i = 0; i < 3; i += 1) nudge[i] += held[i];
  lastNudge = nudge.slice();
  queuedNudge[0] = 0; queuedNudge[1] = 0; queuedNudge[2] = 0;
  ws.send(JSON.stringify({
    type: "command",
    mode: "jog",
    active,
    estop,
    nudge,
    target_offset: targetOffset.slice(),
    pan_delta: panDelta(),
    gripper_delta: gripperDelta(),
    reanchor,
  }));
  reanchor = false;
  frames += 1;
  sentInWindow += 1;
}
function render() {
  $("estop").textContent = estop ? "estop" : "ready";
  $("estop").className = estop ? "pill stop" : "pill ok";
  $("activeBtn").textContent = active ? "stream on" : "stream off";
  $("activeBtn").classList.toggle("active", active);
  $("jog").textContent = targetOffset.map(v => v.toFixed(3)).join(", ");
  $("ee").textContent = latestEe ? latestEe.map(v => Number(v).toFixed(3)).join(", ") : "-, -, -";
  $("pan").textContent = panDelta().toFixed(3);
  $("gripper").textContent = gripperDelta().toFixed(3);
  $("input").textContent = lastInput;
  $("frames").textContent = String(frames);
  updateKeys();
}
function loop(now) {
  const dt = Math.min(0.05, Math.max(0.001, (now - lastTick) / 1000));
  lastTick = now;
  sendCommand(dt);
  render();
  setTimeout(() => requestAnimationFrame(loop), 1000 / sendHz);
}
setInterval(() => {
  $("rate").textContent = `tx: ${sentInWindow} Hz`;
  sentInWindow = 0;
}, 1000);
window.addEventListener("keydown", (e) => {
  const code = e.code || "";
  if (["Space","KeyW","KeyA","KeyS","KeyD","KeyR","KeyF","KeyZ","KeyX","Digit0"].includes(code)) {
    e.preventDefault();
  }
  if (code === "Space") {
    active = !active;
    pressed.add(code);
    lastInput = code;
  } else if (code === "Escape") {
    estop = true;
    lastInput = code;
  } else if (code === "Digit0") {
    reanchor = true;
    targetOffset[0] = 0; targetOffset[1] = 0; targetOffset[2] = 0;
    pressed.add(code);
    lastInput = code;
  } else if (motionKeys.has(code)) {
    if (!pressed.has(code) && !e.repeat) queueNudge(code);
    pressed.add(code);
    active = true;
    lastInput = code || lastInput;
  } else {
    pressed.add(code);
    active = true;
    lastInput = code || lastInput;
  }
});
window.addEventListener("keyup", (e) => {
  pressed.delete(e.code);
  if (!pressed.size) lastInput = "idle";
});
window.addEventListener("blur", () => pressed.clear());
$("activeBtn").onclick = () => { active = !active; lastInput = "stream"; };
$("resetBtn").onclick = () => {
  reanchor = true;
  targetOffset[0] = 0; targetOffset[1] = 0; targetOffset[2] = 0;
  active = true;
  lastInput = "re-anchor";
};
$("stopBtn").onclick = () => { estop = !estop; lastInput = "estop"; };
for (const code of ["KeyW","KeyA","KeyS","KeyD","KeyR","KeyF","KeyZ","KeyX","Digit0","Space"]) {
  const el = $(code);
  if (!el) continue;
  let pointerStarted = 0;
  const down = (e) => {
    e.preventDefault();
    pointerStarted = performance.now();
    if (code === "Space") {
      active = !active;
    } else if (code === "Digit0") {
      reanchor = true;
      targetOffset[0] = 0; targetOffset[1] = 0; targetOffset[2] = 0;
    } else {
      pressed.add(code);
      if (motionKeys.has(code)) queueNudge(code);
    }
    active = code === "Space" ? active : true;
    lastInput = code;
    sendCommand(0);
    render();
  };
  const up = (e) => {
    e.preventDefault();
    pressed.delete(code);
    if (!pressed.size) lastInput = "idle";
    sendCommand(0);
    render();
  };
  el.addEventListener("pointerdown", down);
  el.addEventListener("pointerup", up);
  el.addEventListener("pointercancel", up);
  el.addEventListener("pointerleave", up);
  el.addEventListener("click", (e) => {
    e.preventDefault();
    if (performance.now() - pointerStarted < 250) return;
    const transient = tapInput(code);
    sendCommand(0);
    if (transient) pressed.delete(code);
    render();
  });
}
connect();
requestAnimationFrame(loop);
</script>
</body>
</html>
"""


@dataclass
class Config:
    host: str
    port: int
    sim_host: str
    sim_pub_port: int
    sim_pull_port: int
    robot_id: int
    side: str
    anchor_timeout_s: float
    prime_frames: int
    prime_interval_s: float
    max_offset_m: float
    command_rate_hz: float
    maintain_s: float


@dataclass
class CommandState:
    active: bool = True
    estop: bool = False
    mode: str = "absolute"
    offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    nudge: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    jog: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pan_delta: float = 0.0
    gripper_delta: float = 0.0
    reanchor: bool = False

    @property
    def is_zero_motion(self) -> bool:
        return (
            not self.estop
            and not self.reanchor
            and all(abs(v) < 1e-9 for v in self.offset[:3])
            and all(abs(v) < 1e-9 for v in self.nudge[:3])
            and all(abs(v) < 1e-9 for v in self.jog[:3])
            and abs(self.pan_delta) < 1e-9
            and abs(self.gripper_delta) < 1e-9
        )

    @property
    def is_idle_zero(self) -> bool:
        return (
            self.active
            and self.is_zero_motion
        )


class DirectSimTeleop:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.ctx = zmq.asyncio.Context.instance()
        self.push: zmq.asyncio.Socket | None = None
        self.sub: zmq.asyncio.Socket | None = None
        self.latest_ee: dict[str, list[float]] = {}
        self.anchor_ee: dict[str, list[float]] = {}
        self.target_offsets: dict[str, list[float]] = {
            side: [0.0, 0.0, 0.0] for side in ARM_SIDES
        }
        self.stream_until_s = 0.0
        self.last_send_ns = 0

    def start(self) -> None:
        self.push = self.ctx.socket(zmq.PUSH)
        self.push.setsockopt(zmq.SNDHWM, 8)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(f"tcp://{self.cfg.sim_host}:{self.cfg.sim_pull_port}")

        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.RCVHWM, 8)
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.connect(f"tcp://{self.cfg.sim_host}:{self.cfg.sim_pub_port}")
        self.sub.setsockopt(zmq.SUBSCRIBE, f"tf.links.{self.cfg.robot_id}".encode())
        log.info(
            "direct sim teleop: PUB tcp://%s:%d, PUSH tcp://%s:%d",
            self.cfg.sim_host,
            self.cfg.sim_pub_port,
            self.cfg.sim_host,
            self.cfg.sim_pull_port,
        )

    def close(self) -> None:
        if self.push is not None:
            self.push.close(linger=0)
            self.push = None
        if self.sub is not None:
            self.sub.close(linger=0)
            self.sub = None

    @property
    def anchor_ready(self) -> bool:
        return self.cfg.side in self.anchor_ee

    @property
    def all_anchors_ready(self) -> bool:
        return all(side in self.anchor_ee for side in ARM_SIDES)

    async def tf_loop(self) -> None:
        assert self.sub is not None
        while True:
            topic, payload = await self.sub.recv_multipart()
            try:
                msg = msgpack.unpackb(payload, raw=False)
            except Exception:
                continue
            updates: dict[str, list[float]] = {}
            for entry in msg.get("targets", []) or []:
                name = entry.get("name")
                pose = entry.get("pose")
                if not isinstance(pose, (list, tuple)) or len(pose) != 7:
                    continue
                for side, target_name in TF_TARGET_NAMES.items():
                    if name == target_name:
                        updates[side] = [float(v) for v in pose]
            if not updates:
                continue
            self.latest_ee.update(updates)
            for side, pose in updates.items():
                self.anchor_ee.setdefault(side, list(pose))

    async def wait_for_anchor(self) -> bool:
        deadline = time.monotonic() + self.cfg.anchor_timeout_s
        while time.monotonic() < deadline:
            if self.anchor_ready:
                return True
            await asyncio.sleep(0.05)
        return False

    def reanchor(self) -> None:
        if self.cfg.side in self.latest_ee:
            self.anchor_ee[self.cfg.side] = list(self.latest_ee[self.cfg.side])
            self.target_offsets[self.cfg.side] = [0.0, 0.0, 0.0]
            log.info("re-anchored %s to latest tf.links pose", self.cfg.side)

    async def prime(self, frames: int | None = None, interval_s: float | None = None) -> None:
        """Seed the sim IK slots with current EE poses before user motion.

        This mirrors indory_isaac_sim/examples/keyboard_client.py, which sends
        both current EE targets before streaming selected-side nudges.
        """
        if self.push is None or not self.anchor_ready:
            return
        frame_count = self.cfg.prime_frames if frames is None else int(frames)
        sleep_s = self.cfg.prime_interval_s if interval_s is None else float(interval_s)
        frame_count = max(0, frame_count)
        sides = ARM_SIDES if self.all_anchors_ready else (self.cfg.side,)
        for _ in range(frame_count):
            await self._send_pose_hold(sides)
            if sleep_s > 0:
                await asyncio.sleep(sleep_s)

    async def maintain_loop(self) -> None:
        sleep_s = 1.0 / max(self.cfg.command_rate_hz, 1.0)
        hold = CommandState(active=True, mode="jog")
        while True:
            if self.anchor_ready and time.monotonic() < self.stream_until_s:
                await self.send(hold, renew_stream=False)
            await asyncio.sleep(sleep_s)

    async def send(self, state: CommandState, *, renew_stream: bool = True) -> None:
        if self.push is None:
            return
        if state.reanchor:
            self.reanchor()
        if state.estop or not state.active:
            self.stream_until_s = 0.0
            await self._send_hold()
            return
        if not self.anchor_ready:
            return

        if renew_stream and not state.is_zero_motion:
            self.stream_until_s = time.monotonic() + self.cfg.maintain_s

        side = self.cfg.side
        targets = self._hold_targets()
        targets[side] = self._target_pose(state)
        rel = {
            arm_side: {
                "shoulder_pan": float(state.pan_delta) if arm_side == side else 0.0,
                "gripper": float(state.gripper_delta) if arm_side == side else 0.0,
            }
            for arm_side in targets.keys()
        }
        await self._send_pose_target(targets, rel)

    def _target_pose(self, state: CommandState) -> list[float]:
        side = self.cfg.side
        nudge = state.nudge[:3]
        if any(abs(v) >= 1e-9 for v in nudge):
            offset = self.target_offsets.setdefault(side, [0.0, 0.0, 0.0])
            for idx, delta in enumerate(nudge):
                offset[idx] = max(
                    -self.cfg.max_offset_m,
                    min(self.cfg.max_offset_m, offset[idx] + float(delta)),
                )
        if state.mode == "jog" and any(abs(v) >= 1e-9 for v in state.jog[:3]):
            # Legacy pre-nudge web clients sent "jog" every frame. Keep that
            # behavior for already-open tabs, but new clients use nudge +
            # anchor-relative offsets to avoid target chasing latency.
            base = self.latest_ee.get(side) or self.anchor_ee[side]
            pose = list(base)
            deltas = state.jog[:3]
        elif state.mode == "jog":
            pose = list(self.anchor_ee[side])
            deltas = self.target_offsets.setdefault(side, [0.0, 0.0, 0.0])[:3]
        else:
            pose = list(self.anchor_ee[side])
            deltas = state.offset[:3]
        current = self.latest_ee.get(side)
        if isinstance(current, list) and len(current) == 7:
            # Match indory_isaac_sim/examples/keyboard_client.py: let
            # orientation float by reusing live tf.links orientation.
            pose[3:7] = current[3:7]
        for idx, delta in enumerate(deltas):
            pose[idx] += float(delta)
        pose = _clamp_pose_to_workspace(side, pose)
        anchor = self.anchor_ee.get(side)
        if isinstance(anchor, list) and len(anchor) >= 3:
            self.target_offsets[side] = [
                float(pose[idx]) - float(anchor[idx]) for idx in range(3)
            ]
        return pose

    def _hold_targets(self) -> dict[str, list[float]]:
        targets: dict[str, list[float]] = {}
        for side in ARM_SIDES:
            pose = self._hold_pose(side)
            if pose is not None:
                targets[side] = pose
        if self.cfg.side not in targets and self.cfg.side in self.anchor_ee:
            targets[self.cfg.side] = list(self.anchor_ee[self.cfg.side])
        return targets

    def _hold_pose(self, side: str) -> list[float] | None:
        if side not in self.anchor_ee:
            return None
        pose = list(self.latest_ee.get(side) or self.anchor_ee[side])
        current = self.latest_ee.get(side)
        if isinstance(current, list) and len(current) == 7:
            pose[3:7] = current[3:7]
        return pose

    async def _send_pose_target(
        self,
        targets_by_side: dict[str, list[float]],
        rel: dict[str, dict[str, float]],
    ) -> None:
        if self.push is None:
            return
        payload = {
            "schema": SCHEMA_VERSION_V11,
            "stamp_ns": time.monotonic_ns(),
            "robot_id": self.cfg.robot_id,
            "frame": "body",
            "base_cmd_vel": [0.0, 0.0, 0.0],
            "arm_ee_pose_target": {
                side: {"pose": pose, "mode": "absolute", "frame": "base"}
                for side, pose in targets_by_side.items()
            },
            "arm_joint_relative_target": rel,
            "head_joint_relative_target": {"head_pan": 0.0, "head_tilt": 0.0},
        }
        await self.push.send(msgpack.packb(payload, use_bin_type=True))
        self.last_send_ns = payload["stamp_ns"]

    async def _send_pose_hold(self, sides: tuple[str, ...]) -> None:
        if self.push is None:
            return
        targets: dict[str, dict[str, Any]] = {}
        rel: dict[str, dict[str, float]] = {}
        for side in sides:
            pose = self._hold_pose(side)
            if pose is None:
                continue
            targets[side] = {"pose": pose, "mode": "absolute", "frame": "base"}
            rel[side] = {"shoulder_pan": 0.0, "gripper": 0.0}
        if not targets:
            return
        payload = {
            "schema": SCHEMA_VERSION_V11,
            "stamp_ns": time.monotonic_ns(),
            "robot_id": self.cfg.robot_id,
            "frame": "body",
            "base_cmd_vel": [0.0, 0.0, 0.0],
            "arm_ee_pose_target": targets,
            "arm_joint_relative_target": rel,
            "head_joint_relative_target": {"head_pan": 0.0, "head_tilt": 0.0},
        }
        await self.push.send(msgpack.packb(payload, use_bin_type=True))
        self.last_send_ns = payload["stamp_ns"]

    async def _send_hold(self) -> None:
        if self.push is None:
            return
        payload = {
            "schema": SCHEMA_VERSION_V11,
            "stamp_ns": time.monotonic_ns(),
            "robot_id": self.cfg.robot_id,
            "frame": "body",
            "base_cmd_vel": [0.0, 0.0, 0.0],
            "arm_joint_relative_target": {
                self.cfg.side: {"shoulder_pan": 0.0, "gripper": 0.0}
            },
            "head_joint_relative_target": {"head_pan": 0.0, "head_tilt": 0.0},
        }
        await self.push.send(msgpack.packb(payload, use_bin_type=True))
        self.last_send_ns = payload["stamp_ns"]


class WsHandler:
    def __init__(self, cfg: Config, teleop: DirectSimTeleop):
        self.cfg = cfg
        self.teleop = teleop

    async def handle(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=64 * 1024)
        await ws.prepare(request)
        peer = request.transport.get_extra_info("peername")
        log.info("keyboard ws connect from %s", peer)
        await self.teleop.prime(frames=max(10, self.cfg.prime_frames // 2))
        status_task = asyncio.create_task(self._status_loop(ws))
        count = 0
        last = time.monotonic()
        last_sent_idle_zero = True
        last_sent_inactive_zero = False
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if data.get("type") != "command":
                    continue
                state = CommandState(
                    active=bool(data.get("active", True)),
                    estop=bool(data.get("estop", False)),
                    mode="jog" if data.get("mode") == "jog" else "absolute",
                    offset=_vec3(data.get("offset")),
                    nudge=_vec3(data.get("nudge")),
                    jog=_vec3(data.get("jog")),
                    pan_delta=float(data.get("pan_delta", 0.0)),
                    gripper_delta=float(data.get("gripper_delta", 0.0)),
                    reanchor=bool(data.get("reanchor", False)),
                )
                if state.is_idle_zero and last_sent_idle_zero:
                    continue
                if not state.active and state.is_zero_motion and last_sent_inactive_zero:
                    continue
                await self.teleop.send(state)
                last_sent_idle_zero = state.is_idle_zero
                last_sent_inactive_zero = (not state.active and state.is_zero_motion)
                count += 1
                now = time.monotonic()
                if now - last >= 5.0:
                    log.info("browser command %.1f Hz", count / (now - last))
                    count = 0
                    last = now
        except asyncio.CancelledError:
            pass
        finally:
            status_task.cancel()
            log.info("keyboard ws disconnect")
        return ws

    async def _status_loop(self, ws: web.WebSocketResponse) -> None:
        while not ws.closed:
            ee = self.teleop.latest_ee.get(self.cfg.side)
            await ws.send_json(
                {
                    "type": "status",
                    "sim_host": self.cfg.sim_host,
                    "robot_id": self.cfg.robot_id,
                    "side": self.cfg.side,
                    "anchor_ready": self.teleop.anchor_ready,
                    "last_send_ns": self.teleop.last_send_ns,
                    "ee_xyz": ee[:3] if isinstance(ee, list) and len(ee) >= 3 else None,
                }
            )
            await asyncio.sleep(0.5)


def _vec3(value: Any) -> list[float]:
    if not isinstance(value, (list, tuple)):
        return [0.0, 0.0, 0.0]
    out = [0.0, 0.0, 0.0]
    for idx in range(min(3, len(value))):
        try:
            v = float(value[idx])
        except (TypeError, ValueError):
            v = 0.0
        if not math.isfinite(v):
            v = 0.0
        out[idx] = v
    return out


def _clamp_pose_to_workspace(side: str, pose: list[float]) -> list[float]:
    mount = ARM_MOUNT_OFFSET[side]
    delta = [float(pose[i]) - mount[i] for i in range(3)]
    radius = math.sqrt(sum(v * v for v in delta))
    if radius <= EE_REACH_RADIUS_M or radius <= 1e-9:
        return pose
    scale = EE_REACH_RADIUS_M / radius
    clamped = list(pose)
    for idx in range(3):
        clamped[idx] = mount[idx] + delta[idx] * scale
    return clamped


async def index(request: web.Request) -> web.Response:
    return web.Response(text=HTML, content_type="text/html")


async def health(request: web.Request) -> web.Response:
    cfg: Config = request.app["cfg"]
    teleop: DirectSimTeleop = request.app["teleop"]
    return web.json_response(
        {
            "ok": True,
            "sim_host": cfg.sim_host,
            "robot_id": cfg.robot_id,
            "side": cfg.side,
            "anchor_ready": teleop.anchor_ready,
            "all_anchors_ready": teleop.all_anchors_ready,
            "last_send_ns": teleop.last_send_ns,
            "latest_ee": teleop.latest_ee,
            "anchor_ee": teleop.anchor_ee,
            "target_offsets": teleop.target_offsets,
        }
    )


def _state_from_code(code: str, *, step_m: float, pan_step: float, grip_step: float) -> CommandState:
    state = CommandState(active=True, mode="jog")
    if code == "KeyW":
        state.nudge = [0.0, 0.0, +step_m]
    elif code == "KeyS":
        state.nudge = [0.0, 0.0, -step_m]
    elif code == "KeyR":
        state.nudge = [+step_m, 0.0, 0.0]
    elif code == "KeyF":
        state.nudge = [-step_m, 0.0, 0.0]
    elif code == "KeyA":
        state.pan_delta = -pan_step
    elif code == "KeyD":
        state.pan_delta = +pan_step
    elif code == "KeyZ":
        state.gripper_delta = -grip_step
    elif code == "KeyX":
        state.gripper_delta = +grip_step
    elif code == "Digit0":
        state.reanchor = True
    return state


async def api_nudge(request: web.Request) -> web.Response:
    cfg: Config = request.app["cfg"]
    teleop: DirectSimTeleop = request.app["teleop"]
    if request.content_type.startswith("application/json"):
        try:
            data = await request.json()
        except json.JSONDecodeError:
            data = {}
    else:
        data = dict(await request.post())
    code = str(data.get("code") or request.query.get("code") or "")
    frames = int(float(data.get("frames") or request.query.get("frames") or 5))
    hz = float(data.get("hz") or request.query.get("hz") or 60.0)
    step_m = float(data.get("step") or request.query.get("step") or DEFAULT_STEP_M)
    pan_step = float(data.get("pan_step") or request.query.get("pan_step") or 0.050)
    grip_step = float(data.get("grip_step") or request.query.get("grip_step") or 0.010)
    frames = max(1, min(frames, 30))
    hz = max(1.0, min(hz, 90.0))

    if not teleop.anchor_ready:
        return web.json_response(
            {"ok": False, "error": "anchor_not_ready", "robot_id": cfg.robot_id},
            status=409,
        )

    state = _state_from_code(code, step_m=step_m, pan_step=pan_step, grip_step=grip_step)
    if state.is_idle_zero:
        return web.json_response(
            {"ok": False, "error": "unsupported_code", "code": code},
            status=400,
        )
    sleep_s = 1.0 / hz
    await teleop.send(state)
    hold = CommandState(active=True, mode="jog")
    for _ in range(frames - 1):
        await teleop.send(hold)
        await asyncio.sleep(sleep_s)

    accepts = request.headers.getall("Accept", [])
    if any("text/html" in value for value in accepts):
        raise web.HTTPSeeOther(location=f"/?robot={cfg.robot_id}")
    return web.json_response(
        {
            "ok": True,
            "code": code,
            "frames": frames,
            "maintain_s": cfg.maintain_s,
            "robot_id": cfg.robot_id,
            "side": cfg.side,
            "ee_xyz": (teleop.latest_ee.get(cfg.side) or [None, None, None])[:3],
        }
    )


async def run(cfg: Config) -> None:
    teleop = DirectSimTeleop(cfg)
    app = web.Application()
    app["cfg"] = cfg
    app["teleop"] = teleop
    app.router.add_get("/", index)
    app.router.add_get("/health", health)
    app.router.add_post("/api/nudge", api_nudge)
    app.router.add_get("/ws", WsHandler(cfg, teleop).handle)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, cfg.host, cfg.port)
    tf_task: asyncio.Task | None = None
    maintain_task: asyncio.Task | None = None
    try:
        await site.start()
        log.info(
            "keyboard teleop page: http://%s:%d/?robot=%d",
            cfg.host,
            cfg.port,
            cfg.robot_id,
        )
        teleop.start()
        tf_task = asyncio.create_task(teleop.tf_loop())
        maintain_task = asyncio.create_task(teleop.maintain_loop())
        anchor_ok = await teleop.wait_for_anchor()
        if not anchor_ok:
            log.warning(
                "no tf.links anchor for robot=%d side=%s yet; page will keep waiting",
                cfg.robot_id,
                cfg.side,
            )
        else:
            log.info("anchor ready for robot=%d side=%s", cfg.robot_id, cfg.side)
            await teleop.prime()
            log.info(
                "primed IK targets with %d frame(s) before serving keyboard page",
                cfg.prime_frames,
            )
        while True:
            await asyncio.sleep(3600)
    finally:
        if tf_task is not None:
            tf_task.cancel()
        if maintain_task is not None:
            maintain_task.cancel()
        await runner.cleanup()
        teleop.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Serve a non-VR keyboard page that directly drives indory_isaac_sim EE targets."
    )
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port")
    parser.add_argument("--sim-host", default=DEFAULT_SIM_HOST)
    parser.add_argument("--sim-pub-port", type=int, default=5555)
    parser.add_argument("--sim-pull-port", type=int, default=5556)
    parser.add_argument("--robot-id", type=int, default=DEFAULT_ROBOT_ID)
    parser.add_argument("--side", choices=ARM_SIDES, default="right")
    parser.add_argument("--anchor-timeout", type=float, default=3.0)
    parser.add_argument(
        "--max-offset",
        type=float,
        default=DEFAULT_MAX_OFFSET_M,
        help="Clamp accumulated EE target offset to +/- this many meters per axis.",
    )
    parser.add_argument(
        "--prime-frames",
        type=int,
        default=120,
        help="Initial current-pose command frames to seed the sim IK slots.",
    )
    parser.add_argument(
        "--prime-interval",
        type=float,
        default=0.002,
        help="Seconds between initial IK prime frames.",
    )
    parser.add_argument(
        "--command-rate-hz",
        type=float,
        default=60.0,
        help="Background EE target maintain rate after a nudge.",
    )
    parser.add_argument(
        "--maintain-s",
        type=float,
        default=5.0,
        help="Seconds to keep streaming the latest EE target after each input.",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )
    cfg = Config(
        host=args.host,
        port=args.port,
        sim_host=args.sim_host,
        sim_pub_port=args.sim_pub_port,
        sim_pull_port=args.sim_pull_port,
        robot_id=args.robot_id,
        side=args.side,
        anchor_timeout_s=args.anchor_timeout,
        prime_frames=args.prime_frames,
        prime_interval_s=args.prime_interval,
        max_offset_m=args.max_offset,
        command_rate_hz=args.command_rate_hz,
        maintain_s=args.maintain_s,
    )
    try:
        asyncio.run(run(cfg))
    except OSError as e:
        log.error(
            "failed to start keyboard teleop on http://%s:%d: %s",
            cfg.host,
            cfg.port,
            e,
        )
        return 2
    except KeyboardInterrupt:
        log.info("interrupted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
