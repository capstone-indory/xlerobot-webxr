# CLAUDE.md

This repo owns the WebXR/Mac/browser side of teleop plus development tools that
can drive `indory_isaac_sim` directly. Keep this file aligned with
`../indory_isaac_sim/CLAUDE.md` when teleop architecture changes.

## Architecture Decisions

- The Mac proxy is transport only for WebXR poses. It publishes raw
  `local-floor` controller poses on `pose.<robot_id>` and should not add robot
  coordinate conversion.
- Direct sim command tools must use `tools/teleop_common.py` for shared ZMQ and
  wire-format behavior:
  - `tf.links.<robot_id>` parsing,
  - `gripper_right` / `gripper_left` target-name mapping,
  - `xlerobot_v1.1` absolute EE command payload construction,
  - optional command source metadata,
  - runtime RPC helpers such as `set_stream_rate`,
  - broad reach-sphere clamping constants.
- VR controller poses are input measurements. The sim-facing target is an
  anchor-relative controller delta converted into robot base-frame EE motion.
- Keyboard targets are also anchor-relative. Holding a key should generate
  command-frame motion, not wait for browser or OS key repeat.
- Browser keyboard sending must not combine `setTimeout` with
  `requestAnimationFrame`; that double-throttles a nominal 60 Hz loop toward
  30 Hz on common displays. Use a continuous rAF loop and throttle only by
  elapsed send time.
- Default browser keyboard motion is intentionally assertive:
  `step=0.10 m`, `speed=0.80 m/s`, `max_offset=0.80 m`. Operators can lower this via URL query
  params when working near workspace boundaries.
- Browser keyboard defaults to `hz=90` so a 90Hz rAF source can send every
  frame. With `hz=60` on a 90Hz display, rAF quantization can land at 45Hz.
- `/api/nudge` should reply immediately after the first command. The request
  handler must not sleep through a multi-frame hold loop; background maintain
  may emit only low-rate source-free hold heartbeats.
- Zero-motion keyboard states should send a stateless hold heartbeat, not a
  repeated full `arm_ee_pose_target`. Re-sending an unreachable full target at
  90 Hz can make the Home Server re-run projection repeatedly; a sparse hold
  heartbeat avoids re-targeting the arm. Hold/prime heartbeats are source-free
  so they do not keep refreshing the teleop lease after active motion stops.
- Browser physical `keydown` and `keyup` must send immediately. The rAF loop is
  for continuous held-key motion, not the first response to a key edge.
- Browser input regressions should be tested by executing the embedded HTML
  script, not only by checking source strings. A mock DOM/WebSocket VM smoke can
  prove that a real `keydown` dispatch creates the command payload before the
  next rAF callback.
- WebSocket connection setup must not block command receive on IK priming.
  Server startup primes the sim once; per-connection prime waits add first-key
  latency and can race with the first nudge.
- The public EE frame is the fixed-jaw tip. Do not compare direct tool results
  against the raw rigid body origin.
- Physical reach claims need both offline URDF/FK evidence and runtime
  command-to-`tf.links` evidence.
- When measuring keyboard/WebXR response, check the sim feedback stream rate.
  `tf.links` at 30 Hz can dominate first-motion measurements even when command
  send RTT is ~1 ms. Prefer the Home Server `--profile teleop` or runtime
  `set_stream_rate tf.links.<id> 90` for latency probes.
- Also check Home Server `stream_status`. A nominal 90 Hz client is not enough:
  a server-side loop pacing bug previously made observed sim/feedback cadence
  ~45 Hz while `env.step()` itself had enough headroom.
- `tools/keyboard_teleop.py` and `tools/vr_direct_teleop.py` now request
  `tf.links.<robot_id>` and `proprio.<robot_id>` at 90 Hz on startup by default.
  Use `--feedback-rate-hz 0` only when a run intentionally wants the sim default
  stream profile.
- `tools/vr_direct_teleop.py` and `tools/run.py --vr-direct-teleop` now send
  direct WebXR controller targets at 90 Hz by default, matching the page's
  `POSE_SEND_HZ=90`. Keeping this at 60 Hz made a 90 Hz Quest pose source feel
  quantized/slower before the command even reached the Home Server.
- `tools/mac_proxy.py` pose ZMQ PUB is freshness-oriented too:
  `SNDHWM=1`, `SNDTIMEO=0`, `LINGER=0`, and `send_multipart(..., NOBLOCK)`.
  A slow pose subscriber must drop old controller poses instead of building a
  queue that the direct bridge consumes late.
- Direct command tools should source-tag active movement commands with
  `source_id`, `source_role`, `priority`, and `lease_ms`. This lets the Home
  Server reject lower/same-priority competing writers instead of silently
  last-writer-wins overwriting a target.
- `tools/run.py` must pass the REP port to direct teleop children. Without
  `--sim-rep-port`, `tools/run.py --keyboard-teleop` and
  `--vr-direct-teleop` can connect PUB/PULL to a custom sim port set but still
  send `set_stream_rate` RPCs to the default `5557`, leaving feedback at the
  wrong cadence.
- Home Server now has optional bridge-side teleop target limits through
  `config/teleop_limits.yaml`; Mac/browser tooling should keep sending
  anchor-relative EE intent and let the bridge/sim report limiter and workspace
  projection telemetry instead of duplicating IK.

## Current Tools

- `tools/keyboard_teleop.py`: non-VR browser keyboard page that pushes direct
  sim EE targets.
- `tools/vr_direct_teleop.py`: WebXR `pose.<robot_id>` to direct sim bridge.
- `tools/sim_nudge.py`: one-shot direct EE movement smoke test.
- `tools/ee_target_verify.py`: waypoint accuracy verifier.
- `tools/keyboard_latency_probe.py`: `/api/nudge` to `tf.links` latency probe;
  prints sim-side decode projection debug when the server publishes it.
- `tools/keyboard_ws_latency_probe.py`: established WebSocket command to
  `tf.links` latency probe for the physical-key path; also prints decode
  projection debug when present.
- `tools/reach_boundary_probe.py`: URDF joint-limited reach sampler.
- `tools/teleop_probe.py`: Mac `/ws` -> `pose.<robot_id>` acceptance probe. It
  can inject synthetic WebXR page payloads, or run `--live-quest` to measure an
  already connected Quest pose stream, live sim `tf.links` arm motion, pose
  observed Hz, Mac-publish-to-subscriber age, and a JSON summary artifact.

## Completion Audit

Status on 2026-05-20: this repo's browser/direct-bridge software path now
matches the Home Server teleop contract, but do not mark the cross-repo goal
complete until the remaining physical Quest/device gate is run or explicitly
waived.

- Browser keyboard first response is covered by Node VM tests that execute the
  embedded page script. A physical `keydown` sends the command before the next
  rAF callback, `keyup` sends the stop edge, and later idle rAF ticks do not
  keep emitting 90 Hz zero frames.
- Held keyboard motion is covered by both JS tests and runtime probes. The page
  sends `speed * dt` nudges at the configured cadence while a key is held, and
  `tools/keyboard_ws_latency_probe.py --hold-s` verifies requested/projected
  movement, final target error, and send drops against `tf.links`.
- Delta/workspace limits are no longer the old bottleneck. Browser defaults are
  `step=0.10 m`, `speed=0.80 m/s`, `max_offset=0.80 m`; right and left runtime
  probes reached about `0.73 m` far-back motion, and held probes reached their
  projected targets with near-zero error.
- Feedback cadence is covered by startup RPCs and launcher tests. Keyboard and
  direct-VR tools request 90 Hz `tf.links`/`proprio`; `tools/run.py` forwards
  custom sim ports plus feedback/rate arguments so non-default port sets do not
  silently leave feedback at 30 Hz.
- WebXR/direct bridge coverage is closed for software-only testing: the page
  keeps `POSE_SEND_HZ=90`, `tools/vr_direct_teleop.py` defaults to 90 Hz, the
  launcher passes `--rate-hz 90.0`, Mac proxy pose PUB is freshness-oriented,
  and a synthetic direct-VR runtime run moved the right EE while the Home Server
  held 90 Hz feedback streams.
- Remaining gate: this workspace still has not run a live Quest Browser/WebXR
  session or a real hardware arm acceptance test. A final device check should
  verify first-key/controller response, held motion, stream cadence, and
  far-reach behavior before the overall goal is closed. The current live-device
  command is `tools/teleop_probe.py --live-quest --listen-s 5 --pose-min-hz 80
  --arm-threshold 0.10 --summary-json ...`, paired with Home Server
  `stream_status` and the existing keyboard probes.

## Latest Verification

- On a headless `indory_isaac_sim` vr-mode server at `127.0.0.1:6655/6656/6657`,
  `/api/nudge` through `tools/keyboard_teleop.py` on port `8766`, running under
  `conda run -n leisaac`, reached a reachable `KeyR step=0.035 hz=90` target
  exactly.
- After removing the blocking hold loop from `/api/nudge`,
  `tools/keyboard_latency_probe.py` reported `max_move_m=0.035001` and HTTP
  POST round trip `1.62 ms` for `KeyR step=0.035 hz=90`; first observed
  `tf.links` motion was `57.23 ms` after the probe began watching.
- With the new `max_offset=0.80`, `/api/nudge KeyF step=0.73 hz=90` moved the
  right EE `0.729341 m` through the keyboard path; HTTP POST RTT was `1.02 ms`.
- Established WebSocket key-path probe:
  `tools/keyboard_ws_latency_probe.py --code KeyR --step 0.035 --warm-s 0.25`
  reported `ws_rtt_ms=0.07`, `max_move_m=0.035001`, and first observed
  `tf.links` move `56.87 ms` after watching began.
- Established WebSocket far-reach probe:
  `tools/keyboard_ws_latency_probe.py --code KeyF --step 0.73 --warm-s 0.25`
  reported `ws_rtt_ms=0.08`, `max_move_m=0.731692`, and final EE near
  `[-0.419708, -0.132984, 0.899000]`.
- Direct VR targets now use the shared broad reach-sphere clamp before PUSHing
  to the sim, matching keyboard direct tooling.
- Follow-up: Home Server `examples/vr_teleop_bridge.py` now clamps coarse
  workspace overruns instead of dropping the target side. The browser keyboard
  page and `/api/nudge` default send/maintain cadence now use 90Hz.
- Follow-up: Home Server decoder now applies URDF joint-limited workspace
  projection after the broad reach-sphere clamp. A same-height forward target
  that is inside the sphere but outside true reach is converted to the nearest
  reachable xyz before IsaacLab IK sees it.
- Follow-up: Home Server projection now uses current-joint seed as the fast
  path, then deterministic cached FK-nearest multi-start refinement if the
  first solve misses. Browser/Mac tools should keep sending EE targets, not add
  a separate client-side IK solver.
- Follow-up: Home Server action term now applies the position-only solver's
  joint solution as the articulation target by default. Browser/Mac tools
  should still send anchor-relative EE targets continuously.
- Follow-up: Home Server VR-mode now disables arm self-collision because the
  xlerobot SRDF collision-disable pairs are not applied to the USD articulation.
- Follow-up: Home Server `tf.links` now carries optional `vr_decode_debug`;
  keyboard latency probes summarize requested xyz, projected xyz, residual, and
  whether the decoder changed the target.
- Follow-up: keyboard and direct-VR PUSH sockets are now freshness-oriented:
  `SNDHWM=1`, `SNDTIMEO=0`, `IMMEDIATE=1`, non-blocking send. A full sim-side
  PULL queue drops an old frame instead of blocking the browser/WebSocket input
  path behind stale commands.
  This lets physically valid arm folds reach instead of stopping early in
  PhysX.
- Follow-up: keyboard defaults now use the newly verified workspace:
  `step=0.10`, `speed=0.80`, `max_offset=0.80`. The old `0.30 m` per-axis cap
  prevented browser keyboard from using the far-back reach that the sim now
  supports.
- Follow-up: `/api/nudge` no longer waits for the requested hold frames before
  returning; it sends the first command immediately and relies on background
  maintain streaming for hold.
- Follow-up: physical keyboard `keydown` now calls `sendCommand(0)` immediately
  for first key edges, and `keyup` immediately sends a stop/hold frame. The
  browser no longer waits for the next rAF tick before the first keyboard
  command.
- Runtime follow-up: the same browser `/api/nudge` path with
  `KeyR step=0.095 hz=90` requested an unreachable same-height forward target
  from reset and converged to about `[0.348572, -0.133002, 0.896536]`, matching
  the Home Server projection behavior instead of leaving the old target active.
- Clean sequential probes after the latest Home Server direct-joint solver:
  `tools/keyboard_ws_latency_probe.py --code KeyR --step 0.035 --warm-s 0.25`
  moved `0.035117 m`, first observed motion `31.10 ms`, and decoder debug
  reported no projection. Then
  `tools/keyboard_latency_probe.py --code KeyF --step 0.035 --hz 90` moved
  `0.035066 m`, returned from HTTP in `0.92 ms`, first observed motion
  `49.82 ms`, and decoder debug again reported no projection.
- Follow-up: `WsHandler.handle` no longer awaits `teleop.prime()` before
  processing browser commands. This removes the default ~120 ms connection-time
  delay from the first WebSocket command path; startup priming still happens in
  `run()` after anchor acquisition.
- Follow-up: Home Server decode debug may include `projection_mode`, including
  `directional_segment` when the solver projects an unreachable target along
  the operator's requested direction before falling back to nearest workspace
  projection. Probes now include this field in summaries.
- Runtime follow-up: with Home Server `tf.links/proprio` still at 30 Hz,
  `tools/keyboard_ws_latency_probe.py --code KeyF --step 0.035 --warm-s 0.0`
  reported `ws_rtt_ms=0.02`, first observed motion `48.94 ms`, and
  `max_move_m=0.034970`. After raising `tf.links.0` and `proprio.0` to 90 Hz,
  the same WebSocket path reported `ws_rtt_ms=0.01`, first observed motion
  `28.98 ms`, and `max_move_m=0.035001`. The command path is fast; feedback
  sampling cadence was a remaining measured bottleneck.
- Follow-up: direct keyboard and direct-VR tools now set 90 Hz feedback stream
  rates at startup, and payloads from active direct tools include command source
  metadata. The Home Server can now report/resolve competing sources through its
  command-source lease arbitration.
- Follow-up: `tools/run.py` now forwards `--sim-rep-port`,
  `--keyboard-feedback-rate-hz`, `--keyboard-command-rate-hz`,
  `--keyboard-max-offset`, and `--vr-feedback-rate-hz` into the direct tools.
  Tests assert the launcher passes custom `6655/6656/6657` sim port sets and
  keeps keyboard/VR direct feedback at 90 Hz.
- Verification for this follow-up:
  `conda run -n leisaac python -m pytest tests/test_keyboard_teleop.py -q`
  passed with 14 tests after adding source metadata and feedback-rate coverage.
  `pytest.ini` restricts collection to `tests/` so the heavy dev smoke script
  under `tools/_smoke_test.py` is not collected without its optional
  `aiortc`/`av` stack.
- Latest Home Server runtime after the `RateLimiter` catch-up fix:
  `stream_status` reported observed loop `89.47 Hz` with
  `tf.links.0`/`proprio.0` targets at `90 Hz`; `noop_client.py --jsonl`
  measured `tf.links.0` at about `89.99 Hz` and `proprio.0` at about
  `90.0 Hz`, p95 age `0.1 ms`. The established WebSocket key path
  `tools/keyboard_ws_latency_probe.py --code KeyF --step 0.035 --warm-s 0.0`
  reported `ws_rtt_ms=0.02`, first observed move `15.67 ms`, and
  `max_move_m=0.035009`.
- Latest camera-enabled Home Server runtime after the teleop stream/render
  profile and zero-motion heartbeat fixes:
  `stream_status` reported observed loop `88.24-89.48 Hz`, avg env step
  `7.49-7.80 ms`, `tf.links.0`/`proprio.0` at `90 Hz`, RGB at `15 Hz`, and
  depth/lidar disabled. `noop_client.py --jsonl --profile teleop` measured
  `proprio.0` at `89.91 Hz`, `tf.links.0` at `89.41 Hz`, RGB streams at
  `14.90 Hz`, and p95 control-stream age `0.1 ms`.
- Latest established WebSocket key-path probes on that camera-enabled server:
  `tools/keyboard_ws_latency_probe.py --code KeyF --step 0.100 --warm-s 0.0`
  moved `0.099119 m`, first observed motion `21.20 ms`, and decoder debug
  reported `projection_mode="reachable_solve"`. The next
  `--code KeyR --step 0.300` requested over-forward
  `[0.412443, -0.132998, 0.877924]`, Home Server projected it to
  `[0.349214, -0.133005, 0.878212]` with
  `projection_mode="best_effort_directional"`, and measured final EE near
  `[0.349128, -0.132400, 0.879358]`, first motion `29.90 ms`.
- Home Server same-batch command merge now preserves a tap nudge's
  `arm_ee_pose_target` if a later sparse keyboard hold heartbeat arrives in
  the same drain batch. This pairs with the xlerobot zero-motion heartbeat
  change so a stop/idle frame cannot cancel the nudge that immediately
  preceded it.
- Home Server runtime follow-up for that merge: a direct probe sent a terminal
  keyboard-style `f` target and then a same-source sparse frame with no
  `arm_ee_pose_target` using a reliable two-frame send. The arm still moved
  `0.099119 m` toward the `0.100000 m` projected target with target error
  `0.000942 m`, and after `lease_ms` the Home Server reported
  `active_source_id=null`, `last_rejected_source_id=null`, and
  `held_has_v11_relative_delta=false`.
- Follow-up: zero-motion and prime hold heartbeats are now source-free and the
  background hold heartbeat is capped at `5 Hz`. Active motion commands still
  carry source metadata and refresh the lease, but idle/hold traffic no longer
  extends a keyboard lease or pushes 90 Hz sparse no-op frames into the sim.
- Runtime follow-up after suppressing hold heartbeats during an active source
  lease: on the same camera-enabled teleop server, WebSocket `KeyF step=0.100`
  moved `0.099119 m` with `ws_rtt_ms=0.01` and first observed motion
  `14.17 ms`; the following `KeyR step=0.300` projected to
  `[0.349214, -0.133005, 0.878212]` and reached the same boundary with first
  motion `22.54 ms`. After waiting beyond `lease_ms`, Home Server
  `command_status` reported `active_source_id=null` and
  `last_rejected_source_id=null`, so idle keyboard traffic no longer leaves a
  misleading legacy rejection.
- Follow-up verification: `tests/test_keyboard_teleop.py` now executes the
  actual embedded browser script in Node with a mock DOM/WebSocket. Dispatching
  a physical `KeyR` `keydown` sends a WebSocket command immediately, before any
  queued `requestAnimationFrame` callback, with `nudge=[0.1, 0, 0]`; `keyup`
  sends the zero nudge edge immediately as well.
- Far-back runtime follow-up after the Home Server fast-projection emergency
  seed fix: on a camera-enabled 90Hz teleop server, the WebSocket key probe
  `--keyboard-url http://127.0.0.1:8766 --code KeyF --step 0.730 --warm-s 0.0`
  moved from `[0.310208, -0.132998, 0.898983]` to
  `[-0.417655, -0.132581, 0.899525]` with `max_move_m=0.730415`,
  `ws_rtt_ms=0.02`, and decoder debug
  `projection_mode="reachable_emergency_seed"`, `changed=false`,
  requested/target xyz both `[-0.419792, -0.132998, 0.898983]`.
- Follow-up after Home Server workspace cache prewarm: the same far-back
  WebSocket key probe kept `max_move_m=0.730415` and improved first observed
  motion from `94.43 ms` to `46.11 ms` with `ws_rtt_ms=0.01`. This points to
  seed-cloud cold-start cost as one latency component for the first large
  boundary nudge; normal small-key probes remain faster.
- Probe follow-up: `tools/keyboard_ws_latency_probe.py` and
  `tools/keyboard_latency_probe.py` now report `first_cmd_echo_ms` from
  `tf.links.cmd_echo_stamp_ns`, separately from first EE motion. Re-running the
  far-back WebSocket probe after this instrumentation reported
  `ws_rtt_ms=0.02`, `first_cmd_echo_ms=35.3`, `first_move_ms=35.07`, and
  `max_move_m=0.730415`; the remaining latency is therefore sim step/publish
  observation cadence plus physical tracking, not browser WebSocket send time.
- Held-key probe follow-up: `tools/keyboard_ws_latency_probe.py` now supports
  `--hold-s`, `--hold-hz`, and `--speed` to mimic browser rAF held-key nudge
  frames. A 0.5s `KeyF` hold initially requested about `0.500 m` but the Home
  Server projected the target sideways/upward and moved only `0.454 m`. After
  widening the Home Server backward residual emergency path, the same probe
  reported `requested_nudge_m=0.500306`, `max_move_m=0.502139`,
  `send_drop_delta=0`, and decoder debug `changed=false`,
  `projection_mode="reachable_solve"`.
- Held-key probe now reports `projected_target_move_m`, `target_error_m`,
  `move_to_projected_ratio`, and `move_to_requested_ratio`. On the camera-enabled
  90Hz teleop server, 0.5s hold probes with `step=0.1`, `speed=0.8` reported:
  `KeyF` requested/projected `0.500424 m`, moved `0.500551 m`, target error
  `0.000404 m`; `KeyR` was workspace-projected to `0.072173 m` and moved
  `0.072548 m`; `KeyW` projected to `0.358042 m` and moved `0.360170 m`;
  `KeyS` projected to `0.213679 m` and moved `0.214004 m`. All had
  `send_drop_delta=0` and move-to-projected ratios about `1.00`.
- Home Server `examples/ik_fixture_suite.py` now treats `proprio.<id>` joint
  tracking as first-class runtime evidence, not only final `tf.links` pose.
  The latest camera-enabled 90Hz run passed 12/12 fixtures with
  `joint_tracking_ok=true` and max arm IK joint target error `<=0.000011 rad`,
  while preserving the broad right/left far-back and overreach coverage.
- Left-side keyboard bridge runtime was checked with
  `tools/keyboard_teleop.py --side left` on port `8767`, resetting and
  re-anchoring before each probe. WebSocket `KeyF step=0.730` moved the left EE
  `0.730433 m`, with `first_cmd_echo_ms=47.15`, `first_move_ms=46.94`,
  `send_drop_delta=0`, and decoder debug
  `projection_mode="reachable_emergency_seed"`, `changed=false`. A 0.5s held
  `KeyF` requested `0.500131 m`, moved `0.499895 m`, and had target error
  `0.000241 m`; a 0.5s held `KeyR` was projected to `0.072444 m` and moved
  `0.072821 m` with target error `0.0`. The camera-enabled server stayed at
  `observed_loop_hz=89.669`, 90Hz control streams, 15Hz RGB, and zero drops.
- Direct WebXR/OpenXR bridge follow-up: the page already sends
  `POSE_SEND_HZ=90`, but `tools/vr_direct_teleop.py` and the launcher path were
  still defaulting to 60 Hz. They now default to 90 Hz, and
  `tools/run.py --vr-direct-teleop` passes `--rate-hz 90.0` explicitly. A
  synthetic WebXR pose runtime check against the camera-enabled 90Hz Home
  Server used `tools/vr_direct_teleop.py --source synthetic --rate-hz 90
  --synthetic-radius-m 0.20 --duration-s 5.0`; it set `tf.links.0` and
  `proprio.0` feedback to 90 Hz and moved the right EE
  `max_right_move_m=0.404109`. A `noop_client.py --profile teleop` sample after
  that measured `proprio.0=89.99 Hz`, `tf.links.0=89.99 Hz`, p95 age `0.1 ms`.
- Pose transport follow-up: `tools/mac_proxy.py` now publishes
  `pose.<robot_id>` with HWM 1 and non-blocking send, tracking `send_drops`.
  `tests/test_mac_proxy_pose_transport.py` also guards that the WebXR page has
  one timestamp field in the pose payload and keeps `POSE_SEND_HZ=90`.
- Browser JS follow-up: the Node VM smoke now executes the embedded HTML rAF
  callback while `KeyF` remains pressed. It verifies the physical key edge sends
  the immediate `-0.100 m` nudge, then rAF ticks at 12 ms produce held nudges of
  `-0.0096 m` each from `speed=0.800 m/s`, without relying on OS key-repeat.
- Browser idle follow-up: the rAF loop now gates sends on `hasPendingCommand()`.
  After keyup, the browser still sends the one zero edge but does not keep
  sending 90Hz idle zero WebSocket frames. The Node VM test advances additional
  rAF ticks after keyup and verifies `sendCount` does not increase.
- Quest acceptance tooling follow-up: `tools/teleop_probe.py` now has
  `--live-quest`, `--pose-min-hz`, `--listen-s`, and `--summary-json`. Live mode
  does not inject synthetic `/ws` frames; it measures an already connected Quest
  `pose.<robot_id>` stream rate/schema/age and, when the Home Server bridge is
  running, the sim `tf.links` max arm motion while the operator moves the
  gripped controller. This does not replace the missing physical-device run; it
  makes that remaining gate executable and auditable.
