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
  - broad reach-sphere clamping constants.
- VR controller poses are input measurements. The sim-facing target is an
  anchor-relative controller delta converted into robot base-frame EE motion.
- Keyboard targets are also anchor-relative. Holding a key should generate
  command-frame motion, not wait for browser or OS key repeat.
- The public EE frame is the fixed-jaw tip. Do not compare direct tool results
  against the raw rigid body origin.
- Physical reach claims need both offline URDF/FK evidence and runtime
  command-to-`tf.links` evidence.

## Current Tools

- `tools/keyboard_teleop.py`: non-VR browser keyboard page that pushes direct
  sim EE targets.
- `tools/vr_direct_teleop.py`: WebXR `pose.<robot_id>` to direct sim bridge.
- `tools/sim_nudge.py`: one-shot direct EE movement smoke test.
- `tools/ee_target_verify.py`: waypoint accuracy verifier.
- `tools/keyboard_latency_probe.py`: `/api/nudge` to `tf.links` latency probe.
- `tools/reach_boundary_probe.py`: URDF joint-limited reach sampler.
