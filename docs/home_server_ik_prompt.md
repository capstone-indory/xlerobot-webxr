# Home Server Prompt: Fix right-arm EE target IK accuracy

You are working on the Home Server / Isaac Sim side.

The Mac/Web repo now has a reproducible EE target accuracy verifier. Clone and
checkout this branch, then use the verifier to reproduce the IK failure regions
against the live `indory_isaac_sim` server.

## Repos

```bash
git clone https://github.com/capstone-indory/xlerobot-webxr
cd xlerobot-webxr
git checkout codex/webxr-teleop-checkpoint
```

Also use the sim repo:

```bash
git clone https://github.com/capstone-indory/indory_isaac_sim
cd indory_isaac_sim
```

The sim must be running in VR mode:

```bash
python examples/rpc_client.py --host 127.0.0.1 fleet_info
```

Expected:

```json
{
  "ok": true,
  "vr_mode": true,
  "command_schema": "xlerobot_v1.1",
  "action_dim_per_robot": 23
}
```

## Problem

Mac/Web keyboard teleop now sends anchor-relative EE target nudges correctly.
The command path is not the suspected failure point anymore:

- The sim PULL channel is last-writer-wins per `robot_id`.
- The verifier runs with a single writer.
- Center/return targets often pass exactly.
- Certain right-arm EE target directions fail to converge within tolerance.

The current failure pattern was measured from Mac against sim host
`100.80.87.68`, robot `0`, side `right`:

- Anchor: `[-0.009399, -0.215037, 0.926106]`
- `x_plus` `(+0.03, 0, 0)`: final error about `0.0214m`
- `x_minus` `(-0.03, 0, 0)`: final error about `0.0214m`
- `z_plus` `(0, 0, +0.03)`: final error about `0.0300m`, no visible movement
- `z_minus` `(0, 0, -0.03)`: passes
- `y_plus/y_minus` `(0, +/-0.02, 0)`: final error about `0.014m`
- Worst case `x_minus_z_plus` `(-0.03, 0, +0.03)`: final error about `0.0763m`, no movement

Full report files are committed in:

```text
tools/reports/ee_target_verify_right_full.csv
tools/reports/ee_target_verify_right_full.jsonl
```

## Reproduce

From the `xlerobot-webxr` checkout:

```bash
python3 -m pip install -r tools/requirements.txt

python3 tools/ee_target_verify.py \
  --sim-host 127.0.0.1 \
  --robot-id 0 \
  --side right \
  --settle-s 4.0 \
  --observe-s 1.0 \
  --tolerance 0.012 \
  --out tools/reports/ee_target_verify_right_home
```

If the sim is not local, replace `--sim-host` with the Tailscale/LAN address.

Before running, make sure there are no other command writers for the same robot:

```bash
pgrep -fl 'keyboard_teleop|vr_direct_teleop|vr_teleop_bridge|tools/run.py'
```

## What to inspect in `indory_isaac_sim`

Start with these files:

```text
src/indoory_isaac_sim/env/build.py
src/indoory_isaac_sim/wire/command_decode.py
src/indoory_isaac_sim/wire/schema.py
examples/keyboard_client.py
```

Focus areas:

1. Right arm `DifferentialInverseKinematicsActionCfg` in VR mode.
2. Right EE body name and joint set:
   - `EE_BODY_NAMES["right"]`
   - `ARM_IK_JOINT_NAMES["right"]`
3. Base-frame to articulation-root transform in `_wire_ee_pose_to_root_action`.
4. Whether the 4-DoF right arm is being over-constrained by the 7-D pose target.
5. Whether `+Z` targets near the measured anchor are outside the practical IK
   workspace even though they pass the coarse reach sphere.
6. Whether the orientation component should be ignored, relaxed, or regenerated
   for position-only teleop.
7. Whether the IK controller config needs damping/gain/iteration changes or a
   target clamp projected to the reachable manifold.

## Success criteria

After sim-side changes, rerun:

```bash
python3 tools/ee_target_verify.py \
  --sim-host 127.0.0.1 \
  --robot-id 0 \
  --side right \
  --settle-s 4.0 \
  --observe-s 1.0 \
  --tolerance 0.012 \
  --out tools/reports/ee_target_verify_right_after_fix
```

Target outcome:

- All default right-arm waypoints pass, or
- Any remaining failures are explained by a documented reachable-workspace clamp
  and the verifier is updated to report them as expected clipped targets.

Do not change the Mac/Web command protocol unless the sim-side investigation
proves the wire target frame or pose convention is wrong. The current Mac
payload is `xlerobot_v1.1` with `arm_ee_pose_target[side].pose` in base frame,
mode `absolute`, frame `base`, and continuously refreshed live orientation.
