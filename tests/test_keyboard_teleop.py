from __future__ import annotations

import importlib.util
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


def _cfg(max_offset_m: float = 0.30):
    return keyboard_teleop.Config(
        host="127.0.0.1",
        port=8765,
        sim_host="127.0.0.1",
        sim_pub_port=5555,
        sim_pull_port=5556,
        robot_id=0,
        side="right",
        anchor_timeout_s=0.1,
        prime_frames=0,
        prime_interval_s=0.0,
        max_offset_m=max_offset_m,
        command_rate_hz=60.0,
        maintain_s=1.0,
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


def test_common_tf_pose_parser_rejects_nonfinite_values():
    msg = {
        "targets": [
            {"name": "gripper_right", "pose": [1, 2, 3, 0, 0, 0, 1]},
            {"name": "gripper_left", "pose": [1, 2, float("nan"), 0, 0, 0, 1]},
        ]
    }

    poses = teleop_common.extract_tf_poses(msg)

    assert poses == {"right": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]}
