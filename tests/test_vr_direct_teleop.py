from __future__ import annotations

import importlib.util
import sys
from argparse import Namespace
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))


def _load_tool(name: str):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


vr_direct_teleop = _load_tool("vr_direct_teleop")
run_tool = _load_tool("run")


def test_vr_direct_defaults_match_webxr_pose_rate(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(sys, "argv", ["vr_direct_teleop.py"])

    args = vr_direct_teleop.parse_args()

    assert args.rate_hz == pytest.approx(90.0)
    assert args.feedback_rate_hz == pytest.approx(90.0)
    assert args.position_scale == pytest.approx(1.0)


def test_run_launcher_passes_vr_direct_rate():
    args = Namespace(
        pose_host="127.0.0.1",
        pose_port=7001,
        sim_host="127.0.0.1",
        sim_port=6655,
        sim_pull_port=6656,
        sim_rep_port=6657,
        sim_robot_id=0,
        vr_side="right",
        vr_rate_hz=90.0,
        vr_feedback_rate_hz=90.0,
        vr_position_scale=1.0,
        vr_grip_threshold=0.5,
        log_level="INFO",
        port=8443,
    )

    spec = run_tool._vr_direct_teleop_spec(args)

    assert "--rate-hz" in spec.argv
    assert spec.argv[spec.argv.index("--rate-hz") + 1] == "90.0"
    assert "--feedback-rate-hz" in spec.argv
    assert spec.argv[spec.argv.index("--feedback-rate-hz") + 1] == "90.0"
    assert "--sim-rep-port" in spec.argv
    assert spec.argv[spec.argv.index("--sim-rep-port") + 1] == "6657"


def test_run_launcher_passes_proxy_pose_only():
    args = Namespace(
        host="0.0.0.0",
        port=8443,
        signal_port=8444,
        zmq_addr="tcp://0.0.0.0:7001",
        stun=[],
        log_level="INFO",
        pose_only=True,
    )

    spec = run_tool._proxy_spec(args)

    assert "--pose-only" in spec.argv


def test_run_launcher_passes_keyboard_ports_and_teleop_rates():
    args = Namespace(
        keyboard_host="127.0.0.1",
        keyboard_port=8767,
        keyboard_side="left",
        keyboard_max_offset=0.80,
        keyboard_command_rate_hz=90.0,
        keyboard_feedback_rate_hz=90.0,
        sim_host="127.0.0.1",
        sim_port=6655,
        sim_pull_port=6656,
        sim_rep_port=6657,
        sim_robot_id=0,
        log_level="INFO",
    )

    spec = run_tool._keyboard_teleop_spec(args)

    assert "--sim-rep-port" in spec.argv
    assert spec.argv[spec.argv.index("--sim-rep-port") + 1] == "6657"
    assert "--feedback-rate-hz" in spec.argv
    assert spec.argv[spec.argv.index("--feedback-rate-hz") + 1] == "90.0"
    assert "--command-rate-hz" in spec.argv
    assert spec.argv[spec.argv.index("--command-rate-hz") + 1] == "90.0"
    assert "--max-offset" in spec.argv
    assert spec.argv[spec.argv.index("--max-offset") + 1] == "0.8"
    assert "--side" in spec.argv
    assert spec.argv[spec.argv.index("--side") + 1] == "left"


def test_vr_direct_full_scale_controller_delta_reaches_far_back_target():
    args = Namespace(
        source="zmq",
        robot_id=0,
        side="right",
        position_scale=1.0,
        grip_threshold=0.5,
        gripper_per_tick=0.01,
    )
    bridge = vr_direct_teleop.DirectVrBridge(args)
    bridge.state.latest_ee["right"] = [
        0.310208,
        -0.132998,
        0.898983,
        0.0,
        0.0,
        0.0,
        1.0,
    ]

    bridge._side_target(
        "right",
        {"pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "grip": 1.0, "buttons": {}},
    )
    target, _ = bridge._side_target(
        "right",
        {"pose": [0.0, 0.0, 0.73, 0.0, 0.0, 0.0, 1.0], "grip": 1.0, "buttons": {}},
    )

    assert target is not None
    assert target[:3] == pytest.approx([-0.419792, -0.132998, 0.898983])
