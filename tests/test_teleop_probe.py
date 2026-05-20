from __future__ import annotations

import importlib.util
import sys
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


teleop_probe = _load_tool("teleop_probe")


def test_pose_stream_metrics_reports_rate_and_age_percentiles():
    recv_times_ns = [1_000_000_000 + i * 11_111_111 for i in range(91)]
    ages_ms = [0.1, 0.2, 0.3, 2.0, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]

    metrics = teleop_probe._pose_stream_metrics(recv_times_ns, ages_ms)

    assert metrics["count"] == 91
    assert metrics["observed_hz"] == pytest.approx(90.0, rel=1e-5)
    assert metrics["recv_span_s"] == pytest.approx(0.99999999, rel=1e-6)
    assert metrics["mac_to_sub_age_ms_avg"] == pytest.approx(0.65)
    assert metrics["mac_to_sub_age_ms_p95"] == pytest.approx(2.0)
    assert metrics["mac_to_sub_age_ms_max"] == pytest.approx(2.0)
    assert metrics["age_samples"] == len(ages_ms)


def test_pose_stream_metrics_handles_missing_samples():
    metrics = teleop_probe._pose_stream_metrics([], [])

    assert metrics == {
        "count": 0,
        "recv_span_s": None,
        "observed_hz": None,
        "mac_to_sub_age_ms_avg": None,
        "mac_to_sub_age_ms_p95": None,
        "mac_to_sub_age_ms_max": None,
        "age_samples": 0,
    }


def test_teleop_probe_exposes_live_quest_acceptance_flags():
    source = (TOOLS / "teleop_probe.py").read_text()

    assert "--live-quest" in source
    assert "--pose-min-hz" in source
    assert "--summary-json" in source
    assert "probe_live_arm_motion" in source
