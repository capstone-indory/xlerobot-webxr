"""
Estimate the xlerobot arm's joint-limited EE reach from the URDF.

This is an offline kinematic probe. It does not prove controller convergence
by itself; it gives the physical target envelope that runtime ZMQ tests should
be compared against.

Typical run:

  python3 tools/reach_boundary_probe.py \
    --sim-repo ../indory_isaac_sim --side right \
    --anchor 0.310208,-0.132998,0.898983
"""

from __future__ import annotations

import argparse
import json
import math
import random
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from teleop_common import ARM_SIDES

CHAIN = {
    "right": [
        "arm_base_joint",
        "Rotation",
        "Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
        "Fixed_Jaw_tip_joint",
    ],
    "left": [
        "arm_base_joint_2",
        "Rotation_2",
        "Pitch_2",
        "Elbow_2",
        "Wrist_Pitch_2",
        "Wrist_Roll_2",
        "Fixed_Jaw_tip_joint_2",
    ],
}
MOVING_JOINTS = {
    "right": ("Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"),
    "left": ("Rotation_2", "Pitch_2", "Elbow_2", "Wrist_Pitch_2", "Wrist_Roll_2"),
}


@dataclass(frozen=True)
class JointModel:
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    movable: bool


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    # URDF fixed-axis RPY convention: Rz(yaw) * Ry(pitch) * Rx(roll).
    return _rot_z(rpy[2]) @ _rot_y(rpy[1]) @ _rot_x(rpy[0])


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=np.float64,
    )


def _transform(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = _rpy_matrix(rpy)
    out[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return out


def _joint_transform(axis: np.ndarray, angle: float) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = _axis_angle(axis, angle)
    return out


def _float_tuple(
    raw: str | None,
    length: int,
    default: tuple[float, ...],
) -> tuple[float, ...]:
    if raw is None:
        return default
    values = tuple(float(v) for v in raw.split())
    if len(values) != length:
        raise ValueError(f"expected {length} values, got {values}")
    return values


def _load_chain(urdf: Path, side: str) -> dict[str, JointModel]:
    root = ET.parse(urdf).getroot()
    joints = {j.attrib["name"]: j for j in root.findall("joint")}
    models: dict[str, JointModel] = {}
    for name in CHAIN[side]:
        joint = joints[name]
        origin = joint.find("origin")
        xyz = _float_tuple(
            origin.attrib.get("xyz") if origin is not None else None,
            3,
            (0.0, 0.0, 0.0),
        )
        rpy = _float_tuple(
            origin.attrib.get("rpy") if origin is not None else None,
            3,
            (0.0, 0.0, 0.0),
        )
        axis_node = joint.find("axis")
        axis = np.asarray(
            _float_tuple(
                axis_node.attrib.get("xyz") if axis_node is not None else None,
                3,
                (0.0, 0.0, 1.0),
            )
        )
        limit = joint.find("limit")
        movable = name in MOVING_JOINTS[side]
        if limit is not None and movable:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
        else:
            lower = upper = 0.0
        models[name] = JointModel(
            origin=_transform(xyz, rpy),
            axis=axis / np.linalg.norm(axis),
            lower=lower,
            upper=upper,
            movable=movable,
        )
    return models


def _fk(chain: dict[str, JointModel], side: str, q: dict[str, float]) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    for name in CHAIN[side]:
        model = chain[name]
        T = T @ model.origin
        if model.movable:
            T = T @ _joint_transform(model.axis, q[name])
    return T[:3, 3].copy()


def _parse_anchor(value: str | None) -> np.ndarray | None:
    if not value:
        return None
    vals = [float(v) for v in value.split(",")]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("--anchor must be x,y,z")
    return np.asarray(vals, dtype=np.float64)


def _round_vec(v: np.ndarray, digits: int = 6) -> list[float]:
    return [round(float(x), digits) for x in v.tolist()]


def _default_urdf(sim_repo: Path) -> Path:
    base = sim_repo / "src" / "indoory_isaac_sim" / "assets" / "data" / "robots"
    candidates = [
        base / "xlerobot" / "xlerobot.urdf",
        base / "xlerobot" / "xlerobot" / "xlerobot.urdf",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Estimate joint-limited xlerobot EE reach from URDF sampling.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--sim-repo", type=Path, default=Path("../indory_isaac_sim"))
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--side", choices=ARM_SIDES, default="right")
    parser.add_argument("--anchor", default=None, help="Optional current EE xyz as x,y,z.")
    parser.add_argument("--samples", type=int, default=300000)
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument("--y-tol", type=float, default=0.03)
    parser.add_argument("--z-tol", type=float, default=0.02)
    args = parser.parse_args()

    urdf = args.urdf
    if urdf is None:
        urdf = _default_urdf(args.sim_repo)
    chain = _load_chain(urdf, args.side)
    rng = random.Random(args.seed)
    anchor = _parse_anchor(args.anchor)

    best_x: tuple[np.ndarray, dict[str, float]] | None = None
    best_radius: tuple[float, np.ndarray, dict[str, float]] | None = None
    best_near_anchor: tuple[np.ndarray, dict[str, float]] | None = None

    moving = MOVING_JOINTS[args.side]
    for _ in range(max(1, args.samples)):
        q = {
            name: rng.uniform(chain[name].lower, chain[name].upper)
            for name in moving
        }
        p = _fk(chain, args.side, q)
        if best_x is None or p[0] > best_x[0][0]:
            best_x = (p, q)
        radius = float(np.linalg.norm(p))
        if best_radius is None or radius > best_radius[0]:
            best_radius = (radius, p, q)
        if anchor is not None:
            near_y = abs(float(p[1] - anchor[1])) <= args.y_tol
            near_z = abs(float(p[2] - anchor[2])) <= args.z_tol
            if near_y and near_z:
                if best_near_anchor is None or p[0] > best_near_anchor[0][0]:
                    best_near_anchor = (p, q)

    summary = {
        "side": args.side,
        "urdf": str(urdf),
        "samples": max(1, args.samples),
        "best_x": None
        if best_x is None
        else {"xyz": _round_vec(best_x[0]), "joints": best_x[1]},
        "best_radius_from_base_link_origin": None
        if best_radius is None
        else {
            "radius_m": round(best_radius[0], 6),
            "xyz": _round_vec(best_radius[1]),
            "joints": best_radius[2],
        },
    }
    if anchor is not None:
        summary["anchor_xyz"] = _round_vec(anchor)
        summary["near_anchor_tolerances"] = {"y_tol": args.y_tol, "z_tol": args.z_tol}
        summary["best_x_near_anchor_yz"] = (
            None
            if best_near_anchor is None
            else {
                "xyz": _round_vec(best_near_anchor[0]),
                "joints": best_near_anchor[1],
            }
        )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
