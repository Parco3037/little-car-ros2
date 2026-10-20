"""Measure the original MentorPi visual envelope for an explicit joint pose.

This is an offline CAD calculation.  It reads only the preserved vendor Xacro
and STL files; it does not start ROS nodes or command any actuator.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np


XACRO_EXPRESSION = re.compile(r"\$\{([^{}]+)\}")
JOINT_DEFAULTS = {
    "joint1": 0.0,
    "joint2": 0.9634217471008698,
    "joint3": -1.5498523757709646,
    "joint4": -1.6755160819145563,
    "joint5": 0.0,
    "gripper_base_joint": 0.0,
    "gripper_left_join": 0.0,
    "gripper_right_joint": 0.0,
    "left_jaw_joint": 0.0,
    "right_jaw_joint": 0.0,
}


def _safe_number(expression: str) -> float:
    """Evaluate the numeric-only arithmetic used by the preserved Xacro."""
    tree = ast.parse(expression, mode="eval")
    allowed = (
        ast.Expression,
        ast.Constant,
        ast.UnaryOp,
        ast.UAdd,
        ast.USub,
        ast.BinOp,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
    )
    if any(not isinstance(node, allowed) for node in ast.walk(tree)):
        raise ValueError(f"Unsupported Xacro expression: {expression!r}")
    return float(eval(compile(tree, "<xacro-number>", "eval"), {"__builtins__": {}}, {}))


def _expand_numbers(value: str) -> str:
    return XACRO_EXPRESSION.sub(lambda match: repr(_safe_number(match.group(1))), value)


def _vector(value: str | None, default: str) -> np.ndarray:
    return np.asarray([float(item) for item in _expand_numbers(value or default).split()])


def _origin(element: ET.Element | None) -> np.ndarray:
    result = np.eye(4)
    if element is None:
        return result
    result[:3, 3] = _vector(element.get("xyz"), "0 0 0")
    roll, pitch, yaw = _vector(element.get("rpy"), "0 0 0")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result[:3, :3] = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return result


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm == 0:
        if angle != 0:
            raise ValueError("Non-zero rotation around a zero axis")
        return np.eye(4)
    x, y, z = axis / norm
    c, s, one_minus_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = np.eye(4)
    result[:3, :3] = (
        (c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s),
        (y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s),
        (z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c),
    )
    return result


def _joint_motion(joint: ET.Element, value: float) -> np.ndarray:
    kind = joint.get("type")
    if kind == "fixed":
        return np.eye(4)
    axis = _vector(joint.find("axis").get("xyz"), "1 0 0")
    if kind in ("continuous", "revolute"):
        return _axis_rotation(axis, value)
    if kind == "prismatic":
        result = np.eye(4)
        result[:3, 3] = axis / np.linalg.norm(axis) * value
        return result
    raise ValueError(f"Unsupported joint type {kind!r}")


def _stl_vertices(raw: bytes) -> np.ndarray:
    if len(raw) < 84:
        raise ValueError("Not a binary STL")
    count = struct.unpack_from("<I", raw, 80)[0]
    if count == 0 or len(raw) != 84 + 50 * count:
        raise ValueError("Binary STL size mismatch")
    triangle = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    points = np.frombuffer(raw, dtype=triangle, offset=84, count=count)["vertices"].reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError("Non-finite STL vertices")
    return points


def _convex_hull_xy(points: np.ndarray) -> list[list[float]]:
    """Return the counter-clockwise 2-D convex hull without duplicate closure."""
    unique = sorted({(float(x), float(y)) for x, y in points[:, :2]})
    if len(unique) < 3:
        raise ValueError("At least three distinct planar vertices are required")

    def cross(origin, first, second):
        return ((first[0] - origin[0]) * (second[1] - origin[1])
                - (first[1] - origin[1]) * (second[0] - origin[0]))

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-12:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-12:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError("Planar vertices are collinear")
    return [[x, y] for x, y in hull]


def _support_envelope_xy(points: np.ndarray, sides: int = 16) -> list[list[float]]:
    """Return a small convex polygon that conservatively encloses all XY vertices."""
    if sides < 4 or sides % 4:
        raise ValueError("Support envelope side count must be a multiple of four")
    angles = np.arange(sides, dtype=float) * (2.0 * math.pi / sides)
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    support = np.max(points[:, :2] @ normals.T, axis=0)
    vertices = []
    for index in range(sides):
        following = (index + 1) % sides
        matrix = np.vstack((normals[index], normals[following]))
        vertex = np.linalg.solve(matrix, np.asarray((support[index], support[following])))
        vertices.append([float(vertex[0]), float(vertex[1])])
    candidate = np.asarray(vertices)
    # Every source vertex must remain inside every support half-plane.
    if np.any(points[:, :2] @ normals.T > support + 1e-10):  # pragma: no cover
        raise ValueError("Support envelope does not contain the source model")
    if not np.isfinite(candidate).all():  # pragma: no cover
        raise ValueError("Non-finite support envelope")
    return vertices


def measure(package: Path, joint_positions: dict[str, float]) -> dict:
    sources = {
        "mecanum": package / "urdf/mecanum.xacro",
        "arm": package / "urdf/arm.urdf.xacro",
        "gripper": package / "urdf/gripper.urdf.xacro",
    }
    roots = {group: ET.fromstring(path.read_bytes()) for group, path in sources.items()}
    links: dict[str, tuple[str, ET.Element]] = {}
    joints: list[ET.Element] = []
    for group, root in roots.items():
        for link in root.findall("link"):
            if link.get("name") in links:
                raise ValueError(f"Duplicate link {link.get('name')}")
            links[link.get("name")] = (group, link)
        joints.extend(root.findall("joint"))

    poses = {"base_footprint": np.eye(4)}
    pending = list(joints)
    while pending:
        old_count = len(pending)
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in poses:
                continue
            value = float(joint_positions.get(joint.get("name"), 0.0))
            poses[child] = poses[parent] @ _origin(joint.find("origin")) @ _joint_motion(joint, value)
            pending.remove(joint)
        if len(pending) == old_count:
            unresolved = [joint.get("name") for joint in pending]
            raise ValueError(f"Unresolved joint tree: {unresolved}")

    all_points: list[np.ndarray] = []
    parts: list[dict] = []
    for link_name, (group, link) in links.items():
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError(f"Expected a mesh visual for {link_name}")
            filename = mesh.get("filename")
            if not filename.startswith("${mesh_dir}/"):
                raise ValueError(f"Unexpected mesh URI {filename!r}")
            mesh_path = (package / "meshes" / group / filename.removeprefix("${mesh_dir}/")).resolve()
            mesh_path.relative_to((package / "meshes" / group).resolve())
            raw = mesh_path.read_bytes()
            scale = _vector(mesh.get("scale"), "1 1 1")
            points = _stl_vertices(raw) * scale
            pose = poses[link_name] @ _origin(visual.find("origin"))
            points = points @ pose[:3, :3].T + pose[:3, 3]
            all_points.append(points)
            radius = np.linalg.norm(points[:, :2], axis=1)
            parts.append(
                {
                    "link": link_name,
                    "mesh": str(mesh_path),
                    "mesh_sha256": hashlib.sha256(raw).hexdigest(),
                    "min_xyz_m": points.min(axis=0).tolist(),
                    "max_xyz_m": points.max(axis=0).tolist(),
                    "max_planar_radius_m": float(radius.max()),
                }
            )
    points = np.concatenate(all_points)
    low, high = points.min(axis=0), points.max(axis=0)
    radius = float(np.linalg.norm(points[:, :2], axis=1).max())
    convex_footprint = _convex_hull_xy(points)
    navigation_footprint = _support_envelope_xy(points)
    arm_extension = 0.04
    navigation_radius = math.ceil(radius * 100.0) / 100.0
    extended_work_radius = math.ceil((radius + arm_extension) * 100.0) / 100.0
    return {
        "scope": "Original complete vendor visual model at an explicit stowed joint pose",
        "frame": "base_footprint",
        "source_sha256": {
            group: hashlib.sha256(path.read_bytes()).hexdigest() for group, path in sources.items()
        },
        "joint_positions_rad_or_m": joint_positions,
        "min_xyz_m": low.tolist(),
        "max_xyz_m": high.tolist(),
        "length_width_height_m": (high - low).tolist(),
        "original_max_planar_radius_m": radius,
        "original_convex_footprint_xy_m": convex_footprint,
        "navigation_footprint_xy_m": navigation_footprint,
        "navigation_footprint_method": "16-sided support envelope containing every original model vertex",
        "user_arm_extension_m": arm_extension,
        "arm_extension_in_navigation_footprint": False,
        "recommended_navigation_robot_radius_m": navigation_radius,
        "extended_arm_work_envelope_radius_m": extended_work_radius,
        "retained_footprint_padding_m": 0.05,
        "effective_navigation_radius_with_padding_m": navigation_radius + 0.05,
        "parts": parts,
        "nav2_footprint_deployed": False,
        "no_robot_commands_sent": True,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("package", type=Path)
    parser.add_argument("--joint-positions", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    positions = dict(JOINT_DEFAULTS)
    if args.joint_positions:
        positions.update(json.loads(args.joint_positions.read_text(encoding="utf-8")))
    result = measure(args.package, positions)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({key: value for key, value in result.items() if key != "parts"}, indent=2))
