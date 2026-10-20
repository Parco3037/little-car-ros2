"""Build a mesh-free visualization URDF while preserving robot kinematics.

The generated model replaces STL geometry with primitive bounding volumes.  It
does not modify links, joints, limits, inertials, or transmissions.  This makes
the result suitable for a lightweight RobotModel display while keeping the
vendor URDF as the rollback source of truth.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


_FLOAT_PATTERN = re.compile(
    r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_vector(value: str | None, default: tuple[float, float, float]) -> list[float]:
    if value is None:
        return list(default)
    parts = [float(part) for part in value.split()]
    if len(parts) != 3:
        raise ValueError(f"expected three values, got {value!r}")
    return parts


def _format_vector(values: list[float]) -> str:
    cleaned = [0.0 if abs(value) < 5e-13 else value for value in values]
    return " ".join(f"{value:.9g}" for value in cleaned)


def _rotation_from_rpy(rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def _matvec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def _rpy_from_rotation(matrix: list[list[float]]) -> list[float]:
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return [roll, pitch, yaw]


def _binary_stl_bounds(data: bytes) -> tuple[list[float], list[float]] | None:
    if len(data) < 84:
        return None
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if triangle_count == 0 or expected_size > len(data):
        return None
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for index in range(triangle_count):
        values = struct.unpack_from("<12f", data, 84 + index * 50)
        for offset in (3, 6, 9):
            for axis in range(3):
                value = float(values[offset + axis])
                minimum[axis] = min(minimum[axis], value)
                maximum[axis] = max(maximum[axis], value)
    return minimum, maximum


def _ascii_stl_bounds(data: bytes) -> tuple[list[float], list[float]]:
    text = data.decode("utf-8", errors="ignore")
    vertices: list[list[float]] = []
    for line in text.splitlines():
        if line.lstrip().lower().startswith("vertex "):
            numbers = [float(item) for item in _FLOAT_PATTERN.findall(line)]
            if len(numbers) >= 3:
                vertices.append(numbers[-3:])
    if not vertices:
        raise ValueError("STL contains no readable vertices")
    return (
        [min(vertex[axis] for vertex in vertices) for axis in range(3)],
        [max(vertex[axis] for vertex in vertices) for axis in range(3)],
    )


def read_stl_bounds(path: Path) -> tuple[list[float], list[float]]:
    data = path.read_bytes()
    binary = _binary_stl_bounds(data)
    if binary is not None:
        return binary
    return _ascii_stl_bounds(data)


def _resolve_mesh(filename: str, package_root: Path) -> Path:
    prefix = "package://landerpi_description/"
    if filename.startswith(prefix):
        return package_root / filename[len(prefix):]
    if filename.startswith("file://"):
        return Path(filename[7:])
    path = Path(filename)
    if path.is_absolute():
        return path
    raise ValueError(f"unsupported mesh URI: {filename}")


def _origin(parent: ET.Element) -> ET.Element:
    element = parent.find("origin")
    if element is None:
        element = ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        parent.insert(0, element)
    return element


def _cylinder_axis_rotation(axis: int) -> list[list[float]]:
    # URDF cylinders use local Z as their length axis.
    if axis == 0:
        return _rotation_from_rpy([0.0, math.pi / 2.0, 0.0])
    if axis == 1:
        return _rotation_from_rpy([-math.pi / 2.0, 0.0, 0.0])
    return _rotation_from_rpy([0.0, 0.0, 0.0])


def _replace_geometry(
    owner: ET.Element,
    link_name: str,
    package_root: Path,
    bounds_cache: dict[Path, tuple[list[float], list[float]]],
) -> dict[str, object] | None:
    geometry = owner.find("geometry")
    if geometry is None:
        return None
    mesh = geometry.find("mesh")
    if mesh is None:
        return None
    filename = mesh.get("filename")
    if not filename:
        raise ValueError(f"mesh in link {link_name!r} has no filename")
    mesh_path = _resolve_mesh(filename, package_root).resolve()
    if not mesh_path.is_file():
        raise FileNotFoundError(mesh_path)
    minimum, maximum = bounds_cache.setdefault(mesh_path, read_stl_bounds(mesh_path))
    scale = _parse_vector(mesh.get("scale"), (1.0, 1.0, 1.0))
    scaled_min = [min(minimum[i] * scale[i], maximum[i] * scale[i]) for i in range(3)]
    scaled_max = [max(minimum[i] * scale[i], maximum[i] * scale[i]) for i in range(3)]
    center = [(scaled_min[i] + scaled_max[i]) / 2.0 for i in range(3)]
    extent = [max(1e-5, scaled_max[i] - scaled_min[i]) for i in range(3)]

    origin = _origin(owner)
    xyz = _parse_vector(origin.get("xyz"), (0.0, 0.0, 0.0))
    rpy = _parse_vector(origin.get("rpy"), (0.0, 0.0, 0.0))
    rotation = _rotation_from_rpy(rpy)
    rotated_center = _matvec(rotation, center)
    origin.set("xyz", _format_vector([xyz[i] + rotated_center[i] for i in range(3)]))

    geometry.remove(mesh)
    primitive = "box"
    if "wheel" in link_name.lower():
        axis = min(range(3), key=lambda index: extent[index])
        perpendicular = [extent[index] for index in range(3) if index != axis]
        radius = max(perpendicular) / 2.0
        cylinder = ET.SubElement(geometry, "cylinder")
        cylinder.set("radius", f"{radius:.9g}")
        cylinder.set("length", f"{extent[axis]:.9g}")
        rotation = _matmul(rotation, _cylinder_axis_rotation(axis))
        origin.set("rpy", _format_vector(_rpy_from_rotation(rotation)))
        primitive = "cylinder"
    else:
        box = ET.SubElement(geometry, "box")
        box.set("size", _format_vector(extent))
        origin.set("rpy", _format_vector(rpy))

    return {
        "link": link_name,
        "source": filename,
        "primitive": primitive,
        "size": extent,
    }


def _joint_signature(root: ET.Element) -> list[str]:
    return [ET.tostring(joint, encoding="unicode") for joint in root.findall("joint")]


def _inertial_signature(root: ET.Element) -> list[tuple[str, str]]:
    signature: list[tuple[str, str]] = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is not None:
            signature.append(
                (link.get("name", ""), ET.tostring(inertial, encoding="unicode"))
            )
    return signature


def _transmission_signature(root: ET.Element) -> list[str]:
    return [
        ET.tostring(transmission, encoding="unicode")
        for transmission in root.findall("transmission")
    ]


def build_lightweight_urdf(
    source_path: Path,
    output_path: Path,
    package_root: Path,
    *,
    replace_collisions: bool = True,
) -> dict[str, object]:
    tree = ET.parse(source_path)
    root = tree.getroot()
    before_links = [link.get("name", "") for link in root.findall("link")]
    before_joints = _joint_signature(root)
    before_inertials = _inertial_signature(root)
    before_transmissions = _transmission_signature(root)
    bounds_cache: dict[Path, tuple[list[float], list[float]]] = {}
    replacements: list[dict[str, object]] = []

    for link in root.findall("link"):
        link_name = link.get("name", "")
        owners = list(link.findall("visual"))
        if replace_collisions:
            owners.extend(link.findall("collision"))
        for owner in owners:
            replacement = _replace_geometry(owner, link_name, package_root, bounds_cache)
            if replacement is not None:
                replacement["role"] = owner.tag
                replacements.append(replacement)

    after_links = [link.get("name", "") for link in root.findall("link")]
    after_joints = _joint_signature(root)
    after_inertials = _inertial_signature(root)
    after_transmissions = _transmission_signature(root)
    if before_links != after_links:
        raise RuntimeError("link list changed while replacing geometry")
    if before_joints != after_joints:
        raise RuntimeError("joint definitions changed while replacing geometry")
    if before_inertials != after_inertials:
        raise RuntimeError("inertial definitions changed while replacing geometry")
    if before_transmissions != after_transmissions:
        raise RuntimeError("transmission definitions changed while replacing geometry")
    if replace_collisions and root.findall(".//mesh"):
        raise RuntimeError("mesh references remain in mesh-free mode")

    ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return {
        "source": str(source_path),
        "source_sha256": _sha256(source_path),
        "output": str(output_path),
        "output_sha256": _sha256(output_path),
        "links": len(before_links),
        "joints": len(before_joints),
        "inertials": len(before_inertials),
        "transmissions": len(before_transmissions),
        "replacements": len(replacements),
        "mesh_files_read": len(bounds_cache),
        "remaining_meshes": len(root.findall(".//mesh")),
        "details": replacements,
    }


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="expanded vendor URDF")
    parser.add_argument("output", type=Path, help="mesh-free URDF to create")
    parser.add_argument(
        "--package-root",
        required=True,
        type=Path,
        help="landerpi_description package directory",
    )
    parser.add_argument(
        "--keep-collision-meshes",
        action="store_true",
        help="replace visual meshes only (not fully mesh-free)",
    )
    parser.add_argument("--manifest", type=Path, help="optional JSON evidence file")
    parsed = parser.parse_args(args)
    manifest = build_lightweight_urdf(
        parsed.source,
        parsed.output,
        parsed.package_root,
        replace_collisions=not parsed.keep_collision_meshes,
    )
    payload = json.dumps(manifest, ensure_ascii=False, indent=2)
    if parsed.manifest:
        parsed.manifest.parent.mkdir(parents=True, exist_ok=True)
        parsed.manifest.write_text(payload + "\n", encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
