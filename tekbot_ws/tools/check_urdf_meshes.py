#!/usr/bin/env python3
"""Check mesh file coverage for an URDF/Xacro file.

Usage:
  python3 tools/check_urdf_meshes.py \
    --urdf src/muto_bringup/config/urdf/muto_rs.urdf.xacro \
    --workspace /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/tekbot_ws
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path


def package_uri_to_path(uri: str, workspace: Path) -> Path | None:
    prefix = "package://"
    if not uri.startswith(prefix):
        return None

    rest = uri[len(prefix):]
    parts = rest.split("/", 1)
    if len(parts) != 2:
        return None

    package_name, rel_path = parts
    return workspace / "src" / package_name / rel_path


def extract_mesh_uris(urdf_file: Path) -> list[str]:
    root = ET.parse(urdf_file).getroot()
    uris: list[str] = []
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "").strip()
        if filename:
            uris.append(filename)
    return uris


def main() -> int:
    parser = argparse.ArgumentParser(description="Check missing mesh files referenced by URDF/Xacro")
    parser.add_argument("--urdf", required=True, help="Path to URDF/Xacro")
    parser.add_argument("--workspace", default=str(Path.cwd()), help="Workspace root containing src/")
    args = parser.parse_args()

    workspace = Path(args.workspace).resolve()
    urdf_file = Path(args.urdf).resolve()

    if not urdf_file.exists():
        print(f"ERROR: URDF file not found: {urdf_file}")
        return 2

    uris = extract_mesh_uris(urdf_file)
    if not uris:
        print("No <mesh> entries found.")
        return 0

    counts = Counter(uris)
    resolved: list[tuple[str, Path | None]] = []
    for uri in sorted(counts.keys()):
        resolved.append((uri, package_uri_to_path(uri, workspace)))

    missing: list[tuple[str, Path | None]] = []
    present: list[tuple[str, Path | None]] = []
    external: list[tuple[str, Path | None]] = []

    for uri, path in resolved:
        if path is None:
            external.append((uri, None))
            continue
        if path.exists():
            present.append((uri, path))
        else:
            missing.append((uri, path))

    print("=== URDF Mesh Audit ===")
    print(f"URDF: {urdf_file}")
    print(f"Workspace: {workspace}")
    print(f"Total mesh refs: {len(uris)}")
    print(f"Unique mesh refs: {len(counts)}")
    print(f"Present: {len(present)}")
    print(f"Missing: {len(missing)}")
    print(f"Non-package URIs: {len(external)}")

    if missing:
        print("\nMissing meshes:")
        for uri, path in missing:
            print(f" - {uri}")
            print(f"   -> expected: {path}")

    if external:
        print("\nNon-package URIs:")
        for uri, _ in external:
            print(f" - {uri}")

    if not missing and not external:
        print("\nOK: all mesh references are resolvable.")
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
