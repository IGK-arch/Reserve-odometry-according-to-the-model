#!/usr/bin/env python3
"""Build and verify a deterministic, source-only hackathon submission archive.

The release is selected by an allowlist. Raw rosbags, dataset archives, generated
binary viewers, local build trees and Python caches are never traversed into the
ZIP. Run from any directory with ``python tools/make_release.py``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
RELEASE_DIR = ROOT / "release"
ARCHIVE = RELEASE_DIR / "tram_odometry_source.zip"
MANIFEST = RELEASE_DIR / "manifest.json"
SHA256SUMS = RELEASE_DIR / "SHA256SUMS.txt"
ZIP_TIME = (1980, 1, 1, 0, 0, 0)
MAX_FILE_SIZE = 8 * 1024 * 1024

# Explicit, reviewable release scope. No glob targets dataset/, build/, install/,
# log/, release/, generated visualisation HTML, or raw telemetry exports.
ALLOWLIST = (
    ".gitattributes",
    ".gitignore",
    "README.md",
    "LICENSE",
    "CMakeLists.txt",
    "docs/*.md",
    "docs/validation_metrics.png",
    "docs/audit_generalization.png",
    "docs/route_map_preview.png",
    "docs/reference_comparison.png",
    "docs/round2_comparison.png",
    "ros2_ws/src/tram_vehicle_msgs/CMakeLists.txt",
    "ros2_ws/src/tram_vehicle_msgs/package.xml",
    "ros2_ws/src/tram_vehicle_msgs/msg/*.msg",
    "ros2_ws/src/tram_odometry/CMakeLists.txt",
    "ros2_ws/src/tram_odometry/package.xml",
    "ros2_ws/src/tram_odometry/README.md",
    "ros2_ws/src/tram_odometry/include/tram_odometry/*.hpp",
    "ros2_ws/src/tram_odometry/src/*.cpp",
    "ros2_ws/src/tram_odometry/test/*.cpp",
    "ros2_ws/src/tram_odometry/config/*.yaml",
    "ros2_ws/src/tram_odometry/launch/*.py",
    "ros2_ws/src/tram_odometry/assets/*.csv",
    "ros2_ws/src/tram_odometry/assets/*.json",
    "tools/*.py",
    "tools/*.sh",
    "tools/*.md",
    "tools/docker/Dockerfile",
    "tools/docker/*.md",
    "tools/split_manifest.json",
    "evaluation/*.py",
    "evaluation/requirements.txt",
    "evaluation/replay_cli.cpp",
    "evaluation/navigation_replay.cpp",
    "evaluation/tests/*.py",
    "evaluation/results/**/*.json",
    "evaluation/results/**/*.md",
    "evaluation/results/**/*.patch",
    "evaluation/results/**/*.py",
    "evaluation/results/**/*.cpp",
    "evaluation/results/**/*.log",
    "evaluation/*.patch",
    "evaluation/*study*.cpp",
    "evaluation/results/**/*.csv",
    "evaluation/validation_baseline.csv",
    "evaluation/holdout_baseline.csv",
    # Historical teammate audit of the pre-Navigation deployed mode.
    # Current measured improvements are versioned under evaluation/results/.
    "evaluation/validation_core_deployed.csv",
    "evaluation/holdout_core_deployed.csv",
    "evaluation/validation_distance_deployed.csv",
    "evaluation/holdout_distance_deployed.csv",
    "evaluation/validation_core_rawfreeze_fix.csv",
    "evaluation/holdout_core_rawfreeze_fix.csv",
    "evaluation/validation_core_table_optional.csv",
    "evaluation/holdout_core_table_optional.csv",
    "evaluation/validation_distance_rawfreeze_fix.csv",
    "evaluation/holdout_distance_rawfreeze_fix.csv",
    "evaluation/validation_distance_table_optional.csv",
    "evaluation/holdout_distance_table_optional.csv",
    "evaluation/audit_*.md",
    "evaluation/audit_*.json",
    "evaluation/audit_*.csv",
    "evaluation/covariance_scale_audit.json",
    "evaluation/validation_position_table.csv",
    "evaluation/validation_position_proxy.csv",
    "evaluation/holdout_position_deployed_proxy.csv",
    "evaluation/holdout_position_proxy.csv",
    "evaluation/validation_stop_landmarks.csv",
    "evaluation/holdout_stop_landmarks.csv",
    "evaluation/table_blackout_validation.json",
    "evaluation/wheel_calibration_train.json",
    "evaluation/drive_calibration_train.json",
    "evaluation/ros_smoke/anchored_20260926_231625_pnTr/report.json",
    "evaluation/ros_smoke/anchored_20260926_231625_pnTr/monitor.log",
    "evaluation/ros_smoke/anchored_20260926_231211_B7H1/report.json",
    "evaluation/ros_smoke/anchored_20260926_231211_B7H1/monitor.log",
    "evaluation/ros_smoke/anchored_20260926_232018_2DFf/report.json",
    "evaluation/ros_smoke/anchored_20260926_232018_2DFf/monitor.log",
    "evaluation/ros_smoke/anchored_20260926_233338_Uhja/report.json",
    "evaluation/ros_smoke/anchored_20260926_233338_Uhja/monitor.log",
    "evaluation/ros_smoke/anchored_20260926_233338_Uhja/accuracy.json",
    "evaluation/ros_smoke/anchored_20260926_233338_Uhja/samples.csv",
    "evaluation/ros_smoke/anchored_20260927_000129_myd6/report.json",
    "evaluation/ros_smoke/anchored_20260927_000129_myd6/monitor.log",
    "evaluation/ros_smoke/anchored_20260927_000129_myd6/accuracy.json",
    "evaluation/ros_smoke/anchored_20260927_000129_myd6/samples.csv",
    "evaluation/ros_smoke/anchored_20260927_003136_Gg9N/report.json",
    "evaluation/ros_smoke/anchored_20260927_003136_Gg9N/monitor.log",
    "evaluation/ros_smoke/no-gnss_20260926_230806_jO22/report.json",
    "evaluation/ros_smoke/no-gnss_20260926_230806_jO22/monitor.log",
    "analysis/README.md",
    "analysis/audit_*.py",
    "analysis/audit_*.json",
    "analysis/catalog/README.md",
    "analysis/catalog/*.py",
    "analysis/routes/README.md",
    "analysis/routes/*.py",
    "analysis/signals/NOTCH_README.md",
    "analysis/signals/*.py",
    "analysis/viewer/*.py",
    "analysis/viewer/tram_sensor_cases.template.html",
    "analysis/simulator/*.py",
    "analysis/simulator/*.js",
    "analysis/simulator/*.css",
    "analysis/simulator/*.html",
    "analysis/simulator/*.md",
)

REQUIRED = (
    ".gitattributes",
    ".gitignore",
    "README.md",
    "ros2_ws/src/tram_odometry/package.xml",
    "ros2_ws/src/tram_odometry/src/odometry_node.cpp",
    "ros2_ws/src/tram_odometry/src/navigation.cpp",
    "ros2_ws/src/tram_odometry/include/tram_odometry/navigation.hpp",
    "ros2_ws/src/tram_odometry/include/tram_odometry/route_map.hpp",
    "ros2_ws/src/tram_odometry/include/tram_odometry/elevation_profile.hpp",
    "ros2_ws/src/tram_odometry/assets/official_elevation.csv",
    "CMakeLists.txt",
    "ros2_ws/src/tram_vehicle_msgs/package.xml",
    "ros2_ws/src/tram_odometry/assets/route_map.csv",
    "ros2_ws/src/tram_odometry/assets/drive_accel_table.csv",
    "tools/make_release.py",
    "docs/JURY_CHECK.md",
    "docs/RESULTS.md",
    "docs/VALIDATION_AUDIT.md",
)

FORBIDDEN_PARTS = frozenset(
    {"dataset", "build", "install", "log", "release", "__pycache__", ".git", ".venv", "venv"}
)
FORBIDDEN_SUFFIXES = frozenset(
    {".db3", ".mcap", ".bag", ".zip", ".7z", ".rar", ".exe", ".dll", ".pdb", ".pyc", ".npz", ".npy"}
)
FORBIDDEN_MAGIC = (b"SQLite format 3\x00", b"MCAP0", b"PK\x03\x04", b"7z\xbc\xaf\x27\x1c", b"MZ")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def paths_to_release() -> list[Path]:
    paths: set[Path] = set()
    for pattern in ALLOWLIST:
        for path in ROOT.glob(pattern):
            if path.is_file():
                paths.add(path)
    relative = sorted(path.relative_to(ROOT) for path in paths)
    missing = set(REQUIRED) - {path.as_posix() for path in relative}
    if missing:
        raise ValueError(f"Required release files are missing: {sorted(missing)}")
    for path in relative:
        validate_path(path)
    return relative


def validate_name(relative: Path) -> None:
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"Unsafe release path: {relative}")
    if any(part.lower() in FORBIDDEN_PARTS for part in relative.parts):
        raise ValueError(f"Forbidden directory in release: {relative}")
    if relative.suffix.lower() in FORBIDDEN_SUFFIXES:
        raise ValueError(f"Forbidden file type in release: {relative}")


def validate_path(relative: Path) -> None:
    validate_name(relative)
    source = ROOT / relative
    if source.is_symlink() or not source.resolve().is_relative_to(ROOT.resolve()):
        raise ValueError(f"Symlink or path outside repository: {relative}")
    if source.stat().st_size > MAX_FILE_SIZE:
        raise ValueError(f"File exceeds {MAX_FILE_SIZE} byte source limit: {relative}")


def read_source(relative: Path) -> bytes:
    validate_path(relative)
    data = (ROOT / relative).read_bytes()
    if len(data) > MAX_FILE_SIZE:
        raise ValueError(f"File grew beyond {MAX_FILE_SIZE} byte source limit: {relative}")
    if any(data.startswith(signature) for signature in FORBIDDEN_MAGIC):
        raise ValueError(f"Raw archive, rosbag or executable magic in {relative}")
    return data


def manifest_bytes(files: dict[str, bytes]) -> bytes:
    document = {
        "format": "tram_odometry_source_release_v1",
        "description": "ROS 2 source, derived route assets, documentation, evaluation and visualisation code; no raw bags",
        "files": [
            {"path": path, "size": len(data), "sha256": sha256(data)}
            for path, data in sorted(files.items())
        ],
    }
    return (json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8")


def zip_info(name: str) -> zipfile.ZipInfo:
    info = zipfile.ZipInfo(name, date_time=ZIP_TIME)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    info.create_system = 3
    return info


def verify_archive(archive: Path) -> tuple[int, int]:
    with zipfile.ZipFile(archive, "r") as package:
        members = package.namelist()
        if len(members) != len(set(members)):
            raise ValueError("Duplicate ZIP member")
        if "RELEASE_MANIFEST.json" not in members:
            raise ValueError("Missing embedded manifest")
        document = json.loads(package.read("RELEASE_MANIFEST.json"))
        expected = {entry["path"]: entry for entry in document["files"]}
        if set(members) != set(expected) | {"RELEASE_MANIFEST.json"}:
            raise ValueError("ZIP members differ from embedded manifest")
        total = 0
        for member in members:
            if member == "RELEASE_MANIFEST.json":
                continue
            relative = Path(member)
            validate_name(relative)
            data = package.read(member)
            if any(data.startswith(signature) for signature in FORBIDDEN_MAGIC):
                raise ValueError(f"Forbidden magic in ZIP member: {member}")
            if len(data) != expected[member]["size"] or sha256(data) != expected[member]["sha256"]:
                raise ValueError(f"Checksum mismatch: {member}")
            total += len(data)
        if package.testzip() is not None:
            raise ValueError("ZIP CRC verification failed")
        return len(expected), total


def build_archive() -> tuple[int, int, str]:
    files = {path.as_posix(): read_source(path) for path in paths_to_release()}
    embedded_manifest = manifest_bytes(files)
    RELEASE_DIR.mkdir(exist_ok=True)
    with zipfile.ZipFile(ARCHIVE, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as package:
        for name, data in sorted(files.items()):
            package.writestr(zip_info(name), data, compress_type=zipfile.ZIP_DEFLATED, compresslevel=9)
        package.writestr(zip_info("RELEASE_MANIFEST.json"), embedded_manifest, compress_type=zipfile.ZIP_DEFLATED, compresslevel=9)
    count, total = verify_archive(ARCHIVE)
    archive_digest = sha256(ARCHIVE.read_bytes())
    external = json.loads(embedded_manifest)
    external["archive"] = {"path": ARCHIVE.name, "size": ARCHIVE.stat().st_size, "sha256": archive_digest}
    manifest_data = (json.dumps(external, indent=2, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8")
    MANIFEST.write_bytes(manifest_data)
    SHA256SUMS.write_text(
        f"{archive_digest}  {ARCHIVE.name}\n{sha256(manifest_data)}  {MANIFEST.name}\n", encoding="ascii"
    )
    return count, total, archive_digest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-only", action="store_true", help="Verify an existing release archive")
    args = parser.parse_args()
    if args.verify_only:
        count, total = verify_archive(ARCHIVE)
        digest = sha256(ARCHIVE.read_bytes())
        external = json.loads(MANIFEST.read_text(encoding="utf-8"))
        if external["archive"]["sha256"] != digest:
            raise ValueError("Archive digest differs from external manifest")
    else:
        count, total, digest = build_archive()
    print(json.dumps({
        "archive": str(ARCHIVE), "files": count, "source_bytes": total,
        "zip_bytes": ARCHIVE.stat().st_size, "sha256": digest,
    }, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        print(f"release failed: {exc}", file=sys.stderr)
        sys.exit(1)
